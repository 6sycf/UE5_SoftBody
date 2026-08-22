// MySoftBodyMeshComponent.cpp

#include "MySoftBodyMeshComponent.h"
#include "Engine/World.h"
#include "DrawDebugHelpers.h"
#include "Math/RandomStream.h"
#include "DynamicMesh/DynamicMeshAttributeSet.h"
#include "GeometryScript/MeshAssetFunctions.h"
#include "GeometryScript/MeshBasicEditFunctions.h"
#include "OpenHapticsComponent.h"
#include "SoftBodyColliderComponent.h"
#include "DynamicMesh/DynamicMeshAABBTree3.h"
#include "GeometryScript/MeshRepairFunctions.h"
#include "DynamicMesh/MeshTangents.h"
#include "EngineUtils.h"
#include "MeshSimplification.h"
#include "SoftBodyComputeShader.h"
#include "RenderGraphBuilder.h"
#include "RenderGraphUtils.h"
#include "RHIGPUReadback.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Runtime/GeometryFramework/Private/Components/DynamicMeshSceneProxy.h"
#include "SceneViewExtension.h"
#include "SceneView.h"
#include "EngineModule.h"
#include "GlobalDistanceFieldParameters.h"
#include "RHIStaticStates.h"
#include "SceneRendering.h"


// =========================================================
// 全局距离场 (GDF) 碰撞视图扩展
// 在场景渲染管线末尾 (PostRenderViewFamily_RenderThread) 提取稳定的 pooled texture，
// 供独立模拟 RDG graph 在下一帧采样。QueueTextureExtraction 保证纹理不会被池复用回收。
// =========================================================
struct FSoftBodyGDFCacheData
{
    FGlobalDistanceFieldParameterData GDFData;
    TRefCountPtr<IPooledRenderTarget> PageAtlasPooled;
    TRefCountPtr<IPooledRenderTarget> PageTablePooled;
    FVector PreViewTranslation = FVector::ZeroVector;
    bool bValid = false;
};

class UMySoftBodyMeshComponent;

class FSoftBodyGDFViewExtension : public FSceneViewExtensionBase
{
public:
    FSoftBodyGDFViewExtension(const FAutoRegister& AutoRegister, UMySoftBodyMeshComponent* InOwner)
        : FSceneViewExtensionBase(AutoRegister)
        , Owner(InOwner)
    {}

    virtual void PostRenderViewFamily_RenderThread(FRDGBuilder& GraphBuilder, FSceneViewFamily& InViewFamily) override
    {
        if (!Owner)
        {
            return;
        }

        const FSceneView* SceneView = InViewFamily.Views.Num() > 0 ? InViewFamily.Views[0] : nullptr;
        if (!SceneView || !SceneView->bIsViewInfo)
        {
            return;
        }

        const FViewInfo& View = static_cast<const FViewInfo&>(*SceneView);
        if (!View.GlobalDistanceFieldInfo.bInitialized)
        {
            return;
        }

        Owner->ExtractGDF(GraphBuilder, View);
    }

private:
    UMySoftBodyMeshComponent* Owner = nullptr;
};

// =========================================================
// 通用 GPU 回读辅助函数
// =========================================================
template<typename T>
void ProcessGPUReadback(
    // 双缓冲数组
    TArray<T>& CachedData,
    TArray<T>& ReadyData,
    std::atomic<bool>& bNewDataReady,
    // Readback 对象
    FRHIGPUBufferReadback*& Readback,
    bool& bReadbackPending,
    // Buffer 引用
    const TRefCountPtr<FRDGPooledBuffer>& PooledBuffer,
    // 数据大小
    int32 ElementCount,
    const TCHAR* ReadbackName)
{
    // 1. 交换缓冲
    if (bNewDataReady.load())
    {
        Swap(CachedData, ReadyData);
        bNewDataReady.store(false);
    }

    // 2. 检查 Readback 完成
    if (bReadbackPending && Readback && Readback->IsReady())
    {
        CachedData.SetNum(ElementCount);

        FRHIGPUBufferReadback* ReadbackRef = Readback;
        T* CachePtr = CachedData.GetData();
        int32 DataSize = ElementCount * sizeof(T);
        std::atomic<bool>* FlagPtr = &bNewDataReady;

        ENQUEUE_RENDER_COMMAND(LockReadback)(
            [ReadbackRef, CachePtr, DataSize, FlagPtr](FRHICommandListImmediate& RHICmdList)
            {
                const void* Data = ReadbackRef->Lock(DataSize);
                if (Data) FMemory::Memcpy(CachePtr, Data, DataSize);
                ReadbackRef->Unlock();
                FlagPtr->store(true);
            }
        );
        bReadbackPending = false;
    }

    // 3. 发起新的 Readback
    if (!bReadbackPending && PooledBuffer.IsValid())
    {
        int32 DataSize = ElementCount * sizeof(T);
        TRefCountPtr<FRDGPooledBuffer> BufferRef = PooledBuffer;
        FRHIGPUBufferReadback** ReadbackPtr = &Readback;

        ENQUEUE_RENDER_COMMAND(ReadbackData)(
            [BufferRef, DataSize, ReadbackPtr, ReadbackName](FRHICommandListImmediate& RHICmdList)
            {
                if (!*ReadbackPtr)
                    *ReadbackPtr = new FRHIGPUBufferReadback(ReadbackName);
                (*ReadbackPtr)->EnqueueCopy(RHICmdList, BufferRef->GetRHI(), DataSize);
            }
        );
        bReadbackPending = true;
    }
}

#define DEBUG_DRAW_CONSTRAINTS

// =========================================================
// 构造函数 & 析构函数
// =========================================================

UMySoftBodyMeshComponent::UMySoftBodyMeshComponent(const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer)
{
    PrimaryComponentTick.bCanEverTick = true;
    bTickInEditor = true;

    SourceStaticMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("SourceStaticMesh"));
    SourceStaticMesh->SetupAttachment(this);
    SourceStaticMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    SourceStaticMesh->SetCollisionProfileName(UCollisionProfile::NoCollision_ProfileName);
    SetGenerateOverlapEvents(false);
    
    bShowStaticMesh = false;
    bSimulate = true;
    bEnableHapticInteraction = false;
    SubstepTime = 0.005f;
    ConstraintIterations = 8;
    
    bWorldCollision = true;
    CollisionFriction = 0.2f;
    CollisionRestitution = 0.2f;
    bShow_Normals = false;
    ParticleMass = 1.0f;
    ParticleRadius = 5.0f;
 
    ClothGravityScale = 1.0f;
    ClothForce = FVector::ZeroVector;
    
    
    VolPressure_Coefficient = 100.0f; 

    clothStateExists = false;
    ProxyParticleCount = 0;
    ProxyTriCount = 0;
    ParticleRadiusSq = 0.0f;
    
    ProxyParticleCount = 0;
    HighResParticleCount = 0; // 【检查点 1】确保构造函数里赋初值为 0

}

UMySoftBodyMeshComponent::~UMySoftBodyMeshComponent()
{
    // 释放视图扩展引用 (销毁时自动注销)
    GDFViewExtension.Reset();
    GDFCache.Reset();
    ReleaseGPUResources();
}

// =========================================================
// 生命周期事件
// =========================================================

void UMySoftBodyMeshComponent::OnRegister()
{
    Super::OnRegister();
    UWorld* World = GetWorld();
    if (World && World->IsGameWorld())
    {
        BuildClothState();

        // 创建 GDF 缓存 + 碰撞视图扩展 (每个组件一个)
        if (!GDFCache.IsValid())
        {
            GDFCache = MakeShared<FSoftBodyGDFCacheData, ESPMode::ThreadSafe>();
        }
        if (!GDFViewExtension.IsValid())
        {
            GDFViewExtension = FSceneViewExtensions::NewExtension<FSoftBodyGDFViewExtension>(this);
        }
    }
    else
    {
        if (SourceStaticMesh) SourceStaticMesh->SetVisibility(true);
    }
}

void UMySoftBodyMeshComponent::BeginPlay()
{
    Super::BeginPlay();
    // [新增] 自动寻找场景里的触觉组件
    // 如果在编辑器里没有手动赋值，游戏开始时自动去场景里找一个
    if (!HapticsComponent)
    {
        UWorld* World = GetWorld();
        if (World)
        {
            // 遍历场景里所有的 Actor
            for (TActorIterator<AActor> It(World); It; ++It)
            {
                // 看看这个 Actor 有没有 OpenHaptics 组件
                UOpenHapticsComponent* FoundComp = It->FindComponentByClass<UOpenHapticsComponent>();
                if (FoundComp)
                {
                    HapticsComponent = FoundComp;
                    UE_LOG(LogTemp, Log, TEXT("SoftBody: Successfully linked with Haptics Device on Actor: %s"), *It->GetName());
                    break; // 找到一个就停 (假设场景里只有一个设备)
                }
            }
        }
    }
    
    if (!HapticsComponent)
    {
        UE_LOG(LogTemp, Warning, TEXT("SoftBody: No Haptics Component found! Interaction will be disabled."));
    }

    // 自动扫描场景里的动态碰撞体组件 (球/盒/胶囊)
    if (bUseDynamicColliders)
    {
        DynamicColliders.Empty();
        UWorld* World = GetWorld();
        if (World)
        {
            for (TActorIterator<AActor> It(World); It; ++It)
            {
                TArray<USoftBodyColliderComponent*> FoundColliders;
                It->GetComponents<USoftBodyColliderComponent>(FoundColliders);
                for (USoftBodyColliderComponent* Col : FoundColliders)
                {
                    if (Col)
                    {
                        DynamicColliders.AddUnique(Col);
                    }
                }
            }
        }
        UE_LOG(LogTemp, Log, TEXT("SoftBody: Auto-registered %d dynamic colliders."), DynamicColliders.Num());
    }
}

void UMySoftBodyMeshComponent::RegisterCollider(USoftBodyColliderComponent* Collider)
{
    if (Collider)
    {
        DynamicColliders.AddUnique(Collider);
    }
}

void UMySoftBodyMeshComponent::UnregisterCollider(USoftBodyColliderComponent* Collider)
{
    if (Collider)
    {
        DynamicColliders.Remove(Collider);
    }
}

void UMySoftBodyMeshComponent::ClearColliders()
{
    DynamicColliders.Empty();
}

void UMySoftBodyMeshComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
    /*FString Msg = FString::Printf(TEXT("Config Check: bSimulate=%d, bUseGPU=%d, clothStateExists=%d"), 
        bSimulate, bUseGPU, clothStateExists);
    GEngine->AddOnScreenDebugMessage(101, 0.0f, FColor::Red, Msg);*/
    if (!clothStateExists) return;
    double AllStartTime = FPlatformTime::Seconds();
    Dt = DeltaTime;
    St = FMath::Max(SubstepTime, 0.005f); // 防止步长过小卡死
    if (bSimulate)
    {
        if (bUseGPU)
        {
            // GPU 模式
            if (!bGPUResourcesInitialized) InitGPUResources();
            double StartTime = FPlatformTime::Seconds();
            // 更新棍子碰撞数据
            bHasStickInput = false;
            if (bEnableHapticInteraction && HapticsComponent)
            {
                FHapticToolState ToolState = HapticsComponent->GetHapticToolState();
                FTransform ToolTransform = HapticsComponent->GetComponentTransform();
                CachedStickStart = ToolTransform.TransformPosition(ToolState.StartPos);
                CachedStickEnd = ToolTransform.TransformPosition(ToolState.EndPos);
                CachedStickRadius = ToolState.Radius;
                bHasStickInput = true;
            }
            int32 MaxSubsteps = 4; //设置上限，防止帧率50hz以下之后会导致多次解算
            int32 SubstepCount = 0;
            At += Dt;
            while (At > St && SubstepCount < MaxSubsteps)
            {
                bool bIsLastSubstep = (At - St <= St) || (SubstepCount == MaxSubsteps - 1);
                DispatchGPUCompute(bIsLastSubstep);
                At -= St;
                SubstepCount++;
            }
            if (!bUpdateViaGPU)
            ReadbackGPUPositions();
            double EndTime = FPlatformTime::Seconds();
            float CostMS = (EndTime - StartTime) * 1000.0f;
            if (GEngine)
            {
                // 使用不同的 Key (998 和 997) 保证两行同时显示且不闪烁
                FString MsgA = FString::Printf(TEXT("A. TickUpdateComputer Cost: %.2f ms"), CostMS);
                GEngine->AddOnScreenDebugMessage(998, 0.0f, FColor::Cyan, MsgA);
            }
        }
        else
        {
            // CPU 模式：子步循环
            At += Dt;
            while (At > St)
            {
                SubstepSolve();
                At -= St;
            }
        }
        double StartTime = FPlatformTime::Seconds();
        TickUpdateCloth();
        double EndTime = FPlatformTime::Seconds();
        float CostMS = (EndTime - StartTime) * 1000.0f;

        // ==========================================
        // 打印两位嫌疑人的耗时
        // ==========================================
        if (GEngine)
        {
            FString MsgB = FString::Printf(TEXT("B. TickUpdate Cost: %.2f ms"), CostMS);
            GEngine->AddOnScreenDebugMessage(997, 0.0f, FColor::Magenta, MsgB);
        }
        
    }
    
    // 1. 检查资源是否初始化
    if (!bGPUResourcesInitialized) {
        GEngine->AddOnScreenDebugMessage(101, 0.0f, FColor::Red, TEXT("GPU Init FAILED"));
    }

    // 2. 检查 Shader 是否存在
    // 尝试获取一下 Shader 引用，看是不是空的
    auto ShaderMap = GetGlobalShaderMap(GMaxRHIFeatureLevel);
    TShaderMapRef<FIntegrateCS> ComputeShader(ShaderMap);
    
    if (ComputeShader.IsValid()) {
        GEngine->AddOnScreenDebugMessage(102, 0.0f, FColor::Green, TEXT("Shader Found OK"));
    } else {
        // 如果这里出现，说明 Shader 没被注册进库 -> 肯定是 LoadingPhase 的问题
        GEngine->AddOnScreenDebugMessage(102, 0.0f, FColor::Red, TEXT("Shader MISSING in GlobalMap"));
    }
    
    // 3. 检查回读数据
    if (ReadyGPUPositions.Num() > 0) {
        FString PosMsg = FString::Printf(TEXT("Readback Valid. P0: %s"), *ReadyGPUPositions[0].ToString());
        GEngine->AddOnScreenDebugMessage(103, 0.0f, FColor::Yellow, PosMsg);
    } else {
        GEngine->AddOnScreenDebugMessage(103, 0.0f, FColor::Red, TEXT("Readback Empty"));
    }
    double AllEndTime = FPlatformTime::Seconds();
    float AllCostMS = (AllEndTime - AllStartTime) * 1000.0f;
    if (GEngine)
    {
        FString MsgC = FString::Printf(TEXT("B. All Cost: %.2f ms"), AllCostMS);
        GEngine->AddOnScreenDebugMessage(996, 0.0f, FColor::Red, MsgC);
    }
    DrawDebugCoordinateSystem(GetWorld(), FVector::ZeroVector, FRotator::ZeroRotator, 100.0f, false, -1.0f, 100, 5.0f);
}

#if WITH_EDITOR
void UMySoftBodyMeshComponent::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
    Super::PostEditChangeProperty(PropertyChangedEvent);

    // 如果修改了属性，重置状态标志，提示用户重新构建
    // 或者你可以直接调用 BuildClothState() 来自动刷新
    if (clothStateExists)
    {
        // 简单处理：如果改了配置，就强制重新构建
        BuildClothState();
    }
}
#endif

// =========================================================
// 初始化核心 
// =========================================================


void UMySoftBodyMeshComponent::BuildClothState()
{
    // 1. 安全检查
    if (HasAnyFlags(RF_ClassDefaultObject)) return; // 忽略 CDO
    if (!SourceStaticMesh || !SourceStaticMesh->GetStaticMesh()) return;

    // 获取动态网格体对象
    UDynamicMesh* TargetDynMesh = GetDynamicMesh();
    if (!TargetDynMesh) return;

    // 2. [核心] 使用 GeometryScript 将 StaticMesh 转换为 DynamicMesh
    // 这个函数会自动处理顶点焊接 (Weld)，解决分裂问题
    EGeometryScriptOutcomePins Outcome;
    UGeometryScriptLibrary_StaticMeshFunctions::CopyMeshFromStaticMesh(
        SourceStaticMesh->GetStaticMesh(), 
        TargetDynMesh, 
        FGeometryScriptCopyMeshFromAssetOptions(), // 默认配置通常开启焊接
        FGeometryScriptMeshReadLOD(), 
        Outcome, 
        nullptr
    );

    if (Outcome != EGeometryScriptOutcomePins::Success) return;
    
    // 原理：在运行时只能读到分裂的渲染数据，必须手动把重合的点焊起来
    
    FGeometryScriptWeldEdgesOptions WeldOptions;
    WeldOptions.Tolerance = 0.1f; // 焊接阈值 (0.1cm)，重合的点会被合并
    WeldOptions.bOnlyUniquePairs = false; 
    UGeometryScriptLibrary_MeshRepairFunctions::WeldMeshEdges(TargetDynMesh, WeldOptions, nullptr);
    
    TargetDynMesh->GetMeshRef().CompactInPlace();
    bEnableComplexCollision = true;
    
    // =========================================================
    // [分支逻辑] 选择物理计算网格
    // =========================================================
    FDynamicMesh3* PhysicsMesh = nullptr;
    RenderMesh.Copy(TargetDynMesh->GetMeshRef());
    // 高模数组的分配与初始化
    HighResParticleCount = RenderMesh.VertexCount(); 
    
    // ==========================================
    // 动态创建 VAT 渲染目标
    // ==========================================
    if (HighResParticleCount > 0)
    {
        
        // ==========================================
        // 1. 动态创建 VAT 渲染目标 (画布)
        // ==========================================
        if (HighResParticleCount > 0)
        {
            int32 TexDimension = FMath::CeilToInt(FMath::Sqrt((float)HighResParticleCount));
            if (!PositionRT)
            {
                PositionRT = NewObject<UTextureRenderTarget2D>(this);
                PositionRT->bCanCreateUAV = true; 
                PositionRT->InitCustomFormat(TexDimension, TexDimension, PF_A32B32G32R32F, true);
                PositionRT->UpdateResourceImmediate(true);
            }
            if (!NormalRT)
            {
                NormalRT = NewObject<UTextureRenderTarget2D>(this);
                NormalRT->bCanCreateUAV = true; 
                NormalRT->InitCustomFormat(TexDimension, TexDimension, PF_A32B32G32R32F, true);
                NormalRT->UpdateResourceImmediate(true);
            }
        }
        // ==========================================
        // 2. 将 VAT UV 严格写入额外的 UV 通道 (UV Channel 1)
        // ==========================================
        UE::Geometry::FDynamicMesh3* Mesh = &GetDynamicMesh()->GetMeshRef();
        
        // 确保网格具有属性层
        if (!Mesh->HasAttributes())
        {
            Mesh->EnableAttributes();
        }

        // 确保模型至少有 2 个 UV 通道 (UV0 用于贴图，UV1 用于 VAT 数据)
        Mesh->Attributes()->SetNumUVLayers(2);
        UE::Geometry::FDynamicMeshUVOverlay* VAT_UVOverlay = Mesh->Attributes()->GetUVLayer(1);
        
        if (HighResParticleCount > 0 && VAT_UVOverlay)
        {
            VAT_UVOverlay->ClearElements(); // 清空可能存在的旧数据
            
            int32 TexDimension = FMath::CeilToInt(FMath::Sqrt((float)HighResParticleCount));
            
            // 安全地遍历三角形拓扑写入 UV，防止缝合边 (Seams) 导致的数据断层
            for (int32 Tid : Mesh->TriangleIndicesItr())
            {
                UE::Geometry::FIndex3i Tri = Mesh->GetTriangle(Tid);
                UE::Geometry::FIndex3i UVTri;
                
                for (int j = 0; j < 3; ++j)
                {
                    int32 Vid = Tri[j];
                    // 计算精确到像素中心的 UV 坐标
                    float U = (float(Vid % TexDimension) + 0.5f) / float(TexDimension);
                    float V = (float(Vid / TexDimension) + 0.5f) / float(TexDimension);
                    
                    // 追加 UV 元素并记录 ID
                    UVTri[j] = VAT_UVOverlay->AppendElement(FVector2f(U, V));
                }
                // 将生成的 UV 元素绑定到三角形的三个角
                VAT_UVOverlay->SetTriangle(Tid, UVTri);
            }
            NotifyMeshUpdated(); // 通知显卡更新
        }
        // ==========================================
        // 3. 构建动态材质并绑定贴图！
        // ==========================================
        int32 MatNum = SourceStaticMesh->GetNumMaterials();
        TArray<UMaterialInterface*> MatSet; // 准备强制覆盖的数组

        for (int32 i = 0; i < MatNum; ++i)
        {
            UMaterialInterface* BaseMat = SourceStaticMesh->GetMaterial(i);
            if (BaseMat)
            {
                UMaterialInstanceDynamic* DynMat = UMaterialInstanceDynamic::Create(BaseMat, this);
            
                // 此时 PositionRT 和 NormalRT 已经存在，绝对可以传进去了！
                if (bUpdateViaGPU && PositionRT && NormalRT)
                {
                    DynMat->SetTextureParameterValue(FName("PositionVAT"), PositionRT);
                    DynMat->SetTextureParameterValue(FName("NormalVAT"), NormalRT);
                }
            
                MatSet.Add(DynMat);
            }
        }
        // 强制覆盖模型的材质列表，绝杀冲突！
        this->ConfigureMaterialSet(MatSet);
    }
    
    
    
    // 清空并分配内存
    HighResParticles.Empty(HighResParticleCount);
    HighResRestPositions.Empty(HighResParticleCount);
    HighResParticles.AddDefaulted(HighResParticleCount);
    HighResRestPositions.AddDefaulted(HighResParticleCount);

    float DefaultInvMass = (ParticleMass > 0.0f) ? (1.0f / ParticleMass) : 0.0f;

    // 遍历每一个高模顶点，初始化它的物理状态
    for (int32 i = 0; i < HighResParticleCount; ++i)
    {
        FVector WorldPos = GetComponentTransform().TransformPosition((FVector)RenderMesh.GetVertex(i));
        HighResParticles[i].ID = i;
        HighResParticles[i].Position = WorldPos;
        HighResParticles[i].PrevPosition = WorldPos;
        HighResParticles[i].Force = FVector::ZeroVector;
        HighResParticles[i].Velocity = FVector::ZeroVector;
        HighResParticles[i].Col = FColor::White; 
        
        HighResParticles[i].state = 1; 
        HighResParticles[i].InvMass = DefaultInvMass; // 默认全是普通质量
        HighResRestPositions[i] = WorldPos;
    }
    BuildHighResTopologyCache(RenderMesh);
    HighResCurrentNormals.SetNumZeroed(HighResParticleCount);
    
    // =========================================================
    // [性能优化] 预缓存高模的法线 Element ID，防止每帧重复查询
    // =========================================================
    CachedHighResNormalElements.Empty(HighResParticleCount);
    CachedHighResNormalElements.AddDefaulted(HighResParticleCount);
    
    if (RenderMesh.HasAttributes() && RenderMesh.Attributes()->PrimaryNormals())
    {
        UE::Geometry::FDynamicMeshNormalOverlay* NormalOverlay = RenderMesh.Attributes()->PrimaryNormals();
        for (int32 Vid = 0; Vid < HighResParticleCount; ++Vid)
        {
            if (RenderMesh.IsVertex(Vid))
            {
                // 初始化时查一次，永久受用
                NormalOverlay->GetVertexElements(Vid, CachedHighResNormalElements[Vid]);
            }
        }
    }
    if (bUseProxy)
    {
        // === 模式 A: 使用代理网格 ===
        GenerateProxyAndMapping(); // 生成 ProxyMesh 和 映射
        PhysicsMesh = &ProxyMesh;  // 指向低模
        // 缓存 ProxyMesh 的三角形索引，供后续 DrawDebugMesh 使用
        ProxyDebugIndices.Empty();
        ProxyDebugIndices.Reserve(ProxyMesh.TriangleCount() * 3);
        for (int32 tid : ProxyMesh.TriangleIndicesItr())
        {
            UE::Geometry::FIndex3i Tri = ProxyMesh.GetTriangle(tid);
            ProxyDebugIndices.Add(Tri.A);
            ProxyDebugIndices.Add(Tri.B);
            ProxyDebugIndices.Add(Tri.C);
        }
    }
    else
    {
        // === 模式 B: 使用原始高模  ===
        PhysicsMesh = &RenderMesh; // 指向高模
    }
    // 3. 构建动态材质实例 (MID) 并绑定 VAT 纹理
    int32 MatNum = SourceStaticMesh->GetNumMaterials();
    for (int32 i = 0; i < MatNum; ++i)
    {
        UMaterialInterface* BaseMat = SourceStaticMesh->GetMaterial(i);
        if (BaseMat)
        {
            // 创建动态材质实例
            UMaterialInstanceDynamic* DynMat = UMaterialInstanceDynamic::Create(BaseMat, this);
            
            // 如果开启了 GPU 直通，并且 RT 已经成功创建，就把它们塞进材质里
            if (bUpdateViaGPU && PositionRT && NormalRT)
            {
                DynMat->SetTextureParameterValue(FName("PositionVAT"), PositionRT);
                DynMat->SetTextureParameterValue(FName("NormalVAT"), NormalRT);
                
                // 把纹理的宽度传进去，材质里算 UV 坐标时会用到
                DynMat->SetScalarParameterValue(FName("VAT_Dimension"), (float)PositionRT->SizeX);
            }
            
            // 将这个动态材质赋给我们的软体组件
            this->SetMaterial(i, DynMat);
        }
    }
    // =========================================================
    // 4. 初始化低模粒子
    // =========================================================
    
    // 粒子数量 = 物理网格的顶点数
    ProxyParticleCount = PhysicsMesh->VertexCount(); 
    ProxyTriCount = PhysicsMesh->TriangleCount();

    // 重置数组
    ProxyParticles.Empty(ProxyParticleCount);
    ProxyRestPositions.Empty(ProxyParticleCount);
    ProxyParticles.AddDefaulted(ProxyParticleCount);
    ProxyRestPositions.AddDefaulted(ProxyParticleCount);

    DefaultInvMass = (ParticleMass > 0.0f) ? (1.0f / ParticleMass) : 0.0f;

    for (int32 i = 0; i < ProxyParticleCount; ++i)
    {
        // 获取物理网格的顶点位置
        FVector3d LocalPos = PhysicsMesh->GetVertex(i); 
        FVector WorldPos = GetComponentTransform().TransformPosition((FVector)LocalPos);

        ProxyParticles[i].ID = i;
        ProxyParticles[i].Position = WorldPos;
        ProxyParticles[i].PrevPosition = WorldPos;
        ProxyParticles[i].Force = FVector::ZeroVector;
        ProxyParticles[i].Velocity = FVector::ZeroVector;
        
        // [修改] 颜色属性设为默认白，因为反正不参与渲染
        ProxyParticles[i].Col = FColor::White; 

        ProxyParticles[i].state = 1; 
        ProxyParticles[i].InvMass = DefaultInvMass;
        ProxyRestPositions[i] = WorldPos;
    }
    // 5. 构建拓扑缓存 (邻接表)
    BuildTopologyCache(*PhysicsMesh);

    // 6. 构建三角形缓存 (SimulationTriangles)
    SimulationTriangles.Reset();
    SimulationTriangles.Reserve(ProxyTriCount);
    for (int32 tid : PhysicsMesh->TriangleIndicesItr())
    {
        UE::Geometry::FIndex3i Tri = PhysicsMesh->GetTriangle(tid);
        // 存入结构体
        SimulationTriangles.Emplace(Tri.A, Tri.B, Tri.C);
    }
    
    // 7. 初始化法线
    CurrentNormals.SetNumZeroed(ProxyParticleCount);
    UpdateWeightedNormals();
   
    // 8. 构建各种约束
    BuildConstraints();                   // 距离约束
    BuildBendingConstraints(*PhysicsMesh); // 弯曲约束
    if (bUse_DihedralBending)             
    {
        BuildDihedralConstraints(*PhysicsMesh);
    }
    BuildInternalConstraints(*PhysicsMesh);// 内部支撑 
    
    //FixCantileverEdge(250.0f, 1);
    
    // 10. 计算体积
    if (bUse_VolumePressureForce)
    {
        restVolume = CalcClothVolume();
    }

    // 隐藏源网格，只显示软体
    SourceStaticMesh->SetVisibility(bShowStaticMesh);
    if (bUseGPU)
    {
        InitGPUResources();
    }
    // 根据是否调试代理节点，来决定是否隐藏自身(高模)
    //this->SetVisibility(!bShowProxyNodes);
    
    clothStateExists = true;
    // 强制开启复杂碰撞并将其作为简单碰撞使用
    bEnableComplexCollision = true;
    CollisionType = ECollisionTraceFlag::CTF_UseComplexAsSimple;
    
    // 强制组件立刻重建物理碰撞数据
    UpdateCollision(true); 
    
    clothStateExists = true;
    if (bUseGPU) InitGPUResources();
    NotifyMeshUpdated(); // 刷新一次显示
}


void UMySoftBodyMeshComponent::BuildTopologyCache(const UE::Geometry::FDynamicMesh3& Mesh)
{
    // 初始化邻接表数组
    VertexToTriangleMap.Empty();
    VertexToTriangleMap.AddDefaulted(ProxyParticleCount);

    // 遍历所有三角形，填充“顶点->三角形”映射
    for (int32 tid : Mesh.TriangleIndicesItr())
    {
        UE::Geometry::FIndex3i Tri = Mesh.GetTriangle(tid);
        
        // 安全检查索引范围
        if (Mesh.IsVertex(Tri.A)) VertexToTriangleMap[Tri.A].Add(tid);
        if (Mesh.IsVertex(Tri.B)) VertexToTriangleMap[Tri.B].Add(tid);
        if (Mesh.IsVertex(Tri.C)) VertexToTriangleMap[Tri.C].Add(tid);
    }
    // 构建扁平化数组（GPU 友好格式）
    VertexTriangleOffsets.SetNum(ProxyParticleCount);
    VertexTriangleCounts.SetNum(ProxyParticleCount);
    VertexTriangleIndices.Empty();

    uint32 Offset = 0;
    for (int32 i = 0; i < ProxyParticleCount; ++i)
    {
        VertexTriangleOffsets[i] = Offset;
        VertexTriangleCounts[i] = VertexToTriangleMap[i].Num();
        
        for (int32 TriID : VertexToTriangleMap[i])
        {
            VertexTriangleIndices.Add(TriID);
        }
        Offset += VertexTriangleCounts[i];
    }
}

void UMySoftBodyMeshComponent::BuildConstraints()
{
    Constraints.Empty();

    // [优化] 使用 TSet 快速去重
    TSet<TPair<int32, int32>> AddedEdges;
    AddedEdges.Reserve(ProxyParticleCount * 6); // 预分配（网格平均每顶点6条边）

    for (int32 p = 0; p < ProxyParticleCount; ++p)
    {
        FMySoftBodyParticle& curPt = ProxyParticles[p];
        curPt.conCount = 0;

        for (int32 tid : VertexToTriangleMap[p])
        {
            if (!SimulationTriangles.IsValidIndex(tid)) continue;
            const FSoftBodyTriangle& Tri = SimulationTriangles[tid];
            int32 Indices[3] = { Tri.A, Tri.B, Tri.C };

            for (int32 k = 0; k < 3; ++k)
            {
                int32 neighborID = Indices[k];
                if (neighborID == p) continue;

                // [关键优化] 标准化边的表示（小ID在前）
                int32 MinID = FMath::Min(p, neighborID);
                int32 MaxID = FMath::Max(p, neighborID);
                TPair<int32, int32> Edge(MinID, MaxID);

                // [O(1) 查找] 检查是否已添加
                if (!AddedEdges.Contains(Edge))
                {
                    AddedEdges.Add(Edge);
                    float Dist = FVector::Dist(ProxyRestPositions[MinID], ProxyRestPositions[MaxID]);
                    Constraints.Emplace(MinID, MaxID, Dist);
                    curPt.conCount++;
                }
            }
        }
    }
}

void UMySoftBodyMeshComponent::ResetToInitalState()
{
    if (!clothStateExists) return;

    for (int32 i = 0; i < ProxyParticleCount; ++i)
    {
        // 强制归位到原始位置
        ProxyParticles[i].Position = ProxyRestPositions[i];
        ProxyParticles[i].PrevPosition = ProxyRestPositions[i];
        ProxyParticles[i].Force = FVector::ZeroVector;
    }
    
    // 立即刷新网格
    TickUpdateCloth();
}

// =========================================================
// 物理循环 (Solver)
// =========================================================

void UMySoftBodyMeshComponent::SubstepSolve()
{
    // 1. 更新棍子数据缓存 (只做一次，供后续多次迭代使用)
    bHasStickInput = false;
    bHapticCollisionThisFrame = false;
    if (bEnableHapticInteraction && HapticsComponent && bSimulate)
    {
        FHapticToolState ToolState = HapticsComponent->GetHapticToolState();
        FTransform ToolTransform = HapticsComponent->GetComponentTransform();
        
        CachedStickStart = ToolTransform.TransformPosition(ToolState.StartPos);
        CachedStickEnd = ToolTransform.TransformPosition(ToolState.EndPos);
        CachedStickVelStart = ToolTransform.TransformVector(ToolState.StartVel);
        CachedStickVelEnd = ToolTransform.TransformVector(ToolState.EndVel);
        CachedStickRadius = ToolState.Radius;
        bHasStickInput = true;
    }
    
    // 2. 积分 (应用重力、外力)
    Integrate(St);

    // 3. 约束求解 (距离、弯曲、交互碰撞）
    EvalConstraints();

    // 4. 碰撞处理
    if (bWorldCollision)
        ClothCollisionWorld();
    // 5. 体积保持
    if (bUse_VolumePressureForce)
    {
        UpdateWeightedNormals(); 
        VolumePreservation();
    }
    
    // 6. [双向耦合] 将被物理引擎推挤后的棍子位置写回
    // =========================================================
    if (bHasStickInput && HapticsComponent)
    {
        HapticsComponent->SetCollisionState(bHapticCollisionThisFrame);

        FTransform ToolTransform = HapticsComponent->GetComponentTransform();
        
        // World -> Local
        FVector LocalNewStart = ToolTransform.InverseTransformPosition(CachedStickStart);
        FVector LocalNewEnd   = ToolTransform.InverseTransformPosition(CachedStickEnd);

        // 调用 OpenHaptics 的后门，强制更新 Visual Tool 的位置
        // 这样下一帧计算力反馈时，Visual Tool 就停在了软体表面，而 Force Tool 在软体内部
        HapticsComponent->UpdateVisualStateFromPhysics(LocalNewStart, LocalNewEnd);
    }
    // 7. 更新速度 (根据修正后的位置，算出真实速度，供下一帧使用)
    UpdateVelocities(St);
}
// 处理与触觉棍子的碰撞
void UMySoftBodyMeshComponent::CollideWithHapticStylus(const FVector& V_Start, const FVector& V_End, float Radius)
{
    // 1. 准备碰撞参数
    // 我们稍微加一点点容差(Bias)，确保在视觉接触前一点点就开始计算，防止浮点数抖动
    const float SafetyBias = 0.1f; 
    float TotalRadius = Radius + ParticleRadius + SafetyBias;
    float CollisionRadiusSq = TotalRadius * TotalRadius;

    FVector AccumStickOffset = FVector::ZeroVector;
    int32 ContactCount = 0;

    for (int32 i = 0; i < ProxyParticleCount; ++i)
    {
        FMySoftBodyParticle& Pt = ProxyParticles[i];
        
        // -----------------------------------------------------
        // 1. CCD: 线段对线段检测
        // -----------------------------------------------------
        // 线段 A: 粒子的运动轨迹
        FVector ParticleSegStart = Pt.PrevPosition;
        FVector ParticleSegEnd   = Pt.Position;

        FVector StickSegStart = CachedStickEnd;
        FVector StickSegEnd   = CachedStickStart;
        FVector ClosestPtOnPath; // 粒子轨迹上离棍子最近的点
        FVector ClosestPtOnStick; // 棍子上离粒子轨迹最近的点
        // UE 数学库：计算两条线段之间的最近距离
        FMath::SegmentDistToSegment(
            ParticleSegStart, ParticleSegEnd, // 线段 A
            StickSegStart,    StickSegEnd,    // 线段 B
            ClosestPtOnPath,
            ClosestPtOnStick
        );
        // 计算最近点之间的距离平方
        float DistSq = FVector::DistSquared(ClosestPtOnPath, ClosestPtOnStick);

        // -----------------------------------------------------
        // 2. 碰撞检测与响应
        // -----------------------------------------------------
        if (DistSq < CollisionRadiusSq)
        {
            bHapticCollisionThisFrame = true;

            float Dist = FMath::Sqrt(DistSq);

            // 1. 计算法线：从 棍子 指向 粒子
            FVector Normal;
            if (Dist < 1e-6f)
            {
                // 极端情况：完全重合 (Dist=0)，说明粒子轨迹恰好穿过棍子中轴线
    
                // 1. 获取粒子这一帧的运动向量 (Prev -> Cur)
                FVector ParticleMotion = Pt.Position - Pt.PrevPosition;

                // 2. 移除在这个向量中“平行于棍子”的分量，只保留“垂直于棍子”的分量
                // 这一步能保证推出的方向是垂直于棍子表面的（最短脱离路径）
                FVector StickDir = (StickSegEnd - StickSegStart).GetSafeNormal(); // 棍子轴向
                if (StickDir.IsZero()) StickDir = FVector::UpVector;

                // 分离出垂直分量： V_perp = V - (V . StickDir) * StickDir
                FVector PerpendicularMotion = ParticleMotion - (StickDir * (ParticleMotion | StickDir));

                // 3. 计算推出法线
                if (PerpendicularMotion.SizeSquared() > 1e-6f)
                {
                    // 既然粒子是“撞进去”的，那我们要把它“推回来”
                    // 所以取垂直分量的反方向
                    Normal = -PerpendicularMotion.GetSafeNormal();
                }
                else
                {
                    // 备用方案：如果粒子几乎没动（静止穿透），或者粒子运动完全平行于棍子
                    // 这时候“历史路径”失效了，只能找一个任意垂线推出去
                    Normal = FVector::CrossProduct(StickDir, FVector::RightVector).GetSafeNormal();
                    if (Normal.IsZero()) Normal = FVector::CrossProduct(StickDir, FVector::UpVector).GetSafeNormal();
                }
            }
            else
            {
                Normal = (ClosestPtOnPath - ClosestPtOnStick) / Dist;
            }

            // 2. 计算穿透深度
            float PenetrationDepth = TotalRadius - Dist;

            // 如果粒子是被钉住的(state=0)，棍子全责
            if (Pt.state == 0) 
            {
                FVector TargetPos = ClosestPtOnStick + (Normal * TotalRadius);
                // 直接覆盖位置，保证视觉不穿模
                Pt.Position = TargetPos;
            }

            //记录对棍子的反作用力
            if (HapticFeedbackStiffness > 0.0f)
            {
                // 棍子应该退让的完整向量
                FVector DesiredStickPush = -Normal * PenetrationDepth;
                
                // 累积起来，稍后取平均
                AccumStickOffset += DesiredStickPush;
                ContactCount++;
            }
            
            // -----------------------------------------------------
            // 4. 速度计算 
            // -----------------------------------------------------
            FVector StickVec = StickSegEnd - StickSegStart; 
            float StickLenSq = StickVec.SizeSquared();
    
            float t = 0.0f;
            if (StickLenSq > 1e-6f)
            {
                // 计算投影：(碰撞点 - 起点) 点乘 (棍子向量) / 长度平方
                FVector PointToStart = ClosestPtOnStick - StickSegStart;
                t = (PointToStart | StickVec) / StickLenSq;
        
                // 虽然理论上点在线上，但为了安全一定要 Clamp
                t = FMath::Clamp(t, 0.0f, 1.0f);
            }
            // A. 计算棍子在碰撞点处的速度 (线性插值)
            // 因为刚体上任意一点的速度也是线性的，所以直接 Lerp 是物理正确的
            FVector ColliderVel = FMath::Lerp(V_End,V_Start, t);

            // B. 计算粒子当前速度 (显式或隐式)
            // 显式速度 UpdateVelocities，这里用 Pt.Velocity
            FVector ParticleVel = Pt.Velocity; 

            // C. 相对速度处理 (反弹 + 摩擦)
            FVector RelVel = ParticleVel - ColliderVel;
            
            float NormalSpeed = RelVel | Normal;

            // 只有当粒子“冲向”棍子时才反弹
            if (NormalSpeed < 0.0f)
            {
                FVector NormalVel = Normal * NormalSpeed;
                FVector TangentVel = RelVel - NormalVel;

                // 摩擦力
                FVector NewTangentRel = TangentVel * (1.0f - CollisionFriction);
                
                // 反弹力
                FVector NewNormalRel = -NormalVel * CollisionRestitution;

                // 合成新相对速度
                FVector NewRelVel = NewTangentRel + NewNormalRel;

                // 转回绝对速度
                FVector TargetVel = ColliderVel + NewRelVel;

                // 逆向修改 PrevPosition 以产生正确速度
                Pt.PrevPosition = Pt.Position - (TargetVel * St);
                Pt.Velocity = TargetVel; // 同步显式速度
            }
        }
    }
    // =========================================================
    // 循环结束后，统一应用棍子修正
    // =========================================================
    if (ContactCount > 0 && HapticFeedbackStiffness > 0.0f)
    {
        // 策略：取平均值。这样无论接触 1 个点还是 100 个点，棍子受到的修正都是平滑的。
        FVector AveragePush = AccumStickOffset / (float)ContactCount;

        // 应用 Stiffness。
        // 这里不需要再乘 (1-ParticleRatio) 之类的，因为这是一个独立的“推回”系数。
        // 值越小，棍子越像“无限质量”的墙；值越大，棍子越容易被推走（软）。
        FVector FinalStickMove = AveragePush * HapticFeedbackStiffness;

        // 在这里一次性修改棍子位置
        CachedStickStart += FinalStickMove;
        CachedStickEnd   += FinalStickMove;
    }
}
void UMySoftBodyMeshComponent::Integrate(float InSubstepTime)
{
    const FVector Gravity = FVector(0, 0, GetWorld()->GetGravityZ()) * ClothGravityScale;
    
    // 就像空气阻力，让震荡能停下来。没有这个，软体永远不会稳定。
    float Damping = 0.998f;
    for (int32 i = 0; i < ProxyParticleCount; ++i)
    {
        FMySoftBodyParticle& Pt = ProxyParticles[i];
        if (Pt.state == 0)
        {
            // 如果是被钉住的点，速度强制归零
            Pt.Velocity = FVector::ZeroVector;
            continue;
        }

        // 1. 应用外力更新速度 (V = V + a * dt)
        FVector Accel = Gravity + ((Pt.Force + ClothForce) * Pt.InvMass);
        Pt.Velocity += Accel * InSubstepTime;
        // 2. 应用阻尼
        Pt.Velocity *= Damping;
        
        // 3. 预测下一帧位置 (P = P + V * dt)
        // 我们把当前位置存入 PrevPosition，用于后续碰撞后的速度反推
        Pt.PrevPosition = Pt.Position;
        Pt.Position += Pt.Velocity * InSubstepTime;
        
        // 4. 清零瞬时力
        Pt.Force = FVector::ZeroVector;
    }
}

void UMySoftBodyMeshComponent::BuildBendingConstraints(const UE::Geometry::FDynamicMesh3& Mesh)
{
    BendingConstraints.Empty();

    // 遍历网格的所有边 (Edge)
    for (int32 eid : Mesh.EdgeIndicesItr())
    {
        // 1. 获取这条边连接的两个三角形 (Tri A, Tri B)
        UE::Geometry::FIndex2i TriPair = Mesh.GetEdgeT(eid);

        // 如果是边界边（只有一个三角形），没法弯曲，跳过
        if (TriPair.B == UE::Geometry::FDynamicMesh3::InvalidID) continue;

        // 2. 获取这条边的两个顶点
        UE::Geometry::FIndex2i EdgeV = Mesh.GetEdgeV(eid);

        // 3. [核心] 找出两个三角形中“不在这条边上”的那两个对角点
        // 我们需要找到 P_Opposite_A 和 P_Opposite_B
        
        auto GetOppositeVertex = [&](int32 TriID, int32 E0, int32 E1) -> int32
        {
            UE::Geometry::FIndex3i Tri = Mesh.GetTriangle(TriID);
            if (Tri.A != E0 && Tri.A != E1) return Tri.A;
            if (Tri.B != E0 && Tri.B != E1) return Tri.B;
            return Tri.C; // Tri.C
        };

        int32 P_A = GetOppositeVertex(TriPair.A, EdgeV.A, EdgeV.B);
        int32 P_B = GetOppositeVertex(TriPair.B, EdgeV.A, EdgeV.B);

        // 4. 添加约束
        // 连接这两个对角点，限制它们的距离
        if (P_A != -1 && P_B != -1)
        {
            float Dist = FVector::Dist(ProxyRestPositions[P_A], ProxyRestPositions[P_B]);
            BendingConstraints.Emplace(P_A, P_B, Dist);
        }
    }
    
    UE_LOG(LogTemp, Log, TEXT("Bending Constraints Built: %d"), BendingConstraints.Num());
}

void UMySoftBodyMeshComponent::BuildDihedralConstraints(const UE::Geometry::FDynamicMesh3& Mesh)
{
    DihedralConstraints.Empty();

    for (int32 eid : Mesh.EdgeIndicesItr())
    {
        UE::Geometry::FIndex2i TriPair = Mesh.GetEdgeT(eid);
        if (TriPair.B == UE::Geometry::FDynamicMesh3::InvalidID) continue; // 边界边跳过

        UE::Geometry::FIndex2i EdgeV = Mesh.GetEdgeV(eid);
        UE::Geometry::FIndex3i TriA = Mesh.GetTriangle(TriPair.A);
        UE::Geometry::FIndex3i TriB = Mesh.GetTriangle(TriPair.B);

        // 1. 严格按照 TriA 的逆时针绕序 (CCW) 来分配 P1, P2, P3
        int32 P1 = -1, P2 = -1, P3 = -1;
        if ((TriA.A == EdgeV.A && TriA.B == EdgeV.B) || (TriA.A == EdgeV.B && TriA.B == EdgeV.A)) {
            P1 = TriA.A; P2 = TriA.B; P3 = TriA.C;
        } else if ((TriA.B == EdgeV.A && TriA.C == EdgeV.B) || (TriA.B == EdgeV.B && TriA.C == EdgeV.A)) {
            P1 = TriA.B; P2 = TriA.C; P3 = TriA.A;
        } else {
            P1 = TriA.C; P2 = TriA.A; P3 = TriA.B;
        }

        // 2. 找出 TriB 中的对角点 P4
        int32 P4 = (TriB.A != P1 && TriB.A != P2) ? TriB.A : ((TriB.B != P1 && TriB.B != P2) ? TriB.B : TriB.C);

        if (P3 != -1 && P4 != -1)
        {
            FVector Pos1 = ProxyRestPositions[P1];
            FVector Pos2 = ProxyRestPositions[P2];
            FVector Pos3 = ProxyRestPositions[P3];
            FVector Pos4 = ProxyRestPositions[P4];

            // 3. 计算同侧法线
            // TriA 法线: (P2-P1) x (P3-P1)
            FVector N1 = FVector::CrossProduct(Pos2 - Pos1, Pos3 - Pos1).GetSafeNormal();
            // TriB 法线: 注意方向是 (P1-P2) x (P4-P2) 才能保证和 N1 同侧！
            FVector N2 = FVector::CrossProduct(Pos1 - Pos2, Pos4 - Pos2).GetSafeNormal();

            // 4. 计算带符号的初始二面角 (平坦时为 0 度)
            FVector EdgeDir = (Pos2 - Pos1).GetSafeNormal();
            float Dot = FVector::DotProduct(N1, N2);
            float Det = FVector::DotProduct(EdgeDir, FVector::CrossProduct(N1, N2));
            float RestAngle = FMath::Atan2(Det, Dot); // 结果范围在 [-pi, pi] 之间

            DihedralConstraints.Emplace(P1, P2, P3, P4, RestAngle);
        }
    }

    UE_LOG(LogTemp, Log, TEXT("Dihedral Constraints Built: %d"), DihedralConstraints.Num());
}

void UMySoftBodyMeshComponent::UpdateVelocities(float InSubstepTime)
{
    float InvDt = 1.0f / InSubstepTime;

    for (int32 i = 0; i < ProxyParticleCount; ++i)
    {
        FMySoftBodyParticle& Pt = ProxyParticles[i];
        if (Pt.state == 0) continue;

        // PBD 核心公式：V_new = (Pos_new - Pos_old) / dt
        // 这捕捉了约束求解和碰撞产生的所有速度变化
        Pt.Velocity = (Pt.Position - Pt.PrevPosition) * InvDt;
        
    }
}

void UMySoftBodyMeshComponent::UpdateTangents()
{
    FDynamicMesh3* Mesh = &GetDynamicMesh()->GetMeshRef();
    if (!Mesh->HasAttributes()) return;

    // 获取法线和切线 Overlay
    UE::Geometry::FDynamicMeshNormalOverlay* NormalOverlay = Mesh->Attributes()->PrimaryNormals();
    UE::Geometry::FDynamicMeshNormalOverlay* TangentOverlay = Mesh->Attributes()->PrimaryTangents();
    auto* UVOverlay = Mesh->Attributes()->PrimaryUV();
    if (!NormalOverlay || !TangentOverlay) return;

    // 先把 CurrentNormals 更新到 NormalOverlay（确保渲染法线一致）
    for (int32 VertexID : Mesh->VertexIndicesItr())
    {
        FVector Normal = CurrentNormals[VertexID].GetSafeNormal();
        // 更新 NormalOverlay 中所有关联元素
        TArray<int32> NormalElements;
        NormalOverlay->GetVertexElements(VertexID, NormalElements);
        for (int32 ElemID : NormalElements)
        {
            NormalOverlay->SetElement(ElemID, (FVector3f)Normal);
        }
    }
    // 创建切线计算对象
    UE::Geometry::FMeshTangentsf Tangents;
    Tangents.SetMesh(Mesh);
 
    // 计算切线，传入UV Overlay和是否正交化
    Tangents.ComputeTriangleTangents(UVOverlay, true);
 
    // 将切线写入Overlay
    Tangents.CopyToOverlays(*Mesh);
    
}

void UMySoftBodyMeshComponent::UpdateNormals()
{
    UE::Geometry::FDynamicMesh3* Mesh = &GetDynamicMesh()->GetMeshRef();
    if (!Mesh->HasAttributes()) return;
 
    UE::Geometry::FDynamicMeshNormalOverlay* NormalOverlay = Mesh->Attributes()->PrimaryNormals();
    if (!NormalOverlay) return;
 
    if (CurrentNormals.Num() != Mesh->VertexCount())
    {
        UE_LOG(LogTemp, Warning, TEXT("CurrentNormals size does not match vertex count!"));
        return;
    }
 
    for (int32 VertexID : Mesh->VertexIndicesItr())
    {
        FVector Normal = CurrentNormals[VertexID].GetSafeNormal();
 
        TArray<int32> ElementIDs;
        NormalOverlay->GetVertexElements(VertexID, ElementIDs);
 
        for (int32 ElemID : ElementIDs)
        {
            NormalOverlay->SetElement(ElemID, (FVector3f)Normal);
        }
    }
}

void UMySoftBodyMeshComponent::EvalConstraints()
{
    // 1. 初始化，重置所有约束的 Lambda 为 0
    for (FMySoftBodyConstraint& Con : Constraints) Con.Lambda = 0.0f;
    if (bUse_BendingForce)
        for (FMySoftBodyConstraint& Con : BendingConstraints) Con.Lambda = 0.0f;
    if (bUse_InternalConstraints)
        for (auto& Con : InternalConstraints) Con.Lambda = 0.0f;
    for (int32 k = 0; k < ConstraintIterations; ++k)
    {
        // 1. 距离约束
        for (FMySoftBodyConstraint& Con : Constraints)
        {
            // [GPU 准备] 传入数组和索引
            SolveDistanceConstraintXPBD(ProxyParticles, Con.P0_Index, Con.P1_Index, Con.RestLength, XPBD_StretchStiffness, St, Con.Lambda);
            // Debug: 只在最后一次迭代画线
            if (bShow_Constraints && k == ConstraintIterations - 1)
            {
                DrawDebugLine(GetWorld(), 
                    ProxyParticles[Con.P0_Index].Position, 
                    ProxyParticles[Con.P1_Index].Position, 
                    FColor::Red, false, St);
            }
        }
        // 2. 弯曲约束 (保持平滑)
        // 使用 BendingStiffness 参数
        if (bUse_BendingForce)
        {
            for (FMySoftBodyConstraint& Con : BendingConstraints)
            {
                SolveDistanceConstraintXPBD(ProxyParticles, Con.P0_Index, Con.P1_Index, Con.RestLength, XPBD_BendingStiffness, St, Con.Lambda);
            }
        }
        // 在每次迭代中，粒子推棍子，棍子推粒子
        if (bHasStickInput)
        {
            CollideWithHapticStylus(CachedStickVelStart, CachedStickVelEnd, CachedStickRadius);
        }
        if (bUse_InternalConstraints)
        {
            for (FMySoftBodyConstraint& Con : InternalConstraints)
            {
                // 使用 XPBD_InternalStiffness
                SolveDistanceConstraintXPBD(ProxyParticles, Con.P0_Index, Con.P1_Index, Con.RestLength, XPBD_InternalStiffness, St, Con.Lambda);
            }
        }
    }
    
}
void UMySoftBodyMeshComponent::BuildInternalConstraints(const UE::Geometry::FDynamicMesh3& Mesh)
{
    // 清空专用数组
    InternalConstraints.Empty(); 

    UE::Geometry::FDynamicMeshAABBTree3 Spatial(&Mesh);
    int32 InternalCount = 0;

    // [核心修复] 引入 Set 进行全局去重，彻底废除 BestVert > i 的错误逻辑
    TSet<TPair<int32, int32>> AddedInternalEdges;

    for (int32 i = 0; i < ProxyParticleCount; ++i)
    {
        FVector3d StartPos = Mesh.GetVertex(i);
        FVector Normal = CurrentNormals[i].GetSafeNormal();
        
        // 射线方向：沿着法线反方向（向内部）
        FVector3d RayDir = FVector3d(-Normal.X, -Normal.Y, -Normal.Z);

        UE::Geometry::IMeshSpatial::FQueryOptions Options;
        Options.MaxDistance = 99999.0; 
        
        // 起点沿着射线方向推进一点，防止打到自己
        FRay3d Ray(StartPos + RayDir * 0.1, RayDir);

        int32 HitTID = -1;
        double HitDist = 0.0;

        if (Spatial.FindNearestHitTriangle(Ray, HitDist, HitTID, Options))
        {
            // [新增保护] 忽略距离太近的碰撞 (比如模型凹陷处的误触)
            // 这里的 2.0f 代表 2 厘米，你可以根据你的模型大小微调
            if (HitDist < 2.0) continue; 

            UE::Geometry::FIndex3i Tri = Mesh.GetTriangle(HitTID);

            // [扩大范围] 三脚架连接法：把射线击中的那个三角形的 3 个顶点全都连上！
            // 这会让软体内部形成极度稳固的四面体支撑结构
            int32 TargetVerts[3] = { Tri.A, Tri.B, Tri.C };

            for (int32 k = 0; k < 3; ++k)
            {
                int32 TargetID = TargetVerts[k];

                if (TargetID != i)
                {
                    // 构造边 (小ID在前，大ID在后，保证唯一性)
                    int32 MinID = FMath::Min(i, TargetID);
                    int32 MaxID = FMath::Max(i, TargetID);
                    TPair<int32, int32> Edge(MinID, MaxID);

                    // 检查是否已经添加过这条支撑线
                    if (!AddedInternalEdges.Contains(Edge))
                    {
                        AddedInternalEdges.Add(Edge);
                        
                        float Dist = FVector::Dist(ProxyRestPositions[MinID], ProxyRestPositions[MaxID]);
                        InternalConstraints.Emplace(MinID, MaxID, Dist);
                        InternalCount++;
                    }
                }
            }
        }
    }
    
    UE_LOG(LogTemp, Log, TEXT("SoftBody: Built %d Internal Constraints (Expanded Scope)."), InternalCount);
}
// A. 位置求解器：只管推出去
void UMySoftBodyMeshComponent::SolveStickPosition(
    FMySoftBodyParticle& Pt, 
    const FVector& Start, const FVector& End, float Radius)
{
    FVector StickVec = End - Start;
    float StickLenSq = StickVec.SizeSquared();
    
    // 计算 SDF
    FVector PtToStart = Pt.Position - Start;
    float t = 0.0f;
    if (StickLenSq > 1e-6f)
    {
        t = (PtToStart | StickVec) / StickLenSq;
        t = FMath::Clamp(t, 0.0f, 1.0f);
    }
    FVector ClosestPoint = Start + (StickVec * t);

    // 碰撞检测与投影
    float DistSq = FVector::DistSquared(Pt.Position, ClosestPoint);
    float MinDist = Radius + 5.0f; // 棍子半径 + 粒子半径
    
    if (DistSq < MinDist * MinDist)
    {
        float Dist = FMath::Sqrt(DistSq);
        FVector Normal = (Dist < 1e-4f) ? FVector::UpVector : (Pt.Position - ClosestPoint) / Dist;
        
        // [只改位置] 
        float Penetration = MinDist - Dist;
        Pt.Position += Normal * Penetration;
    }
}
// B. 速度求解器：精细、处理反弹、修改 PrevPosition
void UMySoftBodyMeshComponent::SolveStickVelocity(
    FMySoftBodyParticle& Pt, 
    const FVector& Start, const FVector& End, 
    const FVector& V_Start, const FVector& V_End, 
    float Radius, float Friction, float Restitution, float dt)
{
    // 这里需要再做一次 SDF 检测，因为经过多次迭代后，粒子的位置可能变了
    // 但这次我们只关心“是否还在接触”，如果是，就处理摩擦反弹
    
    FVector StickVec = End - Start;
    float StickLenSq = StickVec.SizeSquared();
    
    FVector PtToStart = Pt.Position - Start;
    float t = 0.0f;
    if (StickLenSq > 1e-6f)
    {
        t = (PtToStart | StickVec) / StickLenSq;
        t = FMath::Clamp(t, 0.0f, 1.0f);
    }
    FVector ClosestPoint = Start + (StickVec * t);
    
    float DistSq = FVector::DistSquared(Pt.Position, ClosestPoint);
    float MinDist = Radius + 5.0f + 0.5f; // 稍微加一点容差(Bias)，判定为接触
    
    if (DistSq < MinDist * MinDist)
    {
        float Dist = FMath::Sqrt(DistSq);
        FVector Normal = (Dist < 1e-4f) ? FVector::UpVector : (Pt.Position - ClosestPoint) / Dist;

        // [核心] 计算速度修正
        FVector ColliderVel = FMath::Lerp(V_Start, V_End, t);
        
        // 使用显式速度作为入射速度 (Integrate 算出来的预测速度)
        FVector ParticleVel = Pt.Velocity; 
        FVector RelVel = ParticleVel - ColliderVel;

        float NormalSpeed = RelVel | Normal;

        // 只有撞向物体时才处理
        if (NormalSpeed < 0.0f)
        {
            FVector NormalVel = Normal * NormalSpeed;
            FVector TangentVel = RelVel - NormalVel;

            // 摩擦 & 反弹
            FVector NewRelVel = TangentVel * (1.0f - Friction);
            if (FMath::Abs(NormalSpeed) > 10.0f) 
            {
                NewRelVel += Normal * (-NormalSpeed * Restitution);
            }

            FVector TargetVel = ColliderVel + NewRelVel;
            
            // [只改 PrevPosition]
            // 这样 UpdateVelocities 就能算出正确的反弹后速度
            Pt.PrevPosition = Pt.Position - (TargetVel * dt);
            
            // 可选：同步更新显式速度，供后续逻辑使用
            Pt.Velocity = TargetVel;
        }
    }
}
void UMySoftBodyMeshComponent::SolveDistanceConstraintXPBD(TArray<FMySoftBodyParticle>& InParticles, int32 IdxA, int32 IdxB, float RestLen, float Stiffness,float dt,float& AccLambda)
{
    FMySoftBodyParticle& PtA = InParticles[IdxA];
    FMySoftBodyParticle& PtB = InParticles[IdxB];

    // 获取两个点的倒质量
    float w1 = PtA.InvMass;
    float w2 = PtB.InvMass;
    float wSum = w1 + w2;
    // 如果两个点都是固定的(质量无穷大)，无法移动，直接返回
    if (wSum < 1e-6f) return;
    
    // 1. 计算当前距离向量和长度
    FVector Delta = PtA.Position - PtB.Position;
    float CurrentDist = Delta.Size();
    if (CurrentDist < 1e-6f) return; // 避免除零

    // 2. 梯度方向 (n)
    FVector n = Delta / CurrentDist;

    // 3. 约束偏差 C(x) = 当前长度 - 原长
    float C = CurrentDist - RestLen;

    // 4. [XPBD 核心] 计算柔度 alpha
    // alpha = 1 / (k * dt * dt)
    float alpha = 0.0f;
    if (Stiffness > 1e-6f)
    {
        alpha = 1.0f / (Stiffness * dt * dt);
    }
    
    // 5. 计算拉格朗日乘子修正量
    // 也就是我们要移动的总距离比例，考虑了刚度(alpha)和质量(wSum)
    float dLambda = (-C - (alpha * AccLambda)) / (wSum + alpha);
    
    // 4. 更新累积值
    AccLambda += dLambda;
    
    // 6. 应用修正 (按质量反比分配)
    FVector CorrectionA = n * (dLambda * w1);
    FVector CorrectionB = n * (dLambda * -w2); // B的方向是反的

    if (PtA.state != 0) PtA.Position += CorrectionA;
    if (PtB.state != 0) PtB.Position += CorrectionB;
    
}

// =========================================================
// 碰撞处理
// =========================================================

void UMySoftBodyMeshComponent::ClothCollisionWorld()
{
    UWorld* World = GetWorld();
    if (!World) return;

    FCollisionQueryParams Params(SCENE_QUERY_STAT(SoftBodyCollision));
    Params.AddIgnoredActor(GetOwner());
    // 1. 定义我们要检测的对象类型列表
    FCollisionObjectQueryParams ObjectParams;
    ObjectParams.AddObjectTypesToQuery(ECC_WorldStatic);
    ObjectParams.AddObjectTypesToQuery(ECC_WorldDynamic);
    ObjectParams.AddObjectTypesToQuery(ECC_Pawn);
    ObjectParams.AddObjectTypesToQuery(ECC_PhysicsBody);
    for (int32 i = 0; i < ProxyParticleCount; ++i)
    {
        FMySoftBodyParticle& Pt = ProxyParticles[i];
        FHitResult Result;
        
        FVector PredictedPos = Pt.Position;
        
        bool bHit = World->SweepSingleByObjectType(
            Result, 
            Pt.PrevPosition, 
            Pt.Position, 
            FQuat::Identity, 
            ObjectParams, 
            FCollisionShape::MakeSphere(ParticleRadius), 
            Params
        );

        if (bHit)
        {
            // 1. 位置修正 (Projection)
            if (Result.bStartPenetrating)
            {
                Pt.Position = Pt.PrevPosition + (Result.Normal * (Result.PenetrationDepth + 0.5f));
            }
            else
            {
                Pt.Position = Result.Location + (Result.Normal * 0.5f);
            }
            // ============================================================
            // [核心修改] 获取碰撞体速度并计算相对速度
            // ============================================================
            
            FVector ColliderVel = FVector::ZeroVector;
            
            // 尝试获取被撞组件的速度
            if (Result.Component.IsValid())
            {
                // 获取组件在世界空间的的线性速度
                // 注意：如果是旋转物体，更精确的做法是 GetPhysicsLinearVelocityAtPoint(Result.ImpactPoint)
                // 但对于一般移动物体，GetComponentVelocity 足够了，且对非物理模拟(Kinematic)物体也有效
                ColliderVel = Result.Component->GetComponentVelocity();
            }
            FVector CurrentEffectiveVel = (Pt.Position - Pt.PrevPosition) / St;
            // [关键] 计算相对速度 (粒子 - 墙)
            // 如果墙撞向粒子，相对速度会很大，产生巨大的反弹
            FVector RelVel = CurrentEffectiveVel - ColliderVel;

            // ============================================================
            // 3. 基于相对速度计算物理反应
            // ============================================================

            float NormalSpeed = RelVel | Result.Normal; // 相对法向速度
            
            // 只有当两者“相互靠近”时才处理反弹
            // NormalSpeed < 0 表示粒子相对于墙壁正在撞进去
            if (NormalSpeed < 0.0f)
            {
                FVector NormalVel = Result.Normal * NormalSpeed;
                FVector TangentVel = RelVel - NormalVel;

                // A. 摩擦力 (作用于相对切向速度)
                FVector NewTangentRel = TangentVel * (1.0f - CollisionFriction);

                // B. 反弹力 (作用于相对法向速度)
                FVector NewNormalRel = FVector::ZeroVector;
                
                // 阈值检查：相对撞击速度够大才反弹
                if (FMath::Abs(NormalSpeed) > 10.0f) 
                {
                    NewNormalRel = -NormalVel * CollisionRestitution;
                }

                // C. 合成新的相对速度
                FVector NewRelVel = NewTangentRel + NewNormalRel;

                // D. [关键] 转回绝对速度
                // V_particle = V_collider + V_rel_new
                // 这意味着粒子会继承墙壁的速度，再加上反弹的速度
                FVector TargetVelocity = ColliderVel + NewRelVel;

                // 4. 逆向修改 PrevPosition
                Pt.PrevPosition = Pt.Position - (TargetVelocity * St);
                
                // 同步显式速度 (可选，但推荐)
                Pt.Velocity = TargetVelocity;
            }
            else
            {
                // 粒子正在远离移动的墙壁（或墙壁追不上粒子），不做反弹处理
                // 但为了防止穿透修正导致的速度误差，可以重置一下
                // 这里简单处理：如果没发生撞击（分离状态），就不强制改速度了
            }
        }
    }
}


// =========================================================
// 网格更新与法线重算
// =========================================================

void UMySoftBodyMeshComponent::TickUpdateCloth()
{
    
    // 获取底层 Mesh
    UE::Geometry::FDynamicMesh3* Mesh = &GetDynamicMesh()->GetMeshRef();
    if (Mesh->VertexCount() == 0) return;
    const FTransform InvTransform = GetComponentTransform().Inverse();
    if (!bUpdateViaGPU)
    {
        double StartTime = FPlatformTime::Seconds();
        if (!bUseProxy)
        {
            ParallelFor(ProxyParticleCount, [&](int32 i)
            {
                FVector LocalPos = InvTransform.TransformPosition(ProxyParticles[i].Position);
                Mesh->SetVertex(i, FVector3d(LocalPos), false); 
            });
        }
        else
        {
            // 如果使用了代理（Proxy），则依然由蒙皮后的 HighResParticles 决定
            ParallelFor(HighResParticleCount, [&](int32 i)
            {
                FVector LocalPos = InvTransform.TransformPosition(HighResParticles[i].Position);
                Mesh->SetVertex(i, FVector3d(LocalPos), false); 
            });
        }
        double EndTime = FPlatformTime::Seconds();
        float CostTimeMS = (EndTime - StartTime) * 1000.0f; 

        if (GEngine)
        {
            FString Msg = FString::Printf(TEXT("SetVertex Cost: %.2f ms !!"), CostTimeMS);
            GEngine->AddOnScreenDebugMessage(995, 0.0f, CostTimeMS > 16.0f ? FColor::Red : FColor::Green, Msg);
        }
    }
    
    
    if (bUseProxy && bShowProxyNodes && ProxyParticles.Num() > 0)
    {
        // 1. 将回读的 FVector3f 转换为 DrawDebugMesh 需要的 FVector
        TArray<FVector> DebugVerts;
        DebugVerts.SetNumUninitialized(ProxyParticles.Num());
        for (int32 i = 0; i < ProxyParticles.Num(); ++i)
        {
            DebugVerts[i] = FVector(ProxyParticles[i].Position);
        }
        UWorld* World = GetWorld();
        if (World)
        {
            // ==========================================
            // 2. 获取摄像机位置用于消隐计算
            // ==========================================
            FVector CameraLoc = FVector::ZeroVector;
            APlayerController* PC = World->GetFirstPlayerController();
            if (PC && PC->PlayerCameraManager)
            {
                CameraLoc = PC->PlayerCameraManager->GetCameraLocation();
            }

            // 3. 画出骨架线 (Wireframe) 并消隐背面
            for (int32 i = 0; i < ProxyDebugIndices.Num(); i += 3)
            {
                // 获取三角形的三个顶点索引
                int32 IdxA = ProxyDebugIndices[i];
                int32 IdxB = ProxyDebugIndices[i + 1];
                int32 IdxC = ProxyDebugIndices[i + 2];

                // 获取对应的顶点坐标 (当前帧的动态世界坐标)
                const FVector& PtA = DebugVerts[IdxA];
                const FVector& PtB = DebugVerts[IdxB];
                const FVector& PtC = DebugVerts[IdxC];

                // --------------------------------------------------
                // 【核心新增】：动态面片剔除 (Backface Culling)
                // --------------------------------------------------
                // 计算当前形变后的面法线 (左手系 + CCW 绕序)
                FVector FaceNormal = FVector::CrossProduct(PtC - PtA, PtB - PtA).GetSafeNormal();

                // 计算三角形中心点，以获取更精准的视线向量
                FVector TriCenter = (PtA + PtB + PtC) / 3.0f;
                FVector ViewDir = (CameraLoc - TriCenter).GetSafeNormal();

                // 如果面背对摄像机（或者处于侧向极限夹角），则跳过绘制
                // 0.0f 表示严格背面剔除，0.05f 能够把极度贴近侧边、快要转过去的面也藏起来，画面更干净
                if (FVector::DotProduct(FaceNormal, ViewDir) < 0.05f)
                {
                    continue; 
                }
                // --------------------------------------------------

                // 画出三角形的三条边 (红色线框)
                // 参数: World, 起点, 终点, 颜色, 是否持久, 寿命, 深度优先级, 线宽
                DrawDebugLine(World, PtA, PtB, FColor::Red, false, -1.0f, 0, 0.5f);
                DrawDebugLine(World, PtB, PtC, FColor::Red, false, -1.0f, 0, 0.5f);
                DrawDebugLine(World, PtC, PtA, FColor::Red, false, -1.0f, 0, 0.5f);
            }
        }
    }
    // 更新高模法线！
    if (!bUpdateViaGPU && Mesh->HasAttributes() && Mesh->Attributes()->PrimaryNormals() && HighResCurrentNormals.Num() == HighResParticleCount)
    {
        UE::Geometry::FDynamicMeshNormalOverlay* NormalOverlay = Mesh->Attributes()->PrimaryNormals();
        
        if (!bUseProxy && CurrentNormals.Num() == ProxyParticleCount)
        {
            // 如果没开代理，直接拿本体的模拟法线覆盖
            ParallelFor(ProxyParticleCount, [&](int32 Vid)
            {
                if (Mesh->IsVertex(Vid))
                {
                    FVector3f N = (FVector3f)InvTransform.TransformVector(CurrentNormals[Vid]).GetSafeNormal();
                    for (int32 ElemID : CachedHighResNormalElements[Vid])
                    {
                        NormalOverlay->SetElement(ElemID, N);
                    }
                }
            });
        }
        else if (bUseProxy && HighResCurrentNormals.Num() == HighResParticleCount)
        {
            // 如果开了代理，使用蒙皮计算回读的高模法线覆盖
            ParallelFor(HighResParticleCount, [&](int32 Vid)
            {
                if (Mesh->IsVertex(Vid))
                {
                    FVector3f N = (FVector3f)InvTransform.TransformVector(HighResCurrentNormals[Vid]).GetSafeNormal();
                    for (int32 ElemID : CachedHighResNormalElements[Vid])
                    {
                        NormalOverlay->SetElement(ElemID, N);
                    }
                }
            });
        }
    }
    if (bShow_Normals)
    {
        // 绘制高模法线 (绿色，较短，细 0.5f，全量无死角绘制)
        if (HighResCurrentNormals.Num() == HighResParticleCount)
        {
            for (int32 i = 0; i < HighResParticleCount; ++i)
            {
                FVector Start = HighResParticles[i].Position;
                // 高模法线画短一点 (10.0f)，防止密集的线把屏幕糊成一团绿
                FVector End = Start + HighResCurrentNormals[i].GetSafeNormal() * 10.0f; 
                DrawDebugLine(GetWorld(), Start, End, FColor::Green, false, -1.0f, 0, 0.5f);
            }
        }
    }
    if (bShowSingleVertexConstraints && SelectedVertexID >= 0 && SelectedVertexID < ProxyParticleCount)
    {
        FVector CenterPos = ProxyParticles[SelectedVertexID].Position;
        
        // 1. 画一个显眼的黄色球标记选中的顶点
        DrawDebugSphere(GetWorld(), CenterPos, ParticleRadius * 1.5f, 12, FColor::Yellow, false, -1.0f, 0, 1.0f);

        // 准备一个递增的 Key，用于屏幕左上角打印时不闪烁、不重叠
        int32 MsgKey = 1000;

        // 2. 距离约束 (黑色线)
        for (const FMySoftBodyConstraint& Con : Constraints)
        {
            if (Con.P0_Index == SelectedVertexID || Con.P1_Index == SelectedVertexID)
            {
                int32 NeighborID = (Con.P0_Index == SelectedVertexID) ? Con.P1_Index : Con.P0_Index;
                FVector NeighborPos = ProxyParticles[NeighborID].Position;
        
                DrawDebugLine(GetWorld(), CenterPos, NeighborPos, FColor::Black, false, -1.0f, 0, 1.0f);

                // 计算当前长度
                float CurrentLen = FVector::Dist(CenterPos, NeighborPos);

                // 方式 A：在 3D 场景中，线的中心点悬浮显示 "当前长度 / 原始长度"
                FVector MidPoint = (CenterPos + NeighborPos) * 0.5f;
                DrawDebugString(GetWorld(), MidPoint, FString::Printf(TEXT("%.2f / %.2f"), CurrentLen, Con.RestLength), nullptr, FColor::Black, 0.0f, false, 0.85f);
        
                // 方式 B：在屏幕左上角打印详细信息
                if (GEngine) GEngine->AddOnScreenDebugMessage(MsgKey++, 0.0f, FColor::Black, FString::Printf(TEXT("Dist [%d->%d]: Cur %.2f (Rest %.2f)"), SelectedVertexID, NeighborID, CurrentLen, Con.RestLength));
            }
        }

        // 3. 弯曲约束 (蓝色线)
        if (bUse_BendingForce)
        {
            for (const FMySoftBodyConstraint& Con : BendingConstraints)
            {
                if (Con.P0_Index == SelectedVertexID || Con.P1_Index == SelectedVertexID)
                {
                    int32 NeighborID = (Con.P0_Index == SelectedVertexID) ? Con.P1_Index : Con.P0_Index;
                    FVector NeighborPos = ProxyParticles[NeighborID].Position;
            
                    DrawDebugLine(GetWorld(), CenterPos, NeighborPos, FColor::Blue, false, -1.0f, 0, 1.0f);

                    float CurrentLen = FVector::Dist(CenterPos, NeighborPos);
            
                    FVector MidPoint = (CenterPos + NeighborPos) * 0.5f;
                    DrawDebugString(GetWorld(), MidPoint, FString::Printf(TEXT("%.2f / %.2f"), CurrentLen, Con.RestLength), nullptr, FColor::Blue, 0.0f, false, 0.85f);

                    if (GEngine) GEngine->AddOnScreenDebugMessage(MsgKey++, 0.0f, FColor::Blue, FString::Printf(TEXT("Bend [%d->%d]: Cur %.2f (Rest %.2f)"), SelectedVertexID, NeighborID, CurrentLen, Con.RestLength));
                }
            }
        }

        // 4. 内部支撑约束 (紫色线)
        if (bUse_InternalConstraints)
        {
            for (const FMySoftBodyConstraint& Con : InternalConstraints)
            {
                if (Con.P0_Index == SelectedVertexID || Con.P1_Index == SelectedVertexID)
                {
                    int32 NeighborID = (Con.P0_Index == SelectedVertexID) ? Con.P1_Index : Con.P0_Index;
                    FVector NeighborPos = ProxyParticles[NeighborID].Position;
            
                    DrawDebugLine(GetWorld(), CenterPos, NeighborPos, FColor::Magenta, false, -1.0f, 0, 1.0f);

                    float CurrentLen = FVector::Dist(CenterPos, NeighborPos);
            
                    FVector MidPoint = (CenterPos + NeighborPos) * 0.5f;
                    DrawDebugString(GetWorld(), MidPoint, FString::Printf(TEXT("%.2f / %.2f"), CurrentLen, Con.RestLength), nullptr, FColor::Magenta, 0.0f, false, 0.85f);

                    if (GEngine) GEngine->AddOnScreenDebugMessage(MsgKey++, 0.0f, FColor::Magenta, FString::Printf(TEXT("Internal [%d->%d]: Cur %.2f (Rest %.2f)"), SelectedVertexID, NeighborID, CurrentLen, Con.RestLength));
                }
            }
        }
    }
    // 3. 通知 GPU 更新
    if (!bUpdateViaGPU)
    {
        if (FDynamicMeshSceneProxy* Proxy = (FDynamicMeshSceneProxy*)SceneProxy)
        {
            Proxy->FastUpdateVertices(true, true, false, false);
            MarkRenderTransformDirty(); 
        }
    }
    // ==========================================
    // 调试：按低模面 ID 给高模顶点着色 (带背面消隐)
    // ==========================================
    if (bShowBindingRegions && ScaffoldBindings.Num() == HighResParticleCount)
    {
        UWorld* World = GetWorld();
        if (World)
        {
            // 1. 获取当前摄像机位置
            FVector CameraLoc = FVector::ZeroVector;
            APlayerController* PC = World->GetFirstPlayerController();
            if (PC && PC->PlayerCameraManager)
            {
                CameraLoc = PC->PlayerCameraManager->GetCameraLocation();
            }

            // 获取组件 Transform，用于把局部法线转到世界空间
            FTransform CompTransform = GetComponentTransform();

            for (int32 i = 0; i < HighResParticleCount; ++i)
            {
                int32 TriIndex = ScaffoldBindings[i].ProxyTriIndex;
                
                if (TriIndex < 0) continue;

                // 获取高模顶点当前的世界坐标
                FVector HighResPos = HighResParticles[i].Position;

                // --------------------------------------------------
                // 【核心新增】：视线剔除 (Backface Culling)
                // --------------------------------------------------
                // 提取它绑定的低模面，计算出局部面法线
                UE::Geometry::FIndex3i Tri = ProxyMesh.GetTriangle(TriIndex);
                FVector A = (FVector)ProxyMesh.GetVertex(Tri.A);
                FVector B = (FVector)ProxyMesh.GetVertex(Tri.B);
                FVector C = (FVector)ProxyMesh.GetVertex(Tri.C);
                
                FVector LocalFaceNormal = FVector::CrossProduct(C - A, B - A).GetSafeNormal();
                
                // 将法线转换到世界空间
                FVector WorldFaceNormal = CompTransform.TransformVectorNoScale(LocalFaceNormal);

                // 计算视线方向 (从顶点指向摄像机)
                FVector ViewDir = (CameraLoc - HighResPos).GetSafeNormal();

                // 视锥点乘：如果面背对摄像机，点乘结果为负。
                // 这里加了 0.05f 的微小容差，把侧面马上要转过去的面也剔除掉，画面更干净
                if (FVector::DotProduct(WorldFaceNormal, ViewDir) < 0.05f)
                {
                    continue; 
                }
                // --------------------------------------------------

                // 利用黄金比例生成区别明显的颜色
                uint8 Hue = (TriIndex * 137) % 256; 
                FColor RegionColor = FLinearColor::MakeFromHSV8(Hue, 220, 255).ToFColor(true);

                // 绘制调试点
                DrawDebugPoint(World, HighResPos, 10.0f, RegionColor, false, -1.0f);
            }
        }
    }
}


// =========================================================
// 体积保持 
// =========================================================
void UMySoftBodyMeshComponent::UpdateWeightedNormals()
{
    // 1. 清零法线数组 (必须，因为我们要累加)
    if (CurrentNormals.Num() != ProxyParticleCount)
    {
        CurrentNormals.SetNumZeroed(ProxyParticleCount);
    }
    else
    {
        FMemory::Memzero(CurrentNormals.GetData(), ProxyParticleCount * sizeof(FVector));
    }

    // 2. 遍历三角形，累加面法线到顶点
    // 这一步虽然遍历了三角形，但在 GPU 上可以对应 "Scatter" 操作，或者每个线程处理一个三角形去原子加顶点
    for (const FSoftBodyTriangle& Tri : SimulationTriangles)
    {
        const FVector& P0 = ProxyParticles[Tri.A].Position;
        const FVector& P1 = ProxyParticles[Tri.B].Position;
        const FVector& P2 = ProxyParticles[Tri.C].Position;

        // 计算未归一化的面法线 (Cross Product)
        // 长度 = 2 * 三角形面积
        FVector FaceNormalArea = (P2 - P0) ^ (P1 - P0); 

        // 累加到三个顶点
        // 这样每个顶点的 CurrentNormal 就代表了它周围一圈三角形的面积加权方向
        CurrentNormals[Tri.A] += FaceNormalArea;
        CurrentNormals[Tri.B] += FaceNormalArea;
        CurrentNormals[Tri.C] += FaceNormalArea;
    }
    
    // 注意：这里【不要】归一化 (Normalize)！
    // 我们需要保留向量的长度，它代表了局部面积大小。
}

float UMySoftBodyMeshComponent::CalcClothVolume()
{
    float TotalVolume = 0.0;

    // [核心修改] 遍历顶点计算体积
    // 这种方式非常适合并行化，每个顶点只读取自己的数据
    for (int32 i = 0; i < ProxyParticleCount; ++i)
    {
        // 散度公式顶点版： V = (Position . WeightedNormal) / 3
        // 因为我们的 WeightedNormal 在计算时没有除以2 (三角形面积公式是 1/2 * Cross)，
        // 所以这里的分母应该是 6 (2 * 3)
        double Volume=ProxyParticles[i].Position | CurrentNormals[i];
        TotalVolume += Volume;
    }

    return (TotalVolume / 6.0);
}

void UMySoftBodyMeshComponent::VolumePreservation()
{
    if(!bUse_VolumePressureForce) return;

    curVolume = CalcClothVolume();
    
    /*// 安全检查：如果体积崩坏了，就不再施加压力
    if (abs(curVolume) <= 0.1f) 
    {
        return;
    }*/
    // 如果体积变小了 (delta < 0)，就需要施加向外的压力
    VolumePressureForce(0);
}

void UMySoftBodyMeshComponent::VolumePressureForce(int32 mode)
{
    if (mode == 0)
    {
        if (CurrentNormals.Num() != ProxyParticleCount) return;

        float VolumeRatio = (restVolume - curVolume) / (restVolume + 1.0f);
        if (VolumeRatio < 0)
        {
            VolumeRatio *= 0.1f; // 削弱回弹力，只给 10% 的力拉回来
        }
        float PressureMag = VolumeRatio * VolPressure_Coefficient;
        float MaxForce = 5000.0f; 
        PressureMag = FMath::Clamp(PressureMag, -MaxForce, MaxForce);

        for(int32 i=0; i<ProxyParticleCount; ++i)
        {
            // 沿着法线方向推
            ProxyParticles[i].Force += CurrentNormals[i] * PressureMag;
        }
    }
}

// =========================================================
// GPU 资源管理
// =========================================================

void UMySoftBodyMeshComponent::BuildAdjacencyData()
{
    // 构建 CSR 格式的邻接表
    SurfaceAdjacencyIndices.Empty();
    SurfaceNeighborOffsets.SetNumZeroed(ProxyParticleCount);
    SurfaceNeighborCounts.SetNumZeroed(ProxyParticleCount);

    InternalAdjacencyIndices.Empty();
    InternalNeighborOffsets.SetNumZeroed(ProxyParticleCount);
    InternalNeighborCounts.SetNumZeroed(ProxyParticleCount);

    TArray<TArray<int32>> TempSurfaceAdjacency;
    TArray<TArray<int32>> TempInternalAdjacency;
    TempSurfaceAdjacency.SetNum(ProxyParticleCount);
    TempInternalAdjacency.SetNum(ProxyParticleCount);
    // 1.距离约束 (放入 Surface)
    for (int32 i = 0; i < Constraints.Num(); ++i)
    {
        const FMySoftBodyConstraint& Con = Constraints[i];
        TempSurfaceAdjacency[Con.P0_Index].Add(i);
        TempSurfaceAdjacency[Con.P1_Index].Add(i);
    }

    // 2. 弯曲约束 (放入 Surface)
    int32 OffsetBending = Constraints.Num();
    if (bUse_BendingForce)
    {
        for (int32 i = 0; i < BendingConstraints.Num(); ++i)
        {
            const FMySoftBodyConstraint& Con = BendingConstraints[i];
            int32 GlobalIdx = OffsetBending + i;
            TempSurfaceAdjacency[Con.P0_Index].Add(GlobalIdx);
            TempSurfaceAdjacency[Con.P1_Index].Add(GlobalIdx);
        }
    }
    
    // 3. 内部约束 (放入 Internal)
    int32 OffsetInternal = OffsetBending + (bUse_BendingForce ? BendingConstraints.Num() : 0);
    if (bUse_InternalConstraints)
    {
        for (int32 i = 0; i < InternalConstraints.Num(); ++i)
        {
            const FMySoftBodyConstraint& Con = InternalConstraints[i];
            int32 GlobalIdx = OffsetInternal + i;
            TempInternalAdjacency[Con.P0_Index].Add(GlobalIdx);
            TempInternalAdjacency[Con.P1_Index].Add(GlobalIdx);
        }
    }
    // ==========================================
    // 3. 构建双份 CSR 扁平化数组
    // ==========================================
    uint32 SurfaceOffset = 0;
    uint32 InternalOffset = 0;

    for (int32 i = 0; i < ProxyParticleCount; ++i)
    {
        // 构建 Surface
        SurfaceNeighborOffsets[i] = SurfaceOffset;
        SurfaceNeighborCounts[i] = TempSurfaceAdjacency[i].Num();
        for (int32 ConIdx : TempSurfaceAdjacency[i])
        {
            SurfaceAdjacencyIndices.Add(ConIdx);
        }
        SurfaceOffset += SurfaceNeighborCounts[i];

        // 构建 Internal
        InternalNeighborOffsets[i] = InternalOffset;
        InternalNeighborCounts[i] = TempInternalAdjacency[i].Num();
        for (int32 ConIdx : TempInternalAdjacency[i])
        {
            InternalAdjacencyIndices.Add(ConIdx);
        }
        InternalOffset += InternalNeighborCounts[i];
    }
    
    DihedralAdjacencyIndices.Empty();
    DihedralNeighborOffsets.SetNumZeroed(ProxyParticleCount);
    DihedralNeighborCounts.SetNumZeroed(ProxyParticleCount);

    if (bUse_DihedralBending)
    {
        TArray<TArray<int32>> TempDihedralAdjacency;
        TempDihedralAdjacency.SetNum(ProxyParticleCount);

        // 遍历所有二面角约束，把它分配给相关的 4 个顶点
        for (int32 i = 0; i < DihedralConstraints.Num(); ++i)
        {
            const FMySoftBodyDihedralConstraint& Con = DihedralConstraints[i];
            TempDihedralAdjacency[Con.P1_Index].Add(i);
            TempDihedralAdjacency[Con.P2_Index].Add(i);
            TempDihedralAdjacency[Con.P3_Index].Add(i);
            TempDihedralAdjacency[Con.P4_Index].Add(i);
        }

        uint32 DihedralOffset = 0;
        for (int32 i = 0; i < ProxyParticleCount; ++i)
        {
            DihedralNeighborOffsets[i] = DihedralOffset;
            DihedralNeighborCounts[i] = TempDihedralAdjacency[i].Num();
            for (int32 ConIdx : TempDihedralAdjacency[i])
            {
                DihedralAdjacencyIndices.Add(ConIdx);
            }
            DihedralOffset += DihedralNeighborCounts[i];
        }
    }
}


void UMySoftBodyMeshComponent::InitGPUResources()
{
    if (bGPUResourcesInitialized || ProxyParticleCount == 0) return;

    // 1. 构建邻接表数据
    BuildAdjacencyData();
    // 2. 准备粒子数据
    TArray<FGPUParticle> ProxyParticleData;
    TArray<FGPUParticle> HighResParticleData;
    HighResParticleData.SetNum(HighResParticleCount);
    ProxyParticleData.SetNum(ProxyParticleCount);
    for (int32 i = 0; i < ProxyParticleCount; ++i)
    {
        ProxyParticleData[i].Position = FVector3f(ProxyParticles[i].Position);
        ProxyParticleData[i].PrevPosition = FVector3f(ProxyParticles[i].PrevPosition);
        ProxyParticleData[i].Velocity = FVector3f(ProxyParticles[i].Velocity);
        ProxyParticleData[i].InvMass = ProxyParticles[i].InvMass;
        ProxyParticleData[i].State = ProxyParticles[i].state;
    }
    for (int32 i = 0; i < HighResParticleCount; ++i)
    {
        HighResParticleData[i].Position = FVector3f(HighResParticles[i].Position);
        HighResParticleData[i].PrevPosition = FVector3f(HighResParticles[i].PrevPosition);
        HighResParticleData[i].Velocity = FVector3f(HighResParticles[i].Velocity);
        HighResParticleData[i].InvMass = HighResParticles[i].InvMass;
        HighResParticleData[i].State = HighResParticles[i].state;
    }
    // 准备高模初始位置数据 (FVector -> FVector3f)
    TArray<FVector3f> HighResRestPosData;
    HighResRestPosData.SetNumUninitialized(HighResParticleCount);
    for (int32 i = 0; i < HighResParticleCount; ++i)
    {
        HighResRestPosData[i] = (FVector3f)HighResRestPositions[i];
    }
    // 3. 准备约束数据
    int32 CountBasic = Constraints.Num();
    int32 CountBending = bUse_BendingForce ? BendingConstraints.Num() : 0;
    int32 CountInternal = bUse_InternalConstraints ? InternalConstraints.Num() : 0; 

    int32 TotalConstraints = CountBasic + CountBending + CountInternal;
    
    TArray<FGPUConstraint> ConstraintData;
    ConstraintData.SetNum(TotalConstraints);
    // 距离约束
    for (int32 i = 0; i < Constraints.Num(); ++i)
    {
        ConstraintData[i].P0 = Constraints[i].P0_Index;
        ConstraintData[i].P1 = Constraints[i].P1_Index;
        ConstraintData[i].RestLength = Constraints[i].RestLength;
        ConstraintData[i].Stiffness = XPBD_StretchStiffness;
    }
    // 弯曲约束
    if (bUse_BendingForce)
    {
        int32 Offset = Constraints.Num();
        for (int32 i = 0; i < BendingConstraints.Num(); ++i)
        {
            int32 idx = Offset + i;
            ConstraintData[idx].P0 = BendingConstraints[i].P0_Index;
            ConstraintData[idx].P1 = BendingConstraints[i].P1_Index;
            ConstraintData[idx].RestLength = BendingConstraints[i].RestLength;
            ConstraintData[idx].Stiffness = XPBD_BendingStiffness;
        }
    }
    // 内部约束
    int32 OffsetInternal = CountBasic + CountBending;
    if (bUse_InternalConstraints)
    {
        for (int32 i = 0; i < CountInternal; ++i)
        {
            int32 idx = OffsetInternal + i;
            ConstraintData[idx].P0 = InternalConstraints[i].P0_Index;
            ConstraintData[idx].P1 = InternalConstraints[i].P1_Index;
            ConstraintData[idx].RestLength = InternalConstraints[i].RestLength;
            ConstraintData[idx].Stiffness = XPBD_InternalStiffness; // 使用内部刚度
        }
    }
    
    // 准备二面角约束数据
    int32 NumDihedrals = bUse_DihedralBending ? DihedralConstraints.Num() : 0;
    TArray<FGPUDihedralConstraint> DihedralData;
    TArray<FGPUNeighborInfo> DihedralNeighborData;
    
    if (NumDihedrals > 0)
    {
        DihedralData.SetNum(NumDihedrals);
        for (int32 i = 0; i < NumDihedrals; ++i)
        {
            DihedralData[i].P1 = DihedralConstraints[i].P1_Index;
            DihedralData[i].P2 = DihedralConstraints[i].P2_Index;
            DihedralData[i].P3 = DihedralConstraints[i].P3_Index;
            DihedralData[i].P4 = DihedralConstraints[i].P4_Index;
            DihedralData[i].RestAngle = DihedralConstraints[i].RestAngle;
            DihedralData[i].Stiffness = XPBD_DihedralStiffness;
        }

        DihedralNeighborData.SetNum(ProxyParticleCount);
        for (int32 i = 0; i < ProxyParticleCount; ++i)
        {
            DihedralNeighborData[i].StartIndex = DihedralNeighborOffsets[i];
            DihedralNeighborData[i].Count = DihedralNeighborCounts[i];
        }
    }
    TArray<uint32> LocalDihedralAdjData = DihedralAdjacencyIndices;
    int32 NumDihedralAdjacency = LocalDihedralAdjData.Num();
    
    // 4. 准备 Lambda 数据
    TArray<FGPULambda> LambdaData;
    LambdaData.SetNum(TotalConstraints);
    for (auto& L : LambdaData) L.Value = 0.0f;

    // 5. 准备邻接表数据
    TArray<FGPUNeighborInfo> SurfaceNeighborData;
    SurfaceNeighborData.SetNum(ProxyParticleCount);
    
    TArray<FGPUNeighborInfo> InternalNeighborData;
    InternalNeighborData.SetNum(ProxyParticleCount);

    for (int32 i = 0; i < ProxyParticleCount; ++i)
    {
        // 表面约束的 Offset 和 Count
        SurfaceNeighborData[i].StartIndex = SurfaceNeighborOffsets[i];
        SurfaceNeighborData[i].Count = SurfaceNeighborCounts[i];
        
        // 内部约束的 Offset 和 Count
        InternalNeighborData[i].StartIndex = InternalNeighborOffsets[i];
        InternalNeighborData[i].Count = InternalNeighborCounts[i];
    }

    int32 NumSurfaceAdjacency = SurfaceAdjacencyIndices.Num();
    int32 NumInternalAdjacency = InternalAdjacencyIndices.Num();

    int32 NumConstraints = TotalConstraints;
    int32 NumProxyParticles = ProxyParticleCount;
    int32 NumHighResParticleCount = HighResParticleCount;
    int32 NumTriangles = ProxyTriCount;
    int32 NumVertexTriangleIndices = VertexTriangleIndices.Num();
    TArray<FGPUTriangle> TriangleData;
    if (NumTriangles > 0)
    {
        TriangleData.SetNum(NumTriangles);
        for (int32 i = 0; i < NumTriangles; ++i)
        {
            TriangleData[i].A = SimulationTriangles[i].A;
            TriangleData[i].B = SimulationTriangles[i].B;
            TriangleData[i].C = SimulationTriangles[i].C;
        }
    }
    
    int32 NumHighResTris = HighResSimulationTriangles.Num();
    TArray<FGPUTriangle> HighResTriangleData;
    
    if (NumHighResTris > 0)
    {
        HighResTriangleData.SetNumUninitialized(NumHighResTris);
        for (int32 i = 0; i < NumHighResTris; ++i)
        {
            HighResTriangleData[i].A = HighResSimulationTriangles[i].A;
            HighResTriangleData[i].B = HighResSimulationTriangles[i].B;
            HighResTriangleData[i].C = HighResSimulationTriangles[i].C;
        }
    }

    // 复制拓扑结构到局部数组
    TArray<uint32> LocalHighResVtxTriOffsets = HighResVertexTriangleOffsets;
    TArray<uint32> LocalHighResVtxTriCounts = HighResVertexTriangleCounts;
    TArray<uint32> LocalHighResVtxTriIndices = HighResVertexTriangleIndices;
    int32 NumHighResVtxTriIndices = LocalHighResVtxTriIndices.Num();
    
    // 1. 拷贝绑定数据到局部变量 (供渲染线程捕获)
    TArray<FScaffoldBinding> LocalBindings = ScaffoldBindings;

    // 2. 提取并打包低模(Proxy)的三角形拓扑数据
    TArray<FGPUTriangle> LocalProxyTriangles;
    if (bUseProxy)
    {
        LocalProxyTriangles.SetNumUninitialized(ProxyMesh.TriangleCount());
        int32 TriIdx = 0;
        for (int32 tid : ProxyMesh.TriangleIndicesItr())
        {
            UE::Geometry::FIndex3i Tri = ProxyMesh.GetTriangle(tid);
            LocalProxyTriangles[TriIdx].A = Tri.A;
            LocalProxyTriangles[TriIdx].B = Tri.B;
            LocalProxyTriangles[TriIdx].C = Tri.C;
            TriIdx++;
        }
    }
    
    ENQUEUE_RENDER_COMMAND(InitSoftBodyGPU)(
        [this, ProxyParticleData = MoveTemp(ProxyParticleData),
         HighResParticleData = MoveTemp(HighResParticleData),
         LocalHighResRestPosData = MoveTemp(HighResRestPosData),
         ConstraintData = MoveTemp(ConstraintData),
         LambdaData = MoveTemp(LambdaData),
         SurfaceNeighborData = MoveTemp(SurfaceNeighborData),
         InternalNeighborData = MoveTemp(InternalNeighborData),
         LocalSurfaceAdjData = SurfaceAdjacencyIndices,
         LocalInternalAdjData = InternalAdjacencyIndices,
         TriangleData = MoveTemp(TriangleData), 
         HighResTriangleData = MoveTemp(HighResTriangleData),
         LocalHighResVtxTriOffsets = MoveTemp(LocalHighResVtxTriOffsets),
         LocalHighResVtxTriCounts = MoveTemp(LocalHighResVtxTriCounts),
         LocalHighResVtxTriIndices = MoveTemp(LocalHighResVtxTriIndices),
         LocalBindings = MoveTemp(LocalBindings),             
         LocalProxyTriangles = MoveTemp(LocalProxyTriangles),
         LocalDihedralData = MoveTemp(DihedralData),
         LocalDihedralNeighborData = MoveTemp(DihedralNeighborData),
         LocalDihedralAdjData = LocalDihedralAdjData,
         NumHighResTris, NumHighResVtxTriIndices, NumDihedrals,NumDihedralAdjacency,
         NumProxyParticles,NumHighResParticleCount, NumSurfaceAdjacency, NumInternalAdjacency,NumConstraints, NumTriangles,NumVertexTriangleIndices]
        (FRHICommandListImmediate& RHICmdList)
        {
            
            // 粒子 Buffer
            if (NumProxyParticles > 0)
            {
                FRDGBufferDesc Desc = FRDGBufferDesc::CreateStructuredDesc(sizeof(FGPUParticle), NumProxyParticles);
                Desc.Usage |= BUF_UnorderedAccess | BUF_SourceCopy;
                ProxyParticlePooledBuffer = AllocatePooledBuffer(Desc, TEXT("SoftBodyParticles"));
                void* Data = RHICmdList.LockBuffer(ProxyParticlePooledBuffer->GetRHI(), 0, NumProxyParticles * sizeof(FGPUParticle), RLM_WriteOnly);
                FMemory::Memcpy(Data, ProxyParticleData.GetData(), NumProxyParticles * sizeof(FGPUParticle));
                RHICmdList.UnlockBuffer(ProxyParticlePooledBuffer->GetRHI());
            }
            if (NumHighResParticleCount > 0)
            {
                FRDGBufferDesc Desc = FRDGBufferDesc::CreateStructuredDesc(sizeof(FGPUParticle), NumHighResParticleCount);
                Desc.Usage |= BUF_UnorderedAccess | BUF_SourceCopy;
                HighResParticlePooledBuffer = AllocatePooledBuffer(Desc, TEXT("HighResParticles"));
                void* HighResData = RHICmdList.LockBuffer(HighResParticlePooledBuffer->GetRHI(), 0, NumHighResParticleCount * sizeof(FGPUParticle), RLM_WriteOnly);
                FMemory::Memcpy(HighResData, HighResParticleData.GetData(), NumHighResParticleCount * sizeof(FGPUParticle));
                RHICmdList.UnlockBuffer(HighResParticlePooledBuffer->GetRHI());
            }
            if (NumHighResParticleCount > 0)
            {
                FRDGBufferDesc Desc = FRDGBufferDesc::CreateStructuredDesc(sizeof(FVector3f), NumHighResParticleCount);
                HighResRestPosPooledBuffer = AllocatePooledBuffer(Desc, TEXT("HighResRestPositions"));
                void* Data = RHICmdList.LockBuffer(HighResRestPosPooledBuffer->GetRHI(), 0, NumHighResParticleCount * sizeof(FVector3f), RLM_WriteOnly);
                FMemory::Memcpy(Data, LocalHighResRestPosData.GetData(), NumHighResParticleCount * sizeof(FVector3f));
                RHICmdList.UnlockBuffer(HighResRestPosPooledBuffer->GetRHI());
            }
            // 约束 Buffer
            if (NumConstraints > 0)
            {
                FRDGBufferDesc Desc = FRDGBufferDesc::CreateStructuredDesc(sizeof(FGPUConstraint), NumConstraints);
                ConstraintPooledBuffer = AllocatePooledBuffer(Desc, TEXT("SoftBodyConstraints"));
                void* Data = RHICmdList.LockBuffer(ConstraintPooledBuffer->GetRHI(), 0, NumConstraints * sizeof(FGPUConstraint), RLM_WriteOnly);
                FMemory::Memcpy(Data, ConstraintData.GetData(), NumConstraints * sizeof(FGPUConstraint));
                RHICmdList.UnlockBuffer(ConstraintPooledBuffer->GetRHI());
            }

            // Lambda Buffer
            if (NumConstraints > 0)
            {
                FRDGBufferDesc Desc = FRDGBufferDesc::CreateStructuredDesc(sizeof(FGPULambda), NumConstraints);
                Desc.Usage |= BUF_UnorderedAccess;
                LambdaPooledBuffer = AllocatePooledBuffer(Desc, TEXT("SoftBodyLambdas"));
                void* Data = RHICmdList.LockBuffer(LambdaPooledBuffer->GetRHI(), 0, NumConstraints * sizeof(FGPULambda), RLM_WriteOnly);
                FMemory::Memcpy(Data, LambdaData.GetData(), NumConstraints * sizeof(FGPULambda));
                RHICmdList.UnlockBuffer(LambdaPooledBuffer->GetRHI());
            }

            // 1. Surface NeighborInfo Buffer
            {
                FRDGBufferDesc Desc = FRDGBufferDesc::CreateStructuredDesc(sizeof(FGPUNeighborInfo), NumProxyParticles);
                SurfaceNeighborInfoPooledBuffer = AllocatePooledBuffer(Desc, TEXT("SurfaceNeighborInfo"));
                void* Data = RHICmdList.LockBuffer(SurfaceNeighborInfoPooledBuffer->GetRHI(), 0, NumProxyParticles * sizeof(FGPUNeighborInfo), RLM_WriteOnly);
                FMemory::Memcpy(Data, SurfaceNeighborData.GetData(), NumProxyParticles * sizeof(FGPUNeighborInfo));
                RHICmdList.UnlockBuffer(SurfaceNeighborInfoPooledBuffer->GetRHI());
            }

            // 2. Surface Adjacency Buffer
            if (NumSurfaceAdjacency > 0)
            {
                FRDGBufferDesc Desc = FRDGBufferDesc::CreateStructuredDesc(sizeof(uint32), NumSurfaceAdjacency);
                SurfaceAdjacencyPooledBuffer = AllocatePooledBuffer(Desc, TEXT("SurfaceAdjacency"));
                void* Data = RHICmdList.LockBuffer(SurfaceAdjacencyPooledBuffer->GetRHI(), 0, NumSurfaceAdjacency * sizeof(uint32), RLM_WriteOnly);
                FMemory::Memcpy(Data, LocalSurfaceAdjData.GetData(), NumSurfaceAdjacency * sizeof(uint32));
                RHICmdList.UnlockBuffer(SurfaceAdjacencyPooledBuffer->GetRHI());
            }

            // 3. Internal NeighborInfo Buffer
            {
                FRDGBufferDesc Desc = FRDGBufferDesc::CreateStructuredDesc(sizeof(FGPUNeighborInfo), NumProxyParticles);
                InternalNeighborInfoPooledBuffer = AllocatePooledBuffer(Desc, TEXT("InternalNeighborInfo"));
                void* Data = RHICmdList.LockBuffer(InternalNeighborInfoPooledBuffer->GetRHI(), 0, NumProxyParticles * sizeof(FGPUNeighborInfo), RLM_WriteOnly);
                FMemory::Memcpy(Data, InternalNeighborData.GetData(), NumProxyParticles * sizeof(FGPUNeighborInfo));
                RHICmdList.UnlockBuffer(InternalNeighborInfoPooledBuffer->GetRHI());
            }

            // 4. Internal Adjacency Buffer
            if (NumInternalAdjacency > 0)
            {
                FRDGBufferDesc Desc = FRDGBufferDesc::CreateStructuredDesc(sizeof(uint32), NumInternalAdjacency);
                InternalAdjacencyPooledBuffer = AllocatePooledBuffer(Desc, TEXT("InternalAdjacency"));
                void* Data = RHICmdList.LockBuffer(InternalAdjacencyPooledBuffer->GetRHI(), 0, NumInternalAdjacency * sizeof(uint32), RLM_WriteOnly);
                FMemory::Memcpy(Data, LocalInternalAdjData.GetData(), NumInternalAdjacency * sizeof(uint32));
                RHICmdList.UnlockBuffer(InternalAdjacencyPooledBuffer->GetRHI());
            }
            // === 二面角 Buffers ===
            if (NumDihedrals > 0)
            {
                // 1. 约束本体 Buffer (存 P1, P2, P3, P4, RestAngle, Stiffness)
                FRDGBufferDesc Desc = FRDGBufferDesc::CreateStructuredDesc(sizeof(FGPUDihedralConstraint), NumDihedrals);
                DihedralConstraintPooledBuffer = AllocatePooledBuffer(Desc, TEXT("DihedralConstraints"));
                void* Data = RHICmdList.LockBuffer(DihedralConstraintPooledBuffer->GetRHI(), 0, NumDihedrals * sizeof(FGPUDihedralConstraint), RLM_WriteOnly);
                FMemory::Memcpy(Data, LocalDihedralData.GetData(), NumDihedrals * sizeof(FGPUDihedralConstraint));
                RHICmdList.UnlockBuffer(DihedralConstraintPooledBuffer->GetRHI());

                // 2. Lambda Buffer (注意：需要设为 BUF_UnorderedAccess，且初始值清零)
                FRDGBufferDesc LambdaDesc = FRDGBufferDesc::CreateStructuredDesc(sizeof(FGPULambda), NumDihedrals);
                LambdaDesc.Usage |= BUF_UnorderedAccess; 
                DihedralLambdaPooledBuffer = AllocatePooledBuffer(LambdaDesc, TEXT("DihedralLambdas"));
                void* LamData = RHICmdList.LockBuffer(DihedralLambdaPooledBuffer->GetRHI(), 0, NumDihedrals * sizeof(FGPULambda), RLM_WriteOnly);
                FMemory::Memzero(LamData, NumDihedrals * sizeof(FGPULambda)); // 初始乘子为 0
                RHICmdList.UnlockBuffer(DihedralLambdaPooledBuffer->GetRHI());

                // 3. Neighbor Info Buffer (长度是粒子总数 NumProxyParticles)
                FRDGBufferDesc NeighborDesc = FRDGBufferDesc::CreateStructuredDesc(sizeof(FGPUNeighborInfo), NumProxyParticles);
                DihedralNeighborInfoPooledBuffer = AllocatePooledBuffer(NeighborDesc, TEXT("DihedralNeighborInfo"));
                void* NData = RHICmdList.LockBuffer(DihedralNeighborInfoPooledBuffer->GetRHI(), 0, NumProxyParticles * sizeof(FGPUNeighborInfo), RLM_WriteOnly);
                FMemory::Memcpy(NData, LocalDihedralNeighborData.GetData(), NumProxyParticles * sizeof(FGPUNeighborInfo));
                RHICmdList.UnlockBuffer(DihedralNeighborInfoPooledBuffer->GetRHI());

                // 4. Adjacency Buffer (邻接表具体的索引值)
                if (NumDihedralAdjacency > 0)
                {
                    FRDGBufferDesc AdjDesc = FRDGBufferDesc::CreateStructuredDesc(sizeof(uint32), NumDihedralAdjacency);
                    DihedralAdjacencyPooledBuffer = AllocatePooledBuffer(AdjDesc, TEXT("DihedralAdjacency"));
                    void* AData = RHICmdList.LockBuffer(DihedralAdjacencyPooledBuffer->GetRHI(), 0, NumDihedralAdjacency * sizeof(uint32), RLM_WriteOnly);
                    FMemory::Memcpy(AData, LocalDihedralAdjData.GetData(), NumDihedralAdjacency * sizeof(uint32));
                    RHICmdList.UnlockBuffer(DihedralAdjacencyPooledBuffer->GetRHI());
                }
            }
            // 三角形 Buffer
            if (NumTriangles > 0)
            {
                FRDGBufferDesc Desc = FRDGBufferDesc::CreateStructuredDesc(sizeof(FGPUTriangle), NumTriangles);
                TrianglePooledBuffer = AllocatePooledBuffer(Desc, TEXT("SoftBodyTriangles"));
                void* Data = RHICmdList.LockBuffer(TrianglePooledBuffer->GetRHI(), 0, NumTriangles * sizeof(FGPUTriangle), RLM_WriteOnly);
                FMemory::Memcpy(Data, TriangleData.GetData(), NumTriangles * sizeof(FGPUTriangle));
                RHICmdList.UnlockBuffer(TrianglePooledBuffer->GetRHI());
            }
            if (NumVertexTriangleIndices > 0)
            {
                // Offsets
                {
                    FRDGBufferDesc Desc = FRDGBufferDesc::CreateStructuredDesc(sizeof(uint32), NumProxyParticles);
                    VertexTriangleOffsetsPooledBuffer = AllocatePooledBuffer(Desc, TEXT("VertexTriangleOffsets"));
                    void* Data = RHICmdList.LockBuffer(VertexTriangleOffsetsPooledBuffer->GetRHI(), 0, NumProxyParticles * sizeof(uint32), RLM_WriteOnly);
                    FMemory::Memcpy(Data, VertexTriangleOffsets.GetData(), NumProxyParticles * sizeof(uint32));
                    RHICmdList.UnlockBuffer(VertexTriangleOffsetsPooledBuffer->GetRHI());
                }
    
                // Counts
                {
                    FRDGBufferDesc Desc = FRDGBufferDesc::CreateStructuredDesc(sizeof(uint32), NumProxyParticles);
                    VertexTriangleCountsPooledBuffer = AllocatePooledBuffer(Desc, TEXT("VertexTriangleCounts"));
                    void* Data = RHICmdList.LockBuffer(VertexTriangleCountsPooledBuffer->GetRHI(), 0, NumProxyParticles * sizeof(uint32), RLM_WriteOnly);
                    FMemory::Memcpy(Data, VertexTriangleCounts.GetData(), NumProxyParticles * sizeof(uint32));
                    RHICmdList.UnlockBuffer(VertexTriangleCountsPooledBuffer->GetRHI());
                }
    
                // Indices
                {
                    int32 NumIndices = VertexTriangleIndices.Num();
                    FRDGBufferDesc Desc = FRDGBufferDesc::CreateStructuredDesc(sizeof(uint32), NumIndices);
                    VertexTriangleIndicesPooledBuffer = AllocatePooledBuffer(Desc, TEXT("VertexTriangleIndices"));
                    void* Data = RHICmdList.LockBuffer(VertexTriangleIndicesPooledBuffer->GetRHI(), 0, NumIndices * sizeof(uint32), RLM_WriteOnly);
                    FMemory::Memcpy(Data, VertexTriangleIndices.GetData(), NumIndices * sizeof(uint32));
                    RHICmdList.UnlockBuffer(VertexTriangleIndicesPooledBuffer->GetRHI());
                }
            }

           // 加权法线 Buffer 
           {
                FRDGBufferDesc Desc = FRDGBufferDesc::CreateStructuredDesc(sizeof(FVector3f), NumProxyParticles); 
               Desc.Usage |= BUF_UnorderedAccess;
               WeightedNormalsPooledBuffer = AllocatePooledBuffer(Desc, TEXT("SoftBodyWeightedNormals"));
           }

           // 体积 Buffer (单个 int)
           {
               FRDGBufferDesc Desc = FRDGBufferDesc::CreateStructuredDesc(sizeof(int32), 1);
               Desc.Usage |= BUF_UnorderedAccess;
               VolumePooledBuffer = AllocatePooledBuffer(Desc, TEXT("SoftBodyVolume"));
           }
            // Position Output Buffer (用于 Readback)
            if (ProxyParticleCount > 0)
            {
                FRDGBufferDesc Desc = FRDGBufferDesc::CreateStructuredDesc(sizeof(FVector3f), NumProxyParticles);
                Desc.Usage |= BUF_UnorderedAccess | BUF_SourceCopy;
                ProxyPositionOutputPooledBuffer = AllocatePooledBuffer(Desc, TEXT("ProxyPositionOutput"));
            }
            // 上传 Scaffold Bindings Buffer
            if (bUseProxy && LocalBindings.Num() > 0)
            {
                FRDGBufferDesc BindDesc = FRDGBufferDesc::CreateStructuredDesc(sizeof(FScaffoldBinding), LocalBindings.Num());
                BindDesc.Usage |= BUF_UnorderedAccess | BUF_SourceCopy | BUF_ShaderResource;
                ScaffoldBindingsPooledBuffer = AllocatePooledBuffer(BindDesc, TEXT("ScaffoldBindingsBuffer"));
                
                void* BindData = RHICmdList.LockBuffer(ScaffoldBindingsPooledBuffer->GetRHI(), 0, LocalBindings.Num() * sizeof(FScaffoldBinding), RLM_WriteOnly);
                FMemory::Memcpy(BindData, LocalBindings.GetData(), LocalBindings.Num() * sizeof(FScaffoldBinding));
                RHICmdList.UnlockBuffer(ScaffoldBindingsPooledBuffer->GetRHI());
            }

            // 上传 Proxy Triangles Buffer
            if (bUseProxy && LocalProxyTriangles.Num() > 0)
            {
                FRDGBufferDesc TriDesc = FRDGBufferDesc::CreateStructuredDesc(sizeof(FGPUTriangle), LocalProxyTriangles.Num());
                TriDesc.Usage |= BUF_UnorderedAccess | BUF_SourceCopy | BUF_ShaderResource;
                ProxyTrianglePooledBuffer = AllocatePooledBuffer(TriDesc, TEXT("ProxyTriangleBuffer"));
                
                void* TriData = RHICmdList.LockBuffer(ProxyTrianglePooledBuffer->GetRHI(), 0, LocalProxyTriangles.Num() * sizeof(FGPUTriangle), RLM_WriteOnly);
                FMemory::Memcpy(TriData, LocalProxyTriangles.GetData(), LocalProxyTriangles.Num() * sizeof(FGPUTriangle));
                RHICmdList.UnlockBuffer(ProxyTrianglePooledBuffer->GetRHI());
            }
            // 1. 高模 Position Output Buffer (数量: HighResParticleCount)
            if (HighResParticleCount > 0)
            {
                FRDGBufferDesc Desc = FRDGBufferDesc::CreateStructuredDesc(sizeof(FVector3f), NumHighResParticleCount);
                Desc.Usage |= BUF_UnorderedAccess | BUF_SourceCopy;
                HighResPositionOutputPooledBuffer = AllocatePooledBuffer(Desc, TEXT("HighResPositionOutput"));
            }
            
            // 法线输出 Buffer
            {
                FRDGBufferDesc Desc = FRDGBufferDesc::CreateStructuredDesc(sizeof(FVector3f), NumProxyParticles);
                Desc.Usage |= BUF_UnorderedAccess | BUF_SourceCopy;
                NormalOutputPooledBuffer = AllocatePooledBuffer(Desc, TEXT("SoftBodyNormalOutput"));
            }
            // 体积输出 Buffer
            {
                FRDGBufferDesc Desc = FRDGBufferDesc::CreateStructuredDesc(sizeof(int32), 1);
                Desc.Usage |= BUF_UnorderedAccess | BUF_SourceCopy;
                VolumeOutputPooledBuffer = AllocatePooledBuffer(Desc, TEXT("SoftBodyVolumeOutput"));
            }
            if (NumHighResTris > 0)
            {
                // 三角形 Buffer
                FRDGBufferDesc TriDesc = FRDGBufferDesc::CreateStructuredDesc(sizeof(FGPUTriangle), NumHighResTris);
                HighResTrianglePooledBuffer = AllocatePooledBuffer(TriDesc, TEXT("HighResTriangles"));
                void* TriData = RHICmdList.LockBuffer(HighResTrianglePooledBuffer->GetRHI(), 0, NumHighResTris * sizeof(FGPUTriangle), RLM_WriteOnly);
                FMemory::Memcpy(TriData, HighResTriangleData.GetData(), NumHighResTris * sizeof(FGPUTriangle));
                RHICmdList.UnlockBuffer(HighResTrianglePooledBuffer->GetRHI());
            }

            if (NumHighResParticleCount > 0)
            {
                // 拓扑 Offsets
                FRDGBufferDesc OffDesc = FRDGBufferDesc::CreateStructuredDesc(sizeof(uint32), NumHighResParticleCount);
                HighResVtxTriOffsetsPooledBuffer = AllocatePooledBuffer(OffDesc, TEXT("HighResVtxTriOffsets"));
                void* OffData = RHICmdList.LockBuffer(HighResVtxTriOffsetsPooledBuffer->GetRHI(), 0, NumHighResParticleCount * sizeof(uint32), RLM_WriteOnly);
                FMemory::Memcpy(OffData, LocalHighResVtxTriOffsets.GetData(), NumHighResParticleCount * sizeof(uint32));
                RHICmdList.UnlockBuffer(HighResVtxTriOffsetsPooledBuffer->GetRHI());

                // 拓扑 Counts
                HighResVtxTriCountsPooledBuffer = AllocatePooledBuffer(OffDesc, TEXT("HighResVtxTriCounts"));
                void* CountData = RHICmdList.LockBuffer(HighResVtxTriCountsPooledBuffer->GetRHI(), 0, NumHighResParticleCount * sizeof(uint32), RLM_WriteOnly);
                FMemory::Memcpy(CountData, LocalHighResVtxTriCounts.GetData(), NumHighResParticleCount * sizeof(uint32));
                RHICmdList.UnlockBuffer(HighResVtxTriCountsPooledBuffer->GetRHI());
            }

            if (NumHighResVtxTriIndices > 0)
            {
                // 拓扑 Indices
                FRDGBufferDesc IdxDesc = FRDGBufferDesc::CreateStructuredDesc(sizeof(uint32), NumHighResVtxTriIndices);
                HighResVtxTriIndicesPooledBuffer = AllocatePooledBuffer(IdxDesc, TEXT("HighResVtxTriIndices"));
                void* IdxData = RHICmdList.LockBuffer(HighResVtxTriIndicesPooledBuffer->GetRHI(), 0, NumHighResVtxTriIndices * sizeof(uint32), RLM_WriteOnly);
                FMemory::Memcpy(IdxData, LocalHighResVtxTriIndices.GetData(), NumHighResVtxTriIndices * sizeof(uint32));
                RHICmdList.UnlockBuffer(HighResVtxTriIndicesPooledBuffer->GetRHI());
            }

            // 加权法线临时 Buffer 和 输出 Buffer (保持不变)
            if (NumHighResParticleCount > 0)
            {
                FRDGBufferDesc NormDesc = FRDGBufferDesc::CreateStructuredDesc(sizeof(FVector3f), NumHighResParticleCount);
                NormDesc.Usage |= BUF_UnorderedAccess | BUF_SourceCopy;
                HighResWeightedNormalsPooledBuffer = AllocatePooledBuffer(NormDesc, TEXT("HighResWeightedNormals"));
                HighResNormalOutputPooledBuffer = AllocatePooledBuffer(NormDesc, TEXT("HighResNormalOutput"));
            }
        }
    );

    FlushRenderingCommands();
    bGPUResourcesInitialized = true;
}

void UMySoftBodyMeshComponent::ReleaseGPUResources()
{
    HighResParticlePooledBuffer.SafeRelease();
    HighResPositionOutputPooledBuffer.SafeRelease();
    
    ProxyParticlePooledBuffer.SafeRelease();
    ProxyPositionOutputPooledBuffer.SafeRelease();
    
    ConstraintPooledBuffer.SafeRelease();
    LambdaPooledBuffer.SafeRelease();
    NeighborInfoPooledBuffer.SafeRelease();
    TrianglePooledBuffer.SafeRelease();
    WeightedNormalsPooledBuffer.SafeRelease();
    VolumePooledBuffer.SafeRelease();
    NormalOutputPooledBuffer.SafeRelease();
    VertexTriangleOffsetsPooledBuffer.SafeRelease();
    VertexTriangleCountsPooledBuffer.SafeRelease();
    VertexTriangleIndicesPooledBuffer.SafeRelease();
    VolumeOutputPooledBuffer.SafeRelease();
    ScaffoldBindingsPooledBuffer.SafeRelease();
    ProxyTrianglePooledBuffer.SafeRelease();
    if (NormalReadback)
    {
        delete NormalReadback;
        NormalReadback = nullptr;
    }

    if (ParticleReadback)
    {
        delete ParticleReadback;
        ParticleReadback = nullptr;
    }
    
    if (VolumeReadback)
    {
        delete VolumeReadback;
        VolumeReadback = nullptr;
    }
    
    if (ProxyReadback) 
    {
        delete ProxyReadback;
        ProxyReadback = nullptr;
    }
    
    
    bGPUResourcesInitialized = false;
    bReadbackPending = false;
    bVolumeReadbackPending = false;
}

void UMySoftBodyMeshComponent::UploadDataToGPU()
{
    // 每帧更新棍子碰撞参数
    // 这里暂时留空，棍子参数通过 Shader 参数传递
}

void UMySoftBodyMeshComponent::DispatchGPUCompute(bool bIsLastSubstep)
{
    if (!bGPUResourcesInitialized || !ProxyParticlePooledBuffer.IsValid()) return;

    int32 NumParticles = ProxyParticleCount;
    int32 NumConstraints = Constraints.Num();
    int32 NumTriangles = ProxyTriCount;
    int32 NumProxyParticles = ProxyParticleCount;
    int32 NumHighResParticles = HighResParticleCount;
    bool bUseVolume = bUse_VolumePressureForce;
    float LocalRestVolume = restVolume;
    float LocalVolCoeff = VolPressure_Coefficient;
    float SubstepDt = St;
    int32 Iterations = ConstraintIterations;
    float OmegaVal = 0.5f; // 松弛因子
    float LocalHighResStiffness = HighResStiffness;
    float LocalHighResDamping = HighResDamping;
    
    // ==========================================
    // 提取 VAT 纹理资源
    // ==========================================
    bool bLocalUpdateViaGPU = bUpdateViaGPU;
    FTextureRenderTargetResource* PosRTResource = PositionRT ? PositionRT->GameThread_GetRenderTargetResource() : nullptr;
    FTextureRenderTargetResource* NormRTResource = NormalRT ? NormalRT->GameThread_GetRenderTargetResource() : nullptr;
    int32 LocalTexDimension = PositionRT ? PositionRT->SizeX : 0;
    
    // 棍子碰撞参数
    FVector3f StickS = FVector3f(CachedStickStart);
    FVector3f StickE = FVector3f(CachedStickEnd);
    // 速度参数
    FVector3f StickVelS = FVector3f(CachedStickVelStart);
    FVector3f StickVelE = FVector3f(CachedStickVelEnd);
    float StickR = CachedStickRadius;
    float PartR = ParticleRadius;
    int32 HasStick = bHasStickInput ? 1 : 0;

    float LocalFriction = CollisionFriction;
    float LocalRestitution = CollisionRestitution;
    float LocalGDFSkinOffset = GDFSkinOffset;

    TRefCountPtr<FRDGPooledBuffer> ProxyParticleBuf = ProxyParticlePooledBuffer;
    TRefCountPtr<FRDGPooledBuffer> HighResRestPosPooledBuf = HighResRestPosPooledBuffer;
    TRefCountPtr<FRDGPooledBuffer> ConstraintBuf = ConstraintPooledBuffer;
    TRefCountPtr<FRDGPooledBuffer> LambdaBuf = LambdaPooledBuffer;
    TRefCountPtr<FRDGPooledBuffer> SurfaceNeighborBuf = SurfaceNeighborInfoPooledBuffer;
    TRefCountPtr<FRDGPooledBuffer> SurfaceAdjBuf = SurfaceAdjacencyPooledBuffer;
    TRefCountPtr<FRDGPooledBuffer> InternalNeighborBuf = InternalNeighborInfoPooledBuffer;
    TRefCountPtr<FRDGPooledBuffer> InternalAdjBuf = InternalAdjacencyPooledBuffer;
    TRefCountPtr<FRDGPooledBuffer> PositionOutputBuf = ProxyPositionOutputPooledBuffer;
    TRefCountPtr<FRDGPooledBuffer> TriangleBuf = TrianglePooledBuffer;
    TRefCountPtr<FRDGPooledBuffer> WeightedNormalsBuf = WeightedNormalsPooledBuffer;
    TRefCountPtr<FRDGPooledBuffer> VolumeBuf = VolumePooledBuffer;
    TRefCountPtr<FRDGPooledBuffer> NormalOutputBuf = NormalOutputPooledBuffer;
    TRefCountPtr<FRDGPooledBuffer> VertexTriangleOffsetsBuf=VertexTriangleOffsetsPooledBuffer;
    TRefCountPtr<FRDGPooledBuffer> VertexTriangleCountsBuf=VertexTriangleCountsPooledBuffer;
    TRefCountPtr<FRDGPooledBuffer> VertexTriangleIndicesBuf=VertexTriangleIndicesPooledBuffer;
    TRefCountPtr<FRDGPooledBuffer> VolumeOutputBuf = VolumeOutputPooledBuffer;
    TRefCountPtr<FRDGPooledBuffer> HighResParticleBuf = HighResParticlePooledBuffer;
    TRefCountPtr<FRDGPooledBuffer> HighResPositionOutputBuf = HighResPositionOutputPooledBuffer;
    TRefCountPtr<FRDGPooledBuffer> HighResTriBuf = HighResTrianglePooledBuffer;
    TRefCountPtr<FRDGPooledBuffer> HighResWeightedNormalsBuf = HighResWeightedNormalsPooledBuffer;
    TRefCountPtr<FRDGPooledBuffer> HighResNormalOutputBuf = HighResNormalOutputPooledBuffer;
    TRefCountPtr<FRDGPooledBuffer> HighResVtxTriOffsetsBuf = HighResVtxTriOffsetsPooledBuffer;
    TRefCountPtr<FRDGPooledBuffer> HighResVtxTriCountsBuf = HighResVtxTriCountsPooledBuffer;
    TRefCountPtr<FRDGPooledBuffer> HighResVtxTriIndicesBuf = HighResVtxTriIndicesPooledBuffer;
    TRefCountPtr<FRDGPooledBuffer> ScaffoldBindingsBuf = ScaffoldBindingsPooledBuffer;
    TRefCountPtr<FRDGPooledBuffer> ProxyTriangleBuf = ProxyTrianglePooledBuffer;
    
    TRefCountPtr<FRDGPooledBuffer> D_ConBuf = DihedralConstraintPooledBuffer;
    TRefCountPtr<FRDGPooledBuffer> D_LamBuf = DihedralLambdaPooledBuffer;
    TRefCountPtr<FRDGPooledBuffer> D_InfoBuf = DihedralNeighborInfoPooledBuffer;
    TRefCountPtr<FRDGPooledBuffer> D_AdjBuf = DihedralAdjacencyPooledBuffer;
    int32 NumDihedrals = bUse_DihedralBending ? DihedralConstraints.Num() : 0;

    // =========================================================
    // 收集动态碰撞体数据 (球/盒/胶囊) —— 每帧在游戏线程收集
    // =========================================================
    TArray<FGPUCollider> ColliderData;
    if (bUseDynamicColliders)
    {
        ColliderData.Reserve(DynamicColliders.Num());
        for (const TWeakObjectPtr<USoftBodyColliderComponent>& WeakCol : DynamicColliders)
        {
            USoftBodyColliderComponent* Col = WeakCol.Get();
            if (!Col || !Col->bEnabled) continue;

            FSoftBodyColliderPrimitive Prim;
            Col->GetWorldCollider(Prim);

            FGPUCollider G;
            G.Shape = static_cast<int32>(Prim.Shape);
            G.Radius = Prim.Radius;
            G.CapsuleHalfHeight = Prim.CapsuleHalfHeight;
            G.Center = FVector3f(Prim.Center);
            G.AxisX = FVector3f(Prim.Rotation.GetAxisX());
            G.AxisY = FVector3f(Prim.Rotation.GetAxisY());
            G.AxisZ = FVector3f(Prim.Rotation.GetAxisZ());
            G.HalfExtents = FVector3f(Prim.HalfExtents);
            G.Velocity = FVector3f(Prim.Velocity);
            G.Friction = Prim.Friction;
            G.Restitution = Prim.Restitution;
            ColliderData.Add(G);
        }
    }

    ENQUEUE_RENDER_COMMAND(DispatchSoftBodyXPBD)(
        [=, this](FRHICommandListImmediate& RHICmdList)
        {
            SCOPED_DRAW_EVENT(RHICmdList, SoftBody_Simulation_Total);
            SCOPED_NAMED_EVENT(SoftBody_Simulation_Total, FColor::Orange);
            FRDGBuilder GraphBuilder(RHICmdList);

            // 注册 Buffer
            FRDGBufferRef ProxyParticleRDG = GraphBuilder.RegisterExternalBuffer(ProxyParticleBuf);
            FRDGBufferRef ConstraintRDG = ConstraintBuf.IsValid() ? GraphBuilder.RegisterExternalBuffer(ConstraintBuf) : nullptr;
            FRDGBufferRef LambdaRDG = LambdaBuf.IsValid() ? GraphBuilder.RegisterExternalBuffer(LambdaBuf) : nullptr;
            FRDGBufferRef SurfaceNeighborRDG = SurfaceNeighborBuf.IsValid() ? GraphBuilder.RegisterExternalBuffer(SurfaceNeighborBuf) : nullptr;
            FRDGBufferRef SurfaceAdjRDG = SurfaceAdjBuf.IsValid() ? GraphBuilder.RegisterExternalBuffer(SurfaceAdjBuf) : nullptr;
            FRDGBufferRef InternalNeighborRDG = InternalNeighborBuf.IsValid() ? GraphBuilder.RegisterExternalBuffer(InternalNeighborBuf) : nullptr;
            FRDGBufferRef InternalAdjRDG = InternalAdjBuf.IsValid() ? GraphBuilder.RegisterExternalBuffer(InternalAdjBuf) : nullptr;
            FRDGBufferRef VtxTriOffsetsRDG = VertexTriangleOffsetsBuf.IsValid() ? GraphBuilder.RegisterExternalBuffer(VertexTriangleOffsetsBuf) : nullptr;
            FRDGBufferRef VtxTriCountsRDG = VertexTriangleCountsBuf.IsValid() ? GraphBuilder.RegisterExternalBuffer(VertexTriangleCountsBuf) : nullptr;
            FRDGBufferRef VtxTriIndicesRDG = VertexTriangleIndicesBuf.IsValid() ? GraphBuilder.RegisterExternalBuffer(VertexTriangleIndicesBuf) : nullptr;
      

            auto ProxyParticleUAV = GraphBuilder.CreateUAV(ProxyParticleRDG);
            auto ProxyParticleSRV = GraphBuilder.CreateSRV(ProxyParticleRDG);
            auto ConstraintSRV = ConstraintRDG ? GraphBuilder.CreateSRV(ConstraintRDG) : nullptr;
            auto LambdaUAV = LambdaRDG ? GraphBuilder.CreateUAV(LambdaRDG) : nullptr;
            auto LambdaSRV = LambdaRDG ? GraphBuilder.CreateSRV(LambdaRDG) : nullptr;
            auto SurfaceNeighborSRV = SurfaceNeighborRDG ? GraphBuilder.CreateSRV(SurfaceNeighborRDG) : nullptr;
            auto SurfaceAdjSRV = SurfaceAdjRDG ? GraphBuilder.CreateSRV(SurfaceAdjRDG) : nullptr;
            auto InternalNeighborSRV = InternalNeighborRDG ? GraphBuilder.CreateSRV(InternalNeighborRDG) : nullptr;
            auto InternalAdjSRV = InternalAdjRDG ? GraphBuilder.CreateSRV(InternalAdjRDG) : nullptr;
            auto VtxTriOffsetsSRV = GraphBuilder.CreateSRV(VtxTriOffsetsRDG);
            auto VtxTriCountsSRV = GraphBuilder.CreateSRV(VtxTriCountsRDG);
            auto VtxTriIndicesSRV = GraphBuilder.CreateSRV(VtxTriIndicesRDG);
            
            
            uint32 ParticleGroups = FMath::DivideAndRoundUp((uint32)NumParticles, 64u);
            uint32 ConstraintGroups = FMath::DivideAndRoundUp((uint32)NumConstraints, 256u);
            
            // ========== 二面角约束注册 ==========
            FRDGBufferRef D_ConRDG = D_ConBuf.IsValid() ? GraphBuilder.RegisterExternalBuffer(D_ConBuf) : nullptr;
            FRDGBufferRef D_LamRDG = D_LamBuf.IsValid() ? GraphBuilder.RegisterExternalBuffer(D_LamBuf) : nullptr;
            FRDGBufferRef D_InfoRDG = D_InfoBuf.IsValid() ? GraphBuilder.RegisterExternalBuffer(D_InfoBuf) : nullptr;
            FRDGBufferRef D_AdjRDG = D_AdjBuf.IsValid() ? GraphBuilder.RegisterExternalBuffer(D_AdjBuf) : nullptr;
            
            auto D_ConSRV = D_ConRDG ? GraphBuilder.CreateSRV(D_ConRDG) : nullptr;
            auto D_LamSRV = D_LamRDG ? GraphBuilder.CreateSRV(D_LamRDG) : nullptr;
            auto D_LamUAV = D_LamRDG ? GraphBuilder.CreateUAV(D_LamRDG) : nullptr;
            auto D_InfoSRV = D_InfoRDG ? GraphBuilder.CreateSRV(D_InfoRDG) : nullptr;
            auto D_AdjSRV = D_AdjRDG ? GraphBuilder.CreateSRV(D_AdjRDG) : nullptr;
            // ========== 体积约束注册 ==========
            FRDGBufferRef TriangleRDG = GraphBuilder.RegisterExternalBuffer(TriangleBuf);
            FRDGBufferRef WeightedNormalsRDG = GraphBuilder.RegisterExternalBuffer(WeightedNormalsBuf);
            FRDGBufferRef VolumeRDG = GraphBuilder.RegisterExternalBuffer(VolumeBuf);
            FRDGBufferRef VolumeOutputRDG = GraphBuilder.RegisterExternalBuffer(VolumeOutputBuf);
                    
            auto TriangleSRV = GraphBuilder.CreateSRV(TriangleRDG);
            auto WeightedNormalsUAV = GraphBuilder.CreateUAV(WeightedNormalsRDG);
            auto WeightedNormalsSRV = GraphBuilder.CreateSRV(WeightedNormalsRDG);
            auto VolumeUAV = GraphBuilder.CreateUAV(VolumeRDG);
            auto VolumeSRV = GraphBuilder.CreateSRV(VolumeRDG);
            auto VolumeOutputUAV = GraphBuilder.CreateUAV(VolumeOutputRDG);
            
            // 1. 清零 Lambda
            if (LambdaUAV && NumConstraints > 0)
            {
                TShaderMapRef<FClearLambdasCS> Shader(GetGlobalShaderMap(GMaxRHIFeatureLevel));
                auto* Params = GraphBuilder.AllocParameters<FClearLambdasCS::FParameters>();
                Params->LambdaBuffer = LambdaUAV;
                Params->ConstraintCount = NumConstraints;
                FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("ClearLambdas"), Shader, Params, FIntVector(ConstraintGroups, 1, 1));
            }
            // 清零二面角 Lambda
            if (bUse_DihedralBending && D_LamUAV && NumDihedrals > 0)
            {
                TShaderMapRef<FClearLambdasCS> Shader(GetGlobalShaderMap(GMaxRHIFeatureLevel));
                auto* Params = GraphBuilder.AllocParameters<FClearLambdasCS::FParameters>();
                Params->LambdaBuffer = D_LamUAV; // 传入二面角的 Lambda Buffer
                Params->ConstraintCount = NumDihedrals;
                
                uint32 D_ConstraintGroups = FMath::DivideAndRoundUp((uint32)NumDihedrals, 256u);
                FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("ClearDihedralLambdas"), Shader, Params, FIntVector(D_ConstraintGroups, 1, 1));
            }
            // 2. 积分
            {
                TShaderMapRef<FIntegrateCS> Shader(GetGlobalShaderMap(GMaxRHIFeatureLevel));
                auto* Params = GraphBuilder.AllocParameters<FIntegrateCS::FParameters>();
                Params->RWParticles = ProxyParticleUAV;
                Params->SubstepTime = SubstepDt;
                Params->ParticleCount = NumParticles;
                FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("Integrate"), Shader, Params, FIntVector(ParticleGroups, 1, 1));
            }
            FRDGBufferRef DummyTargetRDG = GraphBuilder.RegisterExternalBuffer(PositionOutputBuf);
            auto DummyTargetSRV = GraphBuilder.CreateSRV(DummyTargetRDG);
            // 3. 约束迭代
            for (int32 iter = 0; iter < Iterations; ++iter)
            {
                // ==========================================
                // 3b (通道 2)：求解表面约束 (微观皮肤平滑收尾)
                // UE RDG 会自动在这里插入 GPU 内存屏障，
                // 保证执行这一步时，内部骨架已经把顶点推到了正确的大致位置！
                // ==========================================
                if (ConstraintSRV && LambdaSRV && SurfaceNeighborSRV && SurfaceAdjSRV)
                {
                    TShaderMapRef<FSolveAndApplyCS> Shader(GetGlobalShaderMap(GMaxRHIFeatureLevel));
                    auto* Params = GraphBuilder.AllocParameters<FSolveAndApplyCS::FParameters>();
                    Params->RWParticles = ProxyParticleUAV;
                    Params->Constraints = ConstraintSRV;
                    Params->LambdaBuffer = LambdaSRV;
                    Params->ParticleConInfos = SurfaceNeighborSRV; // 传入表面的配置
                    Params->AdjacencyIndices = SurfaceAdjSRV;      // 传入表面的邻接表
                    Params->SubstepTime = SubstepDt;
                    Params->ParticleCount = NumParticles;
                    Params->Omega = OmegaVal;
                    Params->TargetPositions = DummyTargetSRV;
                    Params->AttachmentStiffness = 0.0f;
                    Params->UseTargetPosition = 0;
                    
                    FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("SolveSurface"), Shader, Params, FIntVector(ParticleGroups, 1, 1));
                }
                // ==========================================
                // 3a (通道 1)：求解内部支撑 (宏观体积骨架先行)
                // ==========================================
                if (bUse_InternalConstraints && ConstraintSRV && LambdaSRV && InternalNeighborSRV && InternalAdjSRV)
                {
                    TShaderMapRef<FSolveAndApplyCS> Shader(GetGlobalShaderMap(GMaxRHIFeatureLevel));
                    auto* Params = GraphBuilder.AllocParameters<FSolveAndApplyCS::FParameters>();
                    Params->RWParticles = ProxyParticleUAV;
                    Params->Constraints = ConstraintSRV;
                    Params->LambdaBuffer = LambdaSRV;
                    Params->ParticleConInfos = InternalNeighborSRV; // 传入内部支撑的配置
                    Params->AdjacencyIndices = InternalAdjSRV;      // 传入内部支撑的邻接表
                    Params->SubstepTime = SubstepDt;
                    Params->ParticleCount = NumParticles;
                    Params->Omega = OmegaVal;
                    Params->TargetPositions = DummyTargetSRV;
                    Params->AttachmentStiffness = 0.0f;
                    Params->UseTargetPosition = 0;
                    
                    FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("SolveInternal"), Shader, Params, FIntVector(ParticleGroups, 1, 1));
                }
                // ==========================================
                // 求解二面角弯曲约束 (Jacobi 迭代)
                // ==========================================
                if (bUse_DihedralBending && D_ConSRV && D_LamSRV && D_InfoSRV && D_AdjSRV)
                {
                    TShaderMapRef<FSolveDihedralCS> Shader(GetGlobalShaderMap(GMaxRHIFeatureLevel));
                    auto* Params = GraphBuilder.AllocParameters<FSolveDihedralCS::FParameters>();
                    
                    Params->RWParticles = ProxyParticleUAV;
                    Params->DihedralConstraints = D_ConSRV;
                    Params->DihedralLambdaBuffer = D_LamSRV;
                    Params->DihedralConInfos = D_InfoSRV;
                    Params->DihedralAdjacencyIndices = D_AdjSRV;
                    Params->SubstepTime = SubstepDt;
                    Params->ParticleCount = NumParticles;
                    Params->Omega = OmegaVal;
                    
                    FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("SolveDihedral"), Shader, Params, FIntVector(ParticleGroups, 1, 1));
                }
                // ==========================================
                // 更新二面角 Lambda
                // ==========================================
                if (bUse_DihedralBending && D_LamUAV && D_ConSRV && NumDihedrals > 0)
                {
                    TShaderMapRef<FUpdateDihedralLambdasCS> Shader(GetGlobalShaderMap(GMaxRHIFeatureLevel));
                    auto* Params = GraphBuilder.AllocParameters<FUpdateDihedralLambdasCS::FParameters>();
                    
                    Params->RWParticles = ProxyParticleSRV;
                    Params->DihedralConstraints = D_ConSRV;
                    Params->DihedralLambdaBuffer = D_LamUAV;
                    Params->SubstepTime = SubstepDt;
                    Params->ConstraintCount = NumDihedrals;

                    uint32 D_ConstraintGroups = FMath::DivideAndRoundUp((uint32)NumDihedrals, 256u);
                    FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("UpdateDihedralLambdas"), Shader, Params, FIntVector(D_ConstraintGroups, 1, 1));
                }
                // 3b. 更新 Lambda
                if (LambdaUAV && ConstraintSRV && NumConstraints > 0)
                {
                    TShaderMapRef<FUpdateLambdasCS> Shader(GetGlobalShaderMap(GMaxRHIFeatureLevel));
                    auto* Params = GraphBuilder.AllocParameters<FUpdateLambdasCS::FParameters>();
                    Params->RWParticles = ProxyParticleSRV;
                    Params->Constraints = ConstraintSRV;
                    Params->LambdaBuffer = LambdaUAV;
                    Params->SubstepTime = SubstepDt;
                    Params->ConstraintCount = NumConstraints;
                    FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("UpdateLambdas"), Shader, Params, FIntVector(ConstraintGroups, 1, 1));
                }
                // ========== 体积约束 ==========
                if (bUseVolume && TriangleBuf.IsValid())
                {
                    // 清零法线
                    {
                        TShaderMapRef<FClearNormalsCS> Shader(GetGlobalShaderMap(GMaxRHIFeatureLevel));
                        auto* Params = GraphBuilder.AllocParameters<FClearNormalsCS::FParameters>();
                        Params->WeightedNormals = WeightedNormalsUAV;
                        Params->ParticleCount = NumParticles;
                        FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("ClearNormals"), Shader, Params, FIntVector(ParticleGroups, 1, 1));
                    }
                    // 计算加权法线
                    {
                        TShaderMapRef<FComputeWeightedNormalsCS> Shader(GetGlobalShaderMap(GMaxRHIFeatureLevel));
                        auto* Params = GraphBuilder.AllocParameters<FComputeWeightedNormalsCS::FParameters>();
                        Params->Particles = ProxyParticleSRV;
                        Params->Triangles = TriangleSRV;
                        Params->WeightedNormals = WeightedNormalsUAV;
                        Params->VertexTriangleOffsets = VtxTriOffsetsSRV;  
                        Params->VertexTriangleCounts = VtxTriCountsSRV;    
                        Params->VertexTriangleIndices = VtxTriIndicesSRV;  
                        Params->ParticleCount = NumParticles;
                        FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("ComputeNormals"), Shader, Params, FIntVector(ParticleGroups, 1, 1));
                    }
                    // 清零体积
                    {
                        TShaderMapRef<FClearVolumeCS> Shader(GetGlobalShaderMap(GMaxRHIFeatureLevel));
                        auto* Params = GraphBuilder.AllocParameters<FClearVolumeCS::FParameters>();
                        Params->VolumeInt = VolumeUAV;
                        FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("ClearVolume"), Shader, Params, FIntVector(1, 1, 1));
                    }
                    // 计算体积
                    {
                        TShaderMapRef<FComputeVolumeCS> Shader(GetGlobalShaderMap(GMaxRHIFeatureLevel));
                        auto* Params = GraphBuilder.AllocParameters<FComputeVolumeCS::FParameters>();
                        Params->Particles = ProxyParticleSRV;
                        Params->WeightedNormalsRead = WeightedNormalsSRV;
                        Params->VolumeInt = VolumeUAV;
                        Params->ParticleCount = NumParticles;
                        FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("ComputeVolume"), Shader, Params, FIntVector(ParticleGroups, 1, 1));
                    }
                
                    // 应用压力
                    {
                        TShaderMapRef<FApplyVolumePressureCS> Shader(GetGlobalShaderMap(GMaxRHIFeatureLevel));
                        auto* Params = GraphBuilder.AllocParameters<FApplyVolumePressureCS::FParameters>();
                        Params->RWParticles = ProxyParticleUAV;
                        Params->WeightedNormalsRead = WeightedNormalsSRV;
                        Params->VolumeIntRead = VolumeSRV;
                        Params->ParticleCount = NumParticles;
                        Params->RestVolume = LocalRestVolume;
                        Params->VolumePressureCoeff = LocalVolCoeff;
                        Params->SubstepTime = SubstepDt;
                        FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("ApplyVolumePressure"), Shader, Params, FIntVector(ParticleGroups, 1, 1));
                    }
                    // 回读体积（调试用）
                    {
                        TShaderMapRef<FExportVolumeCS> Shader(GetGlobalShaderMap(GMaxRHIFeatureLevel));
                        auto* Params = GraphBuilder.AllocParameters<FExportVolumeCS::FParameters>();
                        Params->VolumeIntRead = VolumeSRV;
                        Params->OutputVolume = VolumeOutputUAV;
                        FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("ExportVolume"), Shader, Params, FIntVector(1, 1, 1));
                    }
                    //回读法线
                    {
                        FRDGBufferRef NormalOutputRDG = GraphBuilder.RegisterExternalBuffer(NormalOutputBuf);
                        auto NormalOutputUAV = GraphBuilder.CreateUAV(NormalOutputRDG);

                        TShaderMapRef<FExportNormalsCS> Shader(GetGlobalShaderMap(GMaxRHIFeatureLevel));
                        auto* Params = GraphBuilder.AllocParameters<FExportNormalsCS::FParameters>();
                        Params->WeightedNormalsRead = WeightedNormalsSRV;
                        Params->OutputNormals = NormalOutputUAV;
                        Params->ParticleCount = NumParticles;
                        FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("ExportNormals"), Shader, Params, FIntVector(ParticleGroups, 1, 1));
                    }
                }
                // 3c. 棍子碰撞
                if (HasStick)
                {
                    TShaderMapRef<FCollideStickCS> Shader(GetGlobalShaderMap(GMaxRHIFeatureLevel));
                    auto* Params = GraphBuilder.AllocParameters<FCollideStickCS::FParameters>();

                    Params->RWParticles = ProxyParticleUAV;
                    Params->ParticleCount = NumParticles;
                    Params->SubstepTime = SubstepDt;
                    Params->StickStart = StickS;
                    Params->StickEnd = StickE;
                    Params->StickRadius = StickR;
                    Params->ParticleRadius = PartR;
                    Params->HasStickInput = HasStick;
                    Params->StickVelStart = StickVelS;
                    Params->StickVelEnd = StickVelE;
                    Params->StickFriction = LocalFriction;
                    Params->StickRestitution = LocalRestitution;

                    FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("CollideStick"), Shader, Params, FIntVector(ParticleGroups, 1, 1));
                }
            }

            // ========== 全局距离场碰撞 (从缓存读稳定纹理) ==========
            AddGDFCollisionPass(GraphBuilder);

            // ========== 动态碰撞体碰撞 (球/盒/胶囊) ==========
            if (ColliderData.Num() > 0)
            {
                FRDGBufferRef ColliderRDG = CreateStructuredBuffer(GraphBuilder, TEXT("SoftBodyColliders"), ColliderData);
                auto ColliderSRV = GraphBuilder.CreateSRV(ColliderRDG);

                TShaderMapRef<FCollidePrimitivesCS> Shader(GetGlobalShaderMap(GMaxRHIFeatureLevel));
                auto* Params = GraphBuilder.AllocParameters<FCollidePrimitivesCS::FParameters>();
                Params->RWParticles = ProxyParticleUAV;
                Params->Colliders = ColliderSRV;
                Params->ColliderCount = (uint32)ColliderData.Num();
                Params->ParticleCount = (uint32)NumParticles;
                Params->ParticleRadius = PartR;
                Params->SubstepTime = SubstepDt;
                FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("CollidePrimitives"), Shader, Params, FIntVector(ParticleGroups, 1, 1));
            }

            // 4. 更新速度
            {
                TShaderMapRef<FUpdateVelocityCS> Shader(GetGlobalShaderMap(GMaxRHIFeatureLevel));
                auto* Params = GraphBuilder.AllocParameters<FUpdateVelocityCS::FParameters>();
                Params->RWParticles = ProxyParticleUAV;
                Params->SubstepTime = SubstepDt;
                Params->ParticleCount = NumParticles;
                FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("UpdateVelocity"), Shader, Params, FIntVector(ParticleGroups, 1, 1));
            }
            // 5. 导出位置 (用于 Readback)
            if ((bShowProxyNodes || bShowSingleVertexConstraints) && ProxyPositionOutputPooledBuffer.IsValid())
            {
                FRDGBufferRef PositionOutputRDG = GraphBuilder.RegisterExternalBuffer(PositionOutputBuf);
                auto PositionOutputUAV = GraphBuilder.CreateUAV(PositionOutputRDG);

                TShaderMapRef<FExportPositionsCS> Shader(GetGlobalShaderMap(GMaxRHIFeatureLevel));
                auto* Params = GraphBuilder.AllocParameters<FExportPositionsCS::FParameters>();
                Params->ReadParticles = ProxyParticleSRV;
                Params->OutputPositions = PositionOutputUAV;
                Params->ParticleCount = NumParticles;
                FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("ExportPositions"), Shader, Params, FIntVector(ParticleGroups, 1, 1));
            }
            // =============================================================
            // 阶段 2：数据同步 (Proxy -> HighRes)
            // =============================================================
            if (bUseProxy && HighResParticleBuf.IsValid() && ScaffoldBindingsBuf.IsValid() && ProxyTriangleBuf.IsValid())
            {
                // 1. 注册高模和映射表
                FRDGBufferRef HighResParticleRDG = GraphBuilder.RegisterExternalBuffer(HighResParticleBuf);
                auto HighResParticleUAV = GraphBuilder.CreateUAV(HighResParticleRDG);
                
                FRDGBufferRef BindingsRDG = GraphBuilder.RegisterExternalBuffer(ScaffoldBindingsBuf);
                auto BindingsSRV = GraphBuilder.CreateSRV(BindingsRDG);

                FRDGBufferRef ProxyTriRDG = GraphBuilder.RegisterExternalBuffer(ProxyTriangleBuf);
                auto ProxyTriSRV = GraphBuilder.CreateSRV(ProxyTriRDG);
                
                // =============================================================
                // 2. 派发重心坐标蒙皮 Shader
                // =============================================================
                TShaderMapRef<FUpdateHighResTargetCS> UpdateHighResShader(GetGlobalShaderMap(GMaxRHIFeatureLevel));
                auto* Params = GraphBuilder.AllocParameters<FUpdateHighResTargetCS::FParameters>();
                
                // 【核心修改】直接将高模的粒子 Buffer 传给 Shader，让插值结果瞬间变成真实位置！
                Params->RWTargetPositions = HighResParticleUAV;                             
                Params->ReadProxyParticles = GraphBuilder.CreateSRV(ProxyParticleRDG);
                Params->ScaffoldBindings = BindingsSRV;                               
                Params->ProxyTriangles = ProxyTriSRV;                                 
                Params->HighResCount = NumHighResParticles;
                Params->SubstepTime = SubstepDt;
                Params->HighResStiffness = LocalHighResStiffness;
                Params->HighResDamping = LocalHighResDamping;

                uint32 HighResGroups = FMath::DivideAndRoundUp((uint32)NumHighResParticles, 64u);
                FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("UpdateHighResTarget"), UpdateHighResShader, Params, FIntVector(HighResGroups, 1, 1));
                
                // =============================================================
                // 步骤 C：计算并导出高模平滑法线 (渲染必备，原封不动保留)
                // =============================================================
                if (bIsLastSubstep && HighResTriBuf.IsValid() && HighResVtxTriOffsetsBuf.IsValid())
                {
                    FRDGBufferRef HighResTriRDG = GraphBuilder.RegisterExternalBuffer(HighResTriBuf);
                    FRDGBufferRef HighResWeightedNormalsRDG = GraphBuilder.RegisterExternalBuffer(HighResWeightedNormalsBuf);
                    FRDGBufferRef HighResNormalOutRDG = GraphBuilder.RegisterExternalBuffer(HighResNormalOutputBuf);
                    FRDGBufferRef HighResOffsetsRDG = GraphBuilder.RegisterExternalBuffer(HighResVtxTriOffsetsBuf);
                    FRDGBufferRef HighResCountsRDG = GraphBuilder.RegisterExternalBuffer(HighResVtxTriCountsBuf);
                    FRDGBufferRef HighResIndicesRDG = GraphBuilder.RegisterExternalBuffer(HighResVtxTriIndicesBuf);
                        
                    auto HighResTriSRV = GraphBuilder.CreateSRV(HighResTriRDG);
                    auto HighResWeightedNormalsUAV = GraphBuilder.CreateUAV(HighResWeightedNormalsRDG);
                    auto HighResWeightedNormalsSRV = GraphBuilder.CreateSRV(HighResWeightedNormalsRDG);
                    auto HighResNormalOutUAV = GraphBuilder.CreateUAV(HighResNormalOutRDG);
                    auto OffsetsSRV = GraphBuilder.CreateSRV(HighResOffsetsRDG);
                    auto CountsSRV = GraphBuilder.CreateSRV(HighResCountsRDG);
                    auto IndicesSRV = GraphBuilder.CreateSRV(HighResIndicesRDG);

                    // 清零法线
                    TShaderMapRef<FClearNormalsCS> ClearNormalsShader(GetGlobalShaderMap(GMaxRHIFeatureLevel));
                    auto* ClearNormalsParams = GraphBuilder.AllocParameters<FClearNormalsCS::FParameters>();
                    ClearNormalsParams->WeightedNormals = HighResWeightedNormalsUAV;
                    ClearNormalsParams->ParticleCount = NumHighResParticles;
                    FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("ClearHighResNormals"), ClearNormalsShader, ClearNormalsParams, FIntVector(HighResGroups, 1, 1));

                    // 累加加权法线
                    TShaderMapRef<FComputeWeightedNormalsCS> ComputeNormalsShader(GetGlobalShaderMap(GMaxRHIFeatureLevel));
                    auto* ComputeNormalsParams = GraphBuilder.AllocParameters<FComputeWeightedNormalsCS::FParameters>();
                    ComputeNormalsParams->Particles = GraphBuilder.CreateSRV(HighResParticleRDG); // 读最新的高模位置
                    ComputeNormalsParams->Triangles = HighResTriSRV;
                    ComputeNormalsParams->WeightedNormals = HighResWeightedNormalsUAV;
                    ComputeNormalsParams->VertexTriangleOffsets = OffsetsSRV;
                    ComputeNormalsParams->VertexTriangleCounts = CountsSRV;
                    ComputeNormalsParams->VertexTriangleIndices = IndicesSRV;
                    ComputeNormalsParams->ParticleCount = NumHighResParticles;
                    FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("ComputeHighResNormals"), ComputeNormalsShader, ComputeNormalsParams, FIntVector(HighResGroups, 1, 1));

                    // 归一化并导出用于 Readback
                    TShaderMapRef<FExportNormalsCS> ExportNormalsShader(GetGlobalShaderMap(GMaxRHIFeatureLevel));
                    auto* ExportNormalsParams = GraphBuilder.AllocParameters<FExportNormalsCS::FParameters>();
                    ExportNormalsParams->WeightedNormalsRead = HighResWeightedNormalsSRV;
                    ExportNormalsParams->OutputNormals = HighResNormalOutUAV;
                    ExportNormalsParams->ParticleCount = NumHighResParticles;
                    FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("ExportHighResNormals"), ExportNormalsShader, ExportNormalsParams, FIntVector(HighResGroups, 1, 1));
                }
            }
             // =============================================================
            // 阶段 4：导出数据到实时纹理 (VAT) 或 回读
            // =============================================================
            if (bIsLastSubstep)
            {
                if (bLocalUpdateViaGPU && PosRTResource && NormRTResource)
                {
                    // 将外部的 RHI 纹理注册进 RDG
                    FRDGTextureRef PosTexRDG = RegisterExternalTexture(GraphBuilder, PosRTResource->GetTextureRHI(), TEXT("PositionVAT_Tex"));
                    FRDGTextureRef NormTexRDG = RegisterExternalTexture(GraphBuilder, NormRTResource->GetTextureRHI(), TEXT("NormalVAT_Tex"));
                
                    TShaderMapRef<FExportToTextureCS> VATShader(GetGlobalShaderMap(GMaxRHIFeatureLevel));
                    auto* VATParams = GraphBuilder.AllocParameters<FExportToTextureCS::FParameters>();

                    // 【核心修复】：动态决定 VAT 读取的源头！
                    if (bUseProxy)
                    {
                        // 开启代理时，读取蒙皮计算后的高模 Buffer
                        FRDGBufferRef HighResParticleRDG = GraphBuilder.RegisterExternalBuffer(HighResParticleBuf);
                        VATParams->HighResParticles = GraphBuilder.CreateSRV(HighResParticleRDG);
                        VATParams->HighResNormals = GraphBuilder.CreateSRV(GraphBuilder.RegisterExternalBuffer(HighResNormalOutputBuf)); 
                    }
                    else
                    {
                        // 未开启代理时，物理引擎计算的 ProxyParticle 本身就是高模，直接直通输出！
                        VATParams->HighResParticles = GraphBuilder.CreateSRV(ProxyParticleRDG);
                        // 读我们在“阶段3.5”生成的本体法线
                        VATParams->HighResNormals = GraphBuilder.CreateSRV(GraphBuilder.RegisterExternalBuffer(NormalOutputBuf)); 
                    }
                
                    // 初始位置大家都是一样的，直接用
                    VATParams->HighResRestPositions = GraphBuilder.CreateSRV(GraphBuilder.RegisterExternalBuffer(HighResRestPosPooledBuf));
                
                    VATParams->OutPositionTex = GraphBuilder.CreateUAV(PosTexRDG);
                    VATParams->OutNormalTex = GraphBuilder.CreateUAV(NormTexRDG);
                    VATParams->VertexCount = NumHighResParticles; // 无代理时 ProxyCount == HighResCount，直接用这个无妨
                    VATParams->TexDimension = LocalTexDimension;
                
                    // 计算 ThreadGroups
                    uint32 ExportGroups = FMath::DivideAndRoundUp((uint32)NumHighResParticles, 64u);
                    FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("ExportVAT"), VATShader, VATParams, FIntVector(ExportGroups, 1, 1));
                }
                else if (bUseProxy) 
                {
                    // =============================================================
                    // 阶段 3：导出高模位置 (用于 CPU Readback 更新网格)
                    // 只在开启代理且非直通渲染时需要，因为无代理时 ProxyParticleOutput 已经做了导出
                    // =============================================================
                    FRDGBufferRef HighResParticleRDG = GraphBuilder.RegisterExternalBuffer(HighResParticleBuf);
                    FRDGBufferRef PositionOutputRDG = GraphBuilder.RegisterExternalBuffer(HighResPositionOutputBuf);
                    auto PositionOutputUAV = GraphBuilder.CreateUAV(PositionOutputRDG);
                
                    TShaderMapRef<FExportPositionsCS> ExportShader(GetGlobalShaderMap(GMaxRHIFeatureLevel));
                    auto* ExportParams = GraphBuilder.AllocParameters<FExportPositionsCS::FParameters>();
                    ExportParams->ReadParticles = GraphBuilder.CreateSRV(HighResParticleRDG); 
                    ExportParams->OutputPositions = PositionOutputUAV;
                    ExportParams->ParticleCount = NumHighResParticles;
                
                    uint32 HighResGroups = FMath::DivideAndRoundUp((uint32)NumHighResParticles, 64u);
                    FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("ExportHighResPositions"), ExportShader, ExportParams, FIntVector(HighResGroups, 1, 1));
                }
            }
            GraphBuilder.Execute();
        }
    );
}

void UMySoftBodyMeshComponent::ExtractGDF(FRDGBuilder& GraphBuilder, const FViewInfo& View)
{
    if (!View.GlobalDistanceFieldInfo.bInitialized || !GDFCache.IsValid())
    {
        if (GDFCache.IsValid()) GDFCache->bValid = false;
        return;
    }

    const FGlobalDistanceFieldParameterData& GDFParams = View.GlobalDistanceFieldInfo.ParameterData;
    if (!GDFParams.PageAtlasTexture || !GDFParams.PageTableTexture)
    {
        GDFCache->bValid = false;
        return;
    }

    // 页表纹理源与引擎 UpdateParameterData 一致: 优先组合纹理
    IPooledRenderTarget* PageTablePooled = View.GlobalDistanceFieldInfo.PageTableCombinedTexture.GetReference();
    if (!PageTablePooled)
    {
        PageTablePooled = View.GlobalDistanceFieldInfo.PageTableLayerTextures[GDF_Full].GetReference();
    }
    if (!PageTablePooled)
    {
        GDFCache->bValid = false;
        return;
    }

    // 用 QueueTextureExtraction 提取稳定引用 (RDG 保证纹理不会被池回收覆盖)
    GraphBuilder.QueueTextureExtraction(
        GraphBuilder.RegisterExternalTexture(TRefCountPtr<IPooledRenderTarget>(View.GlobalDistanceFieldInfo.PageAtlasTexture.GetReference()), TEXT("SoftBodyGDFPageAtlas")),
        &GDFCache->PageAtlasPooled);
    GraphBuilder.QueueTextureExtraction(
        GraphBuilder.RegisterExternalTexture(TRefCountPtr<IPooledRenderTarget>(PageTablePooled), TEXT("SoftBodyGDFPageTable")),
        &GDFCache->PageTablePooled);

    GDFCache->GDFData = GDFParams;
    GDFCache->PreViewTranslation = View.ViewMatrices.GetPreViewTranslation();
    GDFCache->bValid = true;
}

void UMySoftBodyMeshComponent::AddGDFCollisionPass(FRDGBuilder& GraphBuilder)
{
    if (!bUseDistanceFieldCollision || !bGPUResourcesInitialized || !ProxyParticlePooledBuffer.IsValid())
    {
        return;
    }
    if (!GDFCache.IsValid() || !GDFCache->bValid
        || !GDFCache->PageAtlasPooled.IsValid() || !GDFCache->PageTablePooled.IsValid())
    {
        return;
    }

    int32 NumParticles = ProxyParticleCount;
    uint32 ParticleGroups = FMath::DivideAndRoundUp((uint32)NumParticles, 64u);

    FRDGBufferRef ProxyParticleRDG = GraphBuilder.RegisterExternalBuffer(ProxyParticlePooledBuffer);
    auto ProxyParticleUAV = GraphBuilder.CreateUAV(ProxyParticleRDG);

    FRDGTextureRef GDFPageAtlasRDG = GraphBuilder.RegisterExternalTexture(GDFCache->PageAtlasPooled, TEXT("SoftBodyGDFPageAtlas"));
    FRDGTextureRef GDFPageTableRDG = GraphBuilder.RegisterExternalTexture(GDFCache->PageTablePooled, TEXT("SoftBodyGDFPageTable"));

    const FGlobalDistanceFieldParameterData& GDFParams = GDFCache->GDFData;

    TShaderMapRef<FCollideGDFCS> Shader(GetGlobalShaderMap(GMaxRHIFeatureLevel));
    auto* Params = GraphBuilder.AllocParameters<FCollideGDFCS::FParameters>();
    Params->RWParticles = ProxyParticleUAV;
    Params->GlobalDistanceFieldPageAtlasTexture = GDFPageAtlasRDG;
    Params->GlobalDistanceFieldPageTableTexture = GDFPageTableRDG;
    Params->GlobalDistanceFieldPageAtlasTextureSampler = TStaticSamplerState<SF_Trilinear, AM_Wrap, AM_Wrap, AM_Wrap>::GetRHI();
    for (int32 i = 0; i < GlobalDistanceField::MaxClipmaps; ++i)
    {
        Params->GlobalVolumeTranslatedCenterAndExtent[i] = GDFParams.TranslatedCenterAndExtent[i];
        Params->GlobalVolumeTranslatedWorldToUVAddAndMul[i] = GDFParams.TranslatedWorldToUVAddAndMul[i];
    }
    Params->GlobalDistanceFieldInvPageAtlasSize = FVector3f(GDFParams.InvPageAtlasSize);
    Params->GlobalDistanceFieldClipmapSizeInPages = (uint32)GDFParams.ClipmapSizeInPages;
    Params->GlobalVolumeTexelSize = 1.0f / GDFParams.GlobalDFResolution;
    Params->NumGlobalSDFClipmaps = (uint32)GDFParams.NumGlobalSDFClipmaps;
    Params->ParticleCount = (uint32)NumParticles;
    Params->ParticleRadius = ParticleRadius;
    Params->PreViewTranslation = FVector3f(GDFCache->PreViewTranslation);
    Params->SubstepTime = St;
    Params->GDFFriction = CollisionFriction;
    Params->GDFSkinOffset = GDFSkinOffset;

    FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("CollideGDF"), Shader, Params, FIntVector(ParticleGroups, 1, 1));
}

void UMySoftBodyMeshComponent::ReadbackGPUPositions()
{
    if (!bGPUResourcesInitialized || !HighResPositionOutputPooledBuffer.IsValid()) return;

    // ========== 1. 高模位置 Readback (用于网格渲染) ==========
    if (HighResPositionOutputPooledBuffer.IsValid())
    {
        ProcessGPUReadback<FVector3f>(
            CachedGPUPositions, ReadyGPUPositions, bNewDataReady,
            ParticleReadback, bReadbackPending,
            HighResPositionOutputPooledBuffer, 
            HighResParticleCount, 
            TEXT("SoftBodyHighResReadback")
        );

        if (ReadyGPUPositions.Num() == HighResParticleCount) 
        {
            if (RenderPositions.Num() != HighResParticleCount) RenderPositions.SetNum(HighResParticleCount);

            for (int32 i = 0; i < HighResParticleCount; ++i)
            {
                HighResParticles[i].Position = FVector(ReadyGPUPositions[i]);
                RenderPositions[i] = FVector(ReadyGPUPositions[i]);
            }
        }
    }
    // ========== 2. 低模位置 Readback (仅用于 Debug 显示) ==========
    if (bUseProxy && bShowProxyNodes || ProxyPositionOutputPooledBuffer.IsValid())
    {
        ProcessGPUReadback<FVector3f>(
            CachedProxyPositions, ReadyProxyPositions, bNewProxyDataReady,
            ProxyReadback, bProxyReadbackPending,
            ProxyPositionOutputPooledBuffer, 
            ProxyParticleCount, 
            TEXT("SoftBodyProxyReadback")
        );
        
        // （可选）如果你的其他 C++ 逻辑需要读取最新的低模位置，可以同步回 ProxyParticles
        if (ReadyProxyPositions.Num() == ProxyParticleCount)
        {
            for (int32 i = 0; i < ProxyParticleCount; ++i)
            {
                ProxyParticles[i].Position = FVector(ReadyProxyPositions[i]);
            }
        }
    }
    else if (!bShowProxyNodes)
    {
        // 如果关闭了 Debug，清理数组防止 TickUpdateCloth 继续绘制旧数据
        ReadyProxyPositions.Empty();
    }
    // ========== 法线 Readback ==========
    if (bUse_VolumePressureForce)
    {
        ProcessGPUReadback<FVector3f>(
            CachedGPUNormals, ReadyGPUNormals, bNewNormalDataReady,
            NormalReadback, bNormalReadbackPending,
            NormalOutputPooledBuffer, ProxyParticleCount,
            TEXT("SoftBodyNormalReadback")
        );

        // 应用法线数据
        if (ReadyGPUNormals.Num() == ProxyParticleCount)
        {
            for (int32 i = 0; i < ProxyParticleCount; ++i)
            {
                CurrentNormals[i] = FVector(ReadyGPUNormals[i]);
            }
        }

        // ========== 体积 Readback ==========
        ProcessGPUReadback<int32>(
            CachedGPUVolume, ReadyGPUVolume, bNewVolumeDataReady,
            VolumeReadback, bVolumeReadbackPending,
            VolumeOutputPooledBuffer, 1,
            TEXT("SoftBodyVolumeReadback")
        );

        // 应用体积数据并打印
        if (ReadyGPUVolume.Num() == 1)
        {
            int32 VolumeInt = ReadyGPUVolume[0];
            float Volume = (float)VolumeInt / 6.0f;
            if (GEngine)
            {
               //GEngine->AddOnScreenDebugMessage(-1,5.0f,FColor::Yellow,FString::Printf(TEXT("GPU Volume: VolumeInt=%d, Volume=%.6f, RestVolume=%.6f"), VolumeInt, Volume, restVolume));
                UE_LOG(LogTemp, Log, TEXT("GPU Volume: VolumeInt=%d, Volume=%.6f, RestVolume=%.6f"), VolumeInt, Volume, restVolume);
            }
        }
        // ========== 高模法线 Readback ==========
        if (HighResNormalOutputPooledBuffer.IsValid())
        {
            ProcessGPUReadback<FVector3f>(
                CachedHighResGPUNormals, ReadyHighResGPUNormals, bNewHighResNormalDataReady,
                HighResNormalReadback, bHighResNormalReadbackPending,
                HighResNormalOutputPooledBuffer, 
                HighResParticleCount, 
                TEXT("SoftBodyHighResNormalReadback")
            );

            if (ReadyHighResGPUNormals.Num() == HighResParticleCount) 
            {
                if (HighResCurrentNormals.Num() != HighResParticleCount) HighResCurrentNormals.SetNumUninitialized(HighResParticleCount);
            
                for (int32 i = 0; i < HighResParticleCount; ++i)
                {
                    HighResCurrentNormals[i] = FVector(ReadyHighResGPUNormals[i]);
                }
            }
        }
    }
}



void UMySoftBodyMeshComponent::BuildHighResTopologyCache(const UE::Geometry::FDynamicMesh3& Mesh)
{
    int32 VCount = Mesh.VertexCount();
    
    // 1. 提取三角形
    HighResSimulationTriangles.Reset();
    HighResSimulationTriangles.Reserve(Mesh.TriangleCount());
    for (int32 tid : Mesh.TriangleIndicesItr())
    {
        UE::Geometry::FIndex3i Tri = Mesh.GetTriangle(tid);
        HighResSimulationTriangles.Emplace(Tri.A, Tri.B, Tri.C);
    }

    // 2. 统计每个顶点的相邻三角形
    TArray<TArray<int32>> TempVtxToTri;
    TempVtxToTri.SetNum(VCount);
    for (int32 tid : Mesh.TriangleIndicesItr())
    {
        UE::Geometry::FIndex3i Tri = Mesh.GetTriangle(tid);
        if (Mesh.IsVertex(Tri.A)) TempVtxToTri[Tri.A].Add(tid);
        if (Mesh.IsVertex(Tri.B)) TempVtxToTri[Tri.B].Add(tid);
        if (Mesh.IsVertex(Tri.C)) TempVtxToTri[Tri.C].Add(tid);
    }

    // 3. 拍扁为 CSR 格式
    HighResVertexTriangleOffsets.SetNum(VCount);
    HighResVertexTriangleCounts.SetNum(VCount);
    HighResVertexTriangleIndices.Empty();

    uint32 Offset = 0;
    for (int32 i = 0; i < VCount; ++i)
    {
        HighResVertexTriangleOffsets[i] = Offset;
        HighResVertexTriangleCounts[i] = TempVtxToTri[i].Num();
        for (int32 TriID : TempVtxToTri[i])
        {
            HighResVertexTriangleIndices.Add(TriID);
        }
        Offset += HighResVertexTriangleCounts[i];
    }
}


void UMySoftBodyMeshComponent::GenerateProxyAndMapping()
{
    // 1. 复制 RenderMesh 到 ProxyMesh
    ProxyMesh.Copy(RenderMesh);
    if (ProxyMesh.HasAttributes())
    {
        ProxyMesh.DiscardAttributes();
    }
    // =========================================================
    // 2. 执行简化 
    // =========================================================
    // 创建简化器，直接传入 ProxyMesh 的地址
    UE::Geometry::FQEMSimplification Simplifier(&ProxyMesh);
    // 执行简化到目标面数
    Simplifier.SimplifyToTriangleCount(TargetProxyTriangleCount);
    // 整理内存
    ProxyMesh.CompactInPlace();
    int32 HighResCount = RenderMesh.VertexCount();
    int32 ProxyCount = ProxyMesh.VertexCount();
    UE_LOG(LogTemp, Log, TEXT("SoftBody: Generated Proxy (Native). HighRes: %d, Proxy: %d"), HighResCount, ProxyCount);
    // =========================================================
    // 3. 预计算低模的面法线和边法线 (严格调用 UE 底层 API)
    // =========================================================
    TArray<FVector3d> FaceNormals;
    FaceNormals.SetNumZeroed(ProxyMesh.MaxTriangleID());
    for (int32 tid : ProxyMesh.TriangleIndicesItr())
    {
        // 【严谨优化 1】直接使用 Geometry 框架的权威法线获取函数，完美契合 UE 内部的绕序与左手系规则
        FaceNormals[tid] = ProxyMesh.GetTriNormal(tid);
    }
    TArray<FVector3d> EdgeNormals;
    EdgeNormals.SetNumZeroed(ProxyMesh.MaxEdgeID());
    for (int32 eid : ProxyMesh.EdgeIndicesItr())
    {
        UE::Geometry::FIndex2i TriPair = ProxyMesh.GetEdgeT(eid);
        FVector3d N1 = FaceNormals[TriPair.A];
        
        // 边界边直接用单面法线，非边界边取平均
        if (TriPair.B != UE::Geometry::FDynamicMesh3::InvalidID)
        {
            FVector3d N2 = FaceNormals[TriPair.B];
            EdgeNormals[eid] = (N1 + N2).GetSafeNormal();
        }
        else
        {
            EdgeNormals[eid] = N1;
        }
    }
    
    // 构建低模(Proxy)的空间搜索树，这次我们用它来找三角形！
    UE::Geometry::FDynamicMeshAABBTree3 ProxySpatial(&ProxyMesh);
    ProxySpatial.Build();

    // 确保在多线程之前已经分配好内存（这步极其重要，不能在多线程里 Add）
    ScaffoldBindings.SetNumUninitialized(HighResCount);

    // 开启多线程并行处理
    ParallelFor(HighResCount, [&](int32 HighResID)
    {
        FVector3d HighResPos = RenderMesh.GetVertex(HighResID);
        
        // ---------------------------------------------------------
        // [新增] 1. 计算高模顶点自身原本的法线 (求周围面的平均法线)
        // ---------------------------------------------------------
        FVector3d HighResNormal = FVector3d::Zero();
        TArray<int32> AssociatedTris;
        RenderMesh.GetVtxTriangles(HighResID, AssociatedTris);
        for (int32 Tid : AssociatedTris)
        {
            HighResNormal += RenderMesh.GetTriNormal(Tid);
        }
        if (HighResNormal.SquaredLength() > UE_DOUBLE_KINDA_SMALL_NUMBER)
        {
            HighResNormal.Normalize();
        }
        else
        {
            HighResNormal = FVector3d::UpVector;
        }

        // 寻找绝对空间距离最近的面，作为 BFS 的起点
        double InitialDistSq = UE_BIG_NUMBER;
        int32 InitialNearestTriID = ProxySpatial.FindNearestTriangle(HighResPos, InitialDistSq);
        if (InitialNearestTriID < 0) return;
        
        // [废除] 删掉原来有缺陷的 InitialSign 逻辑
        
        // 拓扑广度优先搜索 (BFS)
        TArray<int32, TInlineAllocator<64>> Queue;
        TArray<int32, TInlineAllocator<64>> Visited;
        Queue.Add(InitialNearestTriID);
        Visited.Add(InitialNearestTriID);
        int32 FinalTargetTriID = -1;
        int32 Head = 0;
        int32 MaxSearchNodes = 15; // 稍微扩大一点搜索范围，因为加了法线剔除，可能会多找几层
        
        while (Head < Queue.Num() && Head < MaxSearchNodes)
        {
            int32 CandidateTriID = Queue[Head++];

            UE::Geometry::FIndex3i TriV = ProxyMesh.GetTriangle(CandidateTriID);
            FVector3d A = ProxyMesh.GetVertex(TriV.A);
            FVector3d B = ProxyMesh.GetVertex(TriV.B);
            FVector3d C = ProxyMesh.GetVertex(TriV.C);
            FVector3d TriNormal = FaceNormals[CandidateTriID];

            // ---------------------------------------------------------
            // [核心新增] 条件 A：法向一致性校验 (Dot Product)
            // 如果候选面与高模顶点方向背道而驰 (夹角超过90度)，直接毙掉！
            // 这里的 0.0 可以视情况放宽到 -0.1，允许一点点弯曲误差
            // ---------------------------------------------------------
            if (HighResNormal.Dot(TriNormal) < 0.0)
            {
                goto ExpandSearch; // 跳过当前面，继续向外搜索
            }

            { // 只有法向一致，才进行条件 B：三棱柱内侧校验
                int32 E_AB = ProxyMesh.FindEdge(TriV.A, TriV.B);
                int32 E_BC = ProxyMesh.FindEdge(TriV.B, TriV.C);
                int32 E_CA = ProxyMesh.FindEdge(TriV.C, TriV.A);

                FVector3d N_AB = EdgeNormals[E_AB];
                FVector3d N_BC = EdgeNormals[E_BC];
                FVector3d N_CA = EdgeNormals[E_CA];

                FVector3d PlaneNormal_AB = N_AB.Cross(B - A);
                FVector3d PlaneNormal_BC = N_BC.Cross(C - B);
                FVector3d PlaneNormal_CA = N_CA.Cross(A - C);

                if (PlaneNormal_AB.Dot(C - A) < 0) PlaneNormal_AB = -PlaneNormal_AB;
                if (PlaneNormal_BC.Dot(A - B) < 0) PlaneNormal_BC = -PlaneNormal_BC;
                if (PlaneNormal_CA.Dot(B - C) < 0) PlaneNormal_CA = -PlaneNormal_CA;

                bool bInsideAB = (HighResPos - A).Dot(PlaneNormal_AB) >= -1e-4;
                bool bInsideBC = (HighResPos - B).Dot(PlaneNormal_BC) >= -1e-4;
                bool bInsideCA = (HighResPos - C).Dot(PlaneNormal_CA) >= -1e-4;

                if (bInsideAB && bInsideBC && bInsideCA)
                {
                    FinalTargetTriID = CandidateTriID; // 完美匹配！
                    break;
                }
            }

        ExpandSearch:
            // 向外拓扑扩展
            UE::Geometry::FIndex3i NbrTris = ProxyMesh.GetTriNeighbourTris(CandidateTriID);
            for (int j = 0; j < 3; ++j)
            {
                int32 NbrTriID = NbrTris[j];
                if (NbrTriID != UE::Geometry::FDynamicMesh3::InvalidID && !Visited.Contains(NbrTriID))
                {
                    Queue.Add(NbrTriID);
                    Visited.Add(NbrTriID);
                }
            }
        }

        // 兜底策略：如果 BFS 没找到完美面
        if (FinalTargetTriID == -1)
        {
            // 如果起点的法线不对，说明空间最近的面是个反面。
            // 此时宁可绑定失败(或者绑定到队列里第一个符合法线的面)，也不能绑给反面。
            // 这里为了简单，找 Visited 里第一个法线大致对的：
            for(int32 vTri : Visited) {
                if (HighResNormal.Dot(FaceNormals[vTri]) > 0.0) {
                    FinalTargetTriID = vTri;
                    break;
                }
            }
            // 如果还是找不到，只能被迫妥协给空间最近面了（属于极度糟糕的拓扑区域）
            if (FinalTargetTriID == -1) FinalTargetTriID = InitialNearestTriID; 
        }
        double MaxBindDistance = 5.0; // 允许的最大绑定距离
        
        // 只有当 BFS 选出的面不是空间最近面时，才需要检查距离
        if (FinalTargetTriID != InitialNearestTriID)
        {
            // 快速估算：计算高模顶点到目标三角形中心的距离
            UE::Geometry::FIndex3i TargetTri = ProxyMesh.GetTriangle(FinalTargetTriID);
            FVector3d Center = (ProxyMesh.GetVertex(TargetTri.A) + 
                                ProxyMesh.GetVertex(TargetTri.B) + 
                                ProxyMesh.GetVertex(TargetTri.C)) / 3.0;

            if (FVector3d::Distance(HighResPos, Center) > MaxBindDistance)
            {
                // 距离超限！放弃完美拓扑，强行绑定给空间最近面
                FinalTargetTriID = InitialNearestTriID;
            }
        }
        // =========================================================
        // 5. 计算重心坐标与法线偏移
        // =========================================================
        UE::Geometry::FIndex3i Tri = ProxyMesh.GetTriangle(FinalTargetTriID);
        FVector3d A = ProxyMesh.GetVertex(Tri.A);
        FVector3d B = ProxyMesh.GetVertex(Tri.B);
        FVector3d C = ProxyMesh.GetVertex(Tri.C);

        FVector3d UnnormalizedNormal = (B - A).Cross(C - A);
        double AreaX2 = UnnormalizedNormal.Length();
        
        FVector3f Bary(1.0f, 0.0f, 0.0f);
        double NormalOffset = 0.0;
        if (AreaX2 > 1e-8)
        {
            FVector3d NormalizedTriNormal = UnnormalizedNormal / AreaX2; 
            NormalOffset = (HighResPos - A).Dot(NormalizedTriNormal);
            
            FVector3d ProjectedPos = HighResPos - NormalizedTriNormal * NormalOffset;
            
            FVector3d NA = (C - B).Cross(ProjectedPos - B);
            FVector3d NB = (A - C).Cross(ProjectedPos - C);
            
            Bary.X = (float)((NA.Dot(NormalizedTriNormal) > 0 ? 1.0 : -1.0) * NA.Length() / AreaX2);
            Bary.Y = (float)((NB.Dot(NormalizedTriNormal) > 0 ? 1.0 : -1.0) * NB.Length() / AreaX2);
            Bary.Z = 1.0f - Bary.X - Bary.Y; 
        }

        // 写入结果，全程并发安全
        ScaffoldBindings[HighResID].ProxyTriIndex = FinalTargetTriID;
        ScaffoldBindings[HighResID].Barycentric = Bary;
        ScaffoldBindings[HighResID].NormalOffset = (float)NormalOffset;
    });
    //DebugDrawBindings();
}


// =========================================================
// 调试
// =========================================================

void UMySoftBodyMeshComponent::DBG_ShowParticles() const
{
    for(const auto& Pt : ProxyParticles)
    {
        DrawDebugSphere(GetWorld(), Pt.Position, ParticleRadius, 4, FColor::Red, false, -1.0f);
    }
}
void UMySoftBodyMeshComponent::DebugDrawBindings()
{
    UWorld* World = GetWorld();
    if (!World) return;

    // 获取组件的 Transform，确保 Debug 线画在世界坐标下的正确位置
    FTransform CompTransform = GetComponentTransform();

    int32 HighResCount = ScaffoldBindings.Num();
    if (HighResCount == 0) return;

    UE_LOG(LogTemp, Warning, TEXT("=== 开始绘制绑定 Debug 线 (全量绘制，共 %d 个顶点) ==="), HighResCount);

    // 针对简单模型 Debug，移除任何 Step 省略逻辑，遍历所有高模顶点
    for (int32 i = 0; i < HighResCount; ++i)
    {
        const FScaffoldBinding& Binding = ScaffoldBindings[i];
        
        // 如果因为某些奇异原因没有绑定上面片，跳过
        if (Binding.ProxyTriIndex < 0) continue;

        // 1. 获取高模点初始位置 (红点)
        FVector3d HighResLocalPos = RenderMesh.GetVertex(i);
        FVector HighResWorldPos = CompTransform.TransformPosition((FVector)HighResLocalPos);

        // 2. 获取低模三角形的三个顶点
        UE::Geometry::FIndex3i Tri = ProxyMesh.GetTriangle(Binding.ProxyTriIndex);
        FVector3d A = ProxyMesh.GetVertex(Tri.A);
        FVector3d B = ProxyMesh.GetVertex(Tri.B);
        FVector3d C = ProxyMesh.GetVertex(Tri.C);

        // 3. 利用重心坐标，算出低模面上的“基础投影点” (绿点)
        float u = Binding.Barycentric.X;
        float v = Binding.Barycentric.Y;
        float w = Binding.Barycentric.Z;
        FVector3d BaseLocalPos = u * A + v * B + w * C;
        FVector BaseWorldPos = CompTransform.TransformPosition((FVector)BaseLocalPos);

        // 4. 绘制 Debug 元素
        // 将保留时间设长一点（20秒），方便你旋转相机仔细观察
        float LifeTime = 20.0f;

        // 画投影点 (绿点) - 代表点在低模上的锚固位置
        DrawDebugPoint(World, BaseWorldPos, 5.0f, FColor::Green, false, LifeTime);
        
        // 画高模点 (红点) - 代表高模实际的位置
        DrawDebugPoint(World, HighResWorldPos, 5.0f, FColor::Red, false, LifeTime);
        
        // 画连线 (黄线) - 代表偏移路径
        // 线条稍微加粗一点点 (Thickness: 0.5f)，看得更清楚
        DrawDebugLine(World, BaseWorldPos, HighResWorldPos, FColor::Yellow, false, LifeTime, 0, 0.5f);
    }
}

void UMySoftBodyMeshComponent::FixCantileverEdge(float Tolerance, int32 Axis)
{
    // ProxyParticleCount 是你类里的变量，ProxyParticles 是你的粒子数组
    if (ProxyParticleCount == 0 || ProxyParticles.Num() == 0) return;

    float MinValue = UE_BIG_NUMBER;

    // Step 1: 遍历寻找极小值（边界）
    for (int32 i = 0; i < ProxyParticleCount; ++i)
    {
        float CurrentVal = 0.0f;
        switch (Axis)
        {
        case 0: CurrentVal = ProxyParticles[i].Position.X; break;
        case 1: CurrentVal = ProxyParticles[i].Position.Y; break;
        case 2: CurrentVal = ProxyParticles[i].Position.Z; break;
        }

        if (CurrentVal < MinValue)
        {
            MinValue = CurrentVal;
        }
    }

    // Step 2: 再次遍历，将处于容差范围内的边缘顶点设为固定状态
    int32 PinnedCount = 0;
    for (int32 i = 0; i < ProxyParticleCount; ++i)
    {
        float CurrentVal = 0.0f;
        switch (Axis)
        {
        case 0: CurrentVal = ProxyParticles[i].Position.X; break;
        case 1: CurrentVal = ProxyParticles[i].Position.Y; break;
        case 2: CurrentVal = ProxyParticles[i].Position.Z; break;
        }

        // 如果坐标在最小边界的容差框内部
        if (CurrentVal <= MinValue + Tolerance)
        {
            // 在你的 CPU 数据结构中，状态设为 0 (Pinned)
            ProxyParticles[i].state = 0; 
            PinnedCount++;
        }
    }

    UE_LOG(LogTemp, Warning, TEXT("[悬臂梁测试] 边界固定完成: 极值=%f, 共固定了 %d 个顶点"), MinValue, PinnedCount);
}

int32 UMySoftBodyMeshComponent::SelectVertexByRay(FVector RayOrigin, FVector RayDirection, float ClickTolerance)
{
    if (!clothStateExists || ProxyParticleCount == 0) return -1;

    int32 BestID = -1;
    float MinDepth = UE_BIG_NUMBER; // 记录深度，确保选中最前面的点

    // 确保射线方向是归一化的
    RayDirection.Normalize();
    
    // 容差的平方，避免开方运算
    float ToleranceSq = ClickTolerance * ClickTolerance;

    for (int32 i = 0; i < ProxyParticleCount; ++i)
    {
        FVector Pt = ProxyParticles[i].Position;
        FVector Diff = Pt - RayOrigin;
        
        // 1. 将粒子位置投影到射线上，求出沿着射线的距离 (Depth)
        float T = FVector::DotProduct(Diff, RayDirection);
        
        // 如果 T < 0，说明这个点在摄像机背后，直接跳过
        if (T < 0.0f) continue;

        // 2. 求射线上的投影点坐标
        FVector ProjectedPt = RayOrigin + (RayDirection * T);
        
        // 3. 计算粒子到射线的垂直距离平方
        float DistToRaySq = FVector::DistSquared(Pt, ProjectedPt);

        // 如果在这个“点击圆柱体”范围内
        if (DistToRaySq < ToleranceSq)
        {
            // 找出离摄像机最近的那个点（防止点到模型背面的点）
            if (T < MinDepth)
            {
                MinDepth = T;
                BestID = i;
            }
        }
    }

    SelectedVertexID = BestID;
    
    if (BestID != -1)
    {
        UE_LOG(LogTemp, Log, TEXT("SoftBody Debug: Ray hit Vertex ID %d"), BestID);
        if (GEngine)
        {
            GEngine->AddOnScreenDebugMessage(-1, 3.0f, FColor::Green, FString::Printf(TEXT("Ray Hit Vertex ID: %d"), BestID));
        }
    }
    
    return BestID;
}
