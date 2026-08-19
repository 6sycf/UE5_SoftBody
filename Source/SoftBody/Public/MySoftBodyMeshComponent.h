#pragma once

#include "CoreMinimal.h"
#include "Components/DynamicMeshComponent.h"
#include "Components/StaticMeshComponent.h"
#include "RenderGraphResources.h"
#include <atomic>
#include "MySoftBodyMeshComponent.generated.h"

// 宏定义：强制内联，优化高频数学计算
#define INLINE __forceinline

// 前置声明
struct FMySoftBodyConstraint;
class UOpenHapticsComponent;
class FRHIGPUBufferReadback;
class ISceneViewExtension;
class FGlobalDistanceFieldParameterData;
struct IPooledRenderTarget;
class FViewInfo;
struct FSoftBodyGDFCacheData;


// =========================================================
// 1. 粒子结构体 (物理状态)
// =========================================================
struct FMySoftBodyParticle
{
    FMySoftBodyParticle()
        : Position(0, 0, 0)
        , PrevPosition(0, 0, 0)
        , Velocity(0, 0, 0)
        , Force(0, 0, 0)
        , Col(255, 255, 255, 255)
        , ID(-1)
        , state(1)
        , conCount(0)
    {}

    FVector Position;      // 当前位置 (double精度)
    FVector PrevPosition;  // 上一帧位置
    FVector Velocity;      // 速度
    FVector Force;         // 累积受力 (体积压力等)
    FColor Col;            // 粒子颜色 (调试或渲染用)

    int32 ID;              // 粒子索引
    int8 state;            // 状态: 1=活动, 0=固定(Pinned)
    int8 conCount;         // 连接的约束数量
    float InvMass = 1.0f;
};
// =========================================================
// [GPU 准备] 2. 三角形缓存 (对应 HLSL: struct Triangle)
// =========================================================
// 用于散度公式计算体积，完全替代 DynamicMesh 的迭代器
struct FSoftBodyTriangle
{
    int32 A;
    int32 B;
    int32 C;

    FSoftBodyTriangle() : A(0), B(0), C(0) {}
    FSoftBodyTriangle(int32 InA, int32 InB, int32 InC) : A(InA), B(InB), C(InC) {}
};
// =========================================================
// [GPU 准备] 3. 约束结构体 (对应 HLSL: struct Constraint)
// =========================================================
// 纯数据结构，只存索引，不存指针/引用
struct FMySoftBodyConstraint
{
    int32 P0_Index; // 粒子 A 的索引
    int32 P1_Index; // 粒子 B 的索引
    float RestLength;
    float Lambda;

    FMySoftBodyConstraint(int32 InP0, int32 InP1, float InLen)
        : P0_Index(InP0), P1_Index(InP1), RestLength(InLen) , Lambda(0.0f)
    {}
};
// =========================================================
// 映射关系：描述一个 HighRes 顶点是如何“挂”在 Proxy 三角形上的
// =========================================================
struct FScaffoldBinding
{
    int32 ProxyTriIndex;   // 属于 Proxy 的哪个三角形
    FVector3f Barycentric; // 重心坐标 (u, v, w)
    float NormalOffset;    // 沿法线方向的距离偏移
};
// =========================================================
// 二面角约束结构体 (对应 HLSL)
// =========================================================
struct FMySoftBodyDihedralConstraint
{
    int32 P1_Index; // 共享边顶点 1
    int32 P2_Index; // 共享边顶点 2
    int32 P3_Index; // 三角形 A 的对角点
    int32 P4_Index; // 三角形 B 的对角点
    float RestAngle; // 初始二面角 (弧度)
    float Lambda;    // XPBD 乘子

    FMySoftBodyDihedralConstraint(int32 InP1, int32 InP2, int32 InP3, int32 InP4, float InAngle)
        : P1_Index(InP1), P2_Index(InP2), P3_Index(InP3), P4_Index(InP4), RestAngle(InAngle), Lambda(0.0f)
    {}
};
// =========================================================
// 3. 软体组件类
// =========================================================
UCLASS(hidecategories = (Object, Physics, Activation, Navigation, Transform, "Components|Activation"), editinlinenew, meta = (BlueprintSpawnableComponent), ClassGroup = Rendering)
class SOFTBODY_API UMySoftBodyMeshComponent : public UDynamicMeshComponent
{
    GENERATED_BODY()

public:
    UMySoftBodyMeshComponent(const FObjectInitializer& ObjectInitializer);

    // 析构函数 (虽然用了智能容器，保留它是个好习惯)
    virtual ~UMySoftBodyMeshComponent() override;

    // --- UActorComponent Overrides ---
    virtual void OnRegister() override;
    virtual void TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

    // 在场景渲染管线内提取 GDF 稳定纹理 (由视图扩展调用)
    void ExtractGDF(FRDGBuilder& GraphBuilder, const FViewInfo& View);

    // 在独立模拟 graph 内插入 GDF 碰撞 pass (从缓存读取)
    void AddGDFCollisionPass(FRDGBuilder& GraphBuilder);

#if WITH_EDITOR
    // 监听编辑器属性修改，防止崩溃
    virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
#endif

public:
   // =========================================================
    // 属性设置
    // =========================================================
    UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Soft Body Setup")
    UStaticMeshComponent* SourceStaticMesh;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Soft Body Setup")
    bool bShowStaticMesh;

    // [引用] 指向场景里的触觉组件
    // UPROPERTY 加上 Transient 表示不需要保存到磁盘，只在运行时有效
    UPROPERTY(Transient, VisibleAnywhere, Category = "Soft Body Interaction")
    UOpenHapticsComponent* HapticsComponent;

    // --- 物理参数 ---
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Soft Body Simulation")
    bool bSimulate;
    /** * [物理子步长 (秒)] - 核心精度参数
         * 决定了每秒进行多少次物理计算 (Hz = 1/SubstepTime)。
         * 值越小：计算越频繁，防穿模效果越好，软体越稳定，但 CPU 开销越大。
         * 推荐值：0.005 (200Hz) ~ 0.002 (500Hz)。对于触觉交互，不能大于 0.01。
         */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Soft Body Simulation", meta = (ClampMin = "0.001", UIMin = "0.005", UIMax = "0.1"))
    float SubstepTime;
    //[约束迭代次数]//
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Soft Body Simulation", meta = (ClampMin = "1", ClampMax = "64"))
    int32 ConstraintIterations;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Soft Body Simulation", meta = (ClampMin = "0.01", ClampMax = "1000.0"))
    float ParticleMass;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Soft Body Simulation", meta = (ClampMin = "0.01", ClampMax = "1000.0"))
    float ParticleRadius;
    // XPBD 拉伸刚度 (单位: Newton/Meter，建议值 1000 ~ 10000)

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "XPBD Settings")
    float XPBD_StretchStiffness = 2000.0f;

    // XPBD 弯曲刚度 (建议值 100 ~ 500)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "XPBD Settings")
    float XPBD_BendingStiffness = 100.0f;
    
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Soft Body Simulation")
    FVector ClothForce;
    
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HighRes Physics", meta = (ClampMin = "1", ClampMax = "64"))
    int32 HighResIterations = 16;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Soft Body Simulation")
    float ClothGravityScale;

    // 默认给 1.0f
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HighRes Physics", meta = (ClampMin = "0.01", ClampMax = "1.0"))
    float HighResStiffness = 1.0f; 

    // 晃动阻尼：控制高模次级晃动平息的速度。值越接近 1，晃动持续时间越长
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HighRes Physics", meta = (ClampMin = "0.8", ClampMax = "1.0"))
    float HighResDamping = 0.98f;
    
    // --- 碰撞 ---
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Soft Body Interaction")
    bool bEnableHapticInteraction;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Soft Body Simulation")
    bool bWorldCollision;

    // 是否使用全局距离场 (GDF) 做静态网格体碰撞 (GPU 路径)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Soft Body Simulation")
    bool bUseDistanceFieldCollision = false;

    // GDF 碰撞的接触容差缓冲带 (cm)。把表面"往外扩"这段距离，
    // 粒子在缓冲带内就被稳妥推出，避免在表面边界上来回震荡/穿模。
    // 值太小会抖，值太大会让软体"悬浮"在表面外。默认 1cm。
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Soft Body Simulation", meta = (ClampMin = "0.0", ClampMax = "20.0", EditCondition = "bUseDistanceFieldCollision"))
    float GDFSkinOffset = 1.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Soft Body Simulation")
    bool bUseGPU = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Soft Body Simulation", meta = (ClampMin = "0.0", ClampMax = "1.0", EditCondition = "bWorldCollision"))
    float CollisionFriction;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Soft Body Simulation", meta = (ClampMin = "0.0", ClampMax = "2.0", EditCondition = "bWorldCollision"))
    float CollisionRestitution;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Soft Body Simulation")
    bool bUse_BendingForce;
    
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Soft Body Simulation")
    bool bUse_DihedralBending = false; // 二面角开关

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "XPBD Settings", meta = (EditCondition = "bUse_DihedralBending"))
    float XPBD_DihedralStiffness = 500.0f; // 二面角刚度
    
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Soft Body Simulation")
    bool bUse_InternalConstraints;
    
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "XPBD Settings")
    float XPBD_InternalStiffness = 1000.0f;
    
    // --- 体积保持 (散度公式) ---
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Soft Body Simulation")
    bool bUse_VolumePressureForce;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Soft Body Simulation", meta = (EditCondition = "bUse_VolumePressureForce"))
    float VolPressure_Coefficient;
    
    //触觉反馈硬度 (0.0 ~ 1.0)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Soft Body Interaction")
    float HapticFeedbackStiffness = 0.5f;

    // --- Debug ---
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Soft Body Debug")
    bool bShow_Constraints;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Soft Body Debug")
    bool bShow_Normals;
    
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Soft Body Debug")
    bool bShowSingleVertexConstraints = false;
    
    // 当前选中的代理顶点 ID
    UPROPERTY(VisibleAnywhere, Category = "Soft Body Debug")
    int32 SelectedVertexID = -1;
    
    UFUNCTION(BlueprintCallable, Category = "Soft Body Debug")
    int32 SelectVertexByRay(FVector RayOrigin, FVector RayDirection, float ClickTolerance = 50.0f);
    
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Soft Body Debug")
    bool bShowBindingRegions = false;
    
    // ==========================================
    // GPU 显卡直通渲染 (VAT)
    // ==========================================
    
    // 是否使用纯 GPU 材质驱动高模 (开启后将极大提升帧率，但 CPU 碰撞盒不再跟随高模变形)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Soft Body Rendering")
    bool bUpdateViaGPU = false;

    // 自动生成的实时位置纹理
    UPROPERTY(VisibleAnywhere, Transient, Category = "Soft Body Rendering")
    UTextureRenderTarget2D* PositionRT;

    // 自动生成的实时法线纹理
    UPROPERTY(VisibleAnywhere, Transient, Category = "Soft Body Rendering")
    UTextureRenderTarget2D* NormalRT;
    
    // =========================================================
    // 函数
    // =========================================================
    UFUNCTION(BlueprintCallable, CallInEditor, Category = "Soft Body Setup")
    void BuildClothState();

    UFUNCTION(BlueprintCallable, CallInEditor, Category = "Soft Body Setup")
    void ResetToInitalState();

    UFUNCTION(BlueprintCallable, CallInEditor, Category = "Soft Body Debug")
    void DBG_ShowParticles() const;

    UFUNCTION(BlueprintCallable, CallInEditor, Category = "Soft Body Debug")
    void DebugDrawBindings();
    
    void FixCantileverEdge(float Tolerance = 2.0f, int32 Axis = 0);
public:
    // =========================================================
    // 核心数据 (Core Data)
    // =========================================================

    // 1. 物理粒子数组 (当前状态)
    TArray<FMySoftBodyParticle> ProxyParticles;
    TArray<FMySoftBodyParticle> HighResParticles;
    // 2. 原始位置数组 (Rest Pose)
    // 用于计算弹簧原长和重置。替代了原来的 smData.Pos
    TArray<FVector> ProxyRestPositions;
    TArray<FVector> HighResRestPositions;
    
    // 3. 三角形索引缓存 (用于散度公式，对应 StructuredBuffer<Triangle>)
    TArray<FSoftBodyTriangle> SimulationTriangles;

    // 4. 约束列表 (纯索引，对应 StructuredBuffer<Constraint>)
    TArray<FMySoftBodyConstraint> Constraints;
    // 弯曲约束列表 (存的是对角线)
    TArray<FMySoftBodyConstraint> BendingConstraints;
    // 存储二面角约束的数组
    TArray<FMySoftBodyDihedralConstraint> DihedralConstraints;
    // 内部支撑约束列表
    TArray<FMySoftBodyConstraint> InternalConstraints;
    // 5. 拓扑邻接表
    // VertexToTriangleMap[VertexID] = { TriID_A, TriID_B, ... }
    // 使用 TArray 嵌套，自动管理内存，无需 new/delete
    TArray<TArray<int32>> VertexToTriangleMap;
    // 6. 缓存计算出的切线
    TArray<FVector> CurrentTangents;
    // 7. 缓存计算出的法线
    TArray<FVector> CurrentNormals;
    // 缓存数据
    int32 ProxyParticleCount;
    int32 HighResParticleCount;
    int32 ProxyTriCount;
    float ParticleRadiusSq;
    void CollideWithHapticStylus(const FVector& V_Start, const FVector& V_End, float Radius);
    
    // 配置
    UPROPERTY(EditAnywhere, Category = "Soft Body Proxy")
    int32 TargetProxyTriangleCount = 3000; // 目标面数

    // 数据
    FDynamicMesh3 RenderMesh; // 初始模型 (HighRes)
    FDynamicMesh3 ProxyMesh;  // 简化模型 (LowRes)
    TArray<int32> ProxyDebugIndices;// 缓存 Proxy 的三角形索引，用于 DrawDebugMesh
    
    TArray<FVector>RenderPositions;
    TArray<FVector>RenderNormals;
    TArray<FVector>ProxyPositions;
    TArray<FVector>ProxyNormals;
    
    // HighResVertices[i] 对应哪个 Proxy 位置
    TArray<FScaffoldBinding> ScaffoldBindings;
    
    // 核心开关：是否使用低模代理
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Soft Body Proxy")
    bool bUseProxy = false;

    // 调试显示：是否显示代理网格的顶点
    UPROPERTY(EditAnywhere, Category = "Soft Body Debug")
    bool bShowProxyNodes = false;
protected:
    // [新增] 游戏开始时调用
    virtual void BeginPlay() override;

private:
    // =========================================================
    // 内部逻辑
    // =========================================================
    void BuildTopologyCache(const UE::Geometry::FDynamicMesh3& Mesh);
    void BuildConstraints();
    void BuildBendingConstraints(const UE::Geometry::FDynamicMesh3& Mesh);   // 构建弯曲约束
    void BuildDihedralConstraints(const UE::Geometry::FDynamicMesh3& Mesh);  // 构建二面角约束
    void BuildInternalConstraints(const UE::Geometry::FDynamicMesh3& Mesh);  // 构建内部形状保持约束 (ISPC)
    void SubstepSolve();
    void Integrate(float InSubstepTime);
    void EvalConstraints();
    void UpdateVelocities(float InSubstepTime);
    void UpdateTangents();
    void UpdateNormals();
    // [新增] 缓存当前的棍子状态 (每帧更新)
        FVector CachedStickStart;
        FVector CachedStickEnd;
        FVector CachedStickVelStart;
        FVector CachedStickVelEnd;
        float CachedStickRadius;
        bool bHasStickInput = false; // 标记是否有棍子输入
    // [新增] 记录 Proxy (代理点) 的位置
    FVector CurrentProxyPos;

    // [新增] 标记这一帧是否与触觉设备发生了碰撞
    bool bHapticCollisionThisFrame = false;
    bool bUseBendingForce = true;

    // [拆分 1] 只修正位置 (用于迭代循环)
    static void SolveStickPosition(
        FMySoftBodyParticle& Pt,
        const FVector& Start, const FVector& End, float Radius);

    // [拆分 2] 只修正速度/摩擦 (用于循环结束后)
    static void SolveStickVelocity(
        FMySoftBodyParticle& Pt,
        const FVector& Start, const FVector& End,
        const FVector& V_Start, const FVector& V_End,
        float Radius, float Friction, float Restitution, float dt);

    // 静态求解函数：不依赖类实例，方便移植到 Shader
    static void SolveDistanceConstraintXPBD(TArray<FMySoftBodyParticle>& InParticles, int32 IdxA, int32 IdxB, float RestLen, float Stiffness, float dt,float& AccLambda);

    void ClothCollisionWorld();

    void TickUpdateCloth();
    // void UpdateNormals(...) // 已移除，改用 DynamicMesh 内置计算

    // --- 体积保持 (散度公式版) ---
    void UpdateWeightedNormals();
    float CalcClothVolume(); // 基于散度定理
    void VolumePreservation();
    void VolumePressureForce(int32 mode);

    INLINE float SquareDist(const FVector& A, const FVector& B)
    {
        return FVector::DistSquared(A, B);
    }

    float restVolume, curVolume, deltaVolume;
    float Dt, At, St;
    bool clothStateExists;


    // =========================================================
    // GPU 资源
    // =========================================================

    // Pooled Buffers (RDG 友好)
    TRefCountPtr<FRDGPooledBuffer> ProxyParticlePooledBuffer;   // 低模粒子的 GPU Buffer
    TRefCountPtr<FRDGPooledBuffer> HighResParticlePooledBuffer;// 高模粒子的 GPU Buffer
    TRefCountPtr<FRDGPooledBuffer> HighResRestPosPooledBuffer; // 高模粒子的初始位置GPU Buffer
    TRefCountPtr<FRDGPooledBuffer> ConstraintPooledBuffer;    // FGPUConstraint
    TRefCountPtr<FRDGPooledBuffer> LambdaPooledBuffer;        // FGPULambda
    TRefCountPtr<FRDGPooledBuffer> NeighborInfoPooledBuffer;  // FGPUNeighborInfo
    // 表面约束 (距离+弯曲) 的 Buffer
    TRefCountPtr<FRDGPooledBuffer> SurfaceNeighborInfoPooledBuffer;
    TRefCountPtr<FRDGPooledBuffer> SurfaceAdjacencyPooledBuffer;
    // 内部约束 (支撑) 的 Buffer
    TRefCountPtr<FRDGPooledBuffer> InternalNeighborInfoPooledBuffer;
    TRefCountPtr<FRDGPooledBuffer> InternalAdjacencyPooledBuffer;
    
    TRefCountPtr<FRDGPooledBuffer> HighResPositionOutputPooledBuffer;// Position 的输出 Buffer
    TRefCountPtr<FRDGPooledBuffer> ProxyPositionOutputPooledBuffer;// Position 的输出 Buffer
    
    TRefCountPtr<FRDGPooledBuffer> VertexTriangleOffsetsPooledBuffer;  // 每个顶点的起始索引
    TRefCountPtr<FRDGPooledBuffer> VertexTriangleCountsPooledBuffer;   // 每个顶点的三角形数量
    TRefCountPtr<FRDGPooledBuffer> VertexTriangleIndicesPooledBuffer;  // 扁平化的三角形索引
    // GPU 体积约束资源
    TRefCountPtr<FRDGPooledBuffer> TrianglePooledBuffer;
    TRefCountPtr<FRDGPooledBuffer> WeightedNormalsPooledBuffer;
    TRefCountPtr<FRDGPooledBuffer> VolumePooledBuffer;
    TRefCountPtr<FRDGPooledBuffer> NormalOutputPooledBuffer;  // 法线回读 Buffer
    TRefCountPtr<FRDGPooledBuffer> VolumeOutputPooledBuffer; // 体积回读
    
    // 高模约束与邻接表的 RDG 缓冲指针
    TRefCountPtr<FRDGPooledBuffer> HighResTrianglePooledBuffer;
    TRefCountPtr<FRDGPooledBuffer> HighResWeightedNormalsPooledBuffer;
    TRefCountPtr<FRDGPooledBuffer> HighResNormalOutputPooledBuffer;
    TRefCountPtr<FRDGPooledBuffer> HighResVtxTriOffsetsPooledBuffer;
    TRefCountPtr<FRDGPooledBuffer> HighResVtxTriCountsPooledBuffer;
    TRefCountPtr<FRDGPooledBuffer> HighResVtxTriIndicesPooledBuffer;
    
    // 映射表的 GPU Buffer (用于在显存中同步低模到高模)
    TRefCountPtr<FRDGPooledBuffer> ScaffoldBindingsPooledBuffer;
    TRefCountPtr<FRDGPooledBuffer> ProxyTrianglePooledBuffer;
    
    bool bGPUResourcesInitialized = false;

    // --- 全局距离场 (GDF) 碰撞 ---
    // 视图扩展每帧提取稳定 pooled texture 缓存，独立模拟 graph 采样
    TSharedPtr<ISceneViewExtension, ESPMode::ThreadSafe> GDFViewExtension;
    TSharedPtr<FSoftBodyGDFCacheData, ESPMode::ThreadSafe> GDFCache;

    // --- 二面角 GPU 资源 ---
    TRefCountPtr<FRDGPooledBuffer> DihedralConstraintPooledBuffer;
    TRefCountPtr<FRDGPooledBuffer> DihedralLambdaPooledBuffer;
    TRefCountPtr<FRDGPooledBuffer> DihedralNeighborInfoPooledBuffer;
    TRefCountPtr<FRDGPooledBuffer> DihedralAdjacencyPooledBuffer;

    // 二面角邻接表数据 (CPU端缓存)
    TArray<uint32> DihedralAdjacencyIndices;
    TArray<uint32> DihedralNeighborOffsets;
    TArray<uint32> DihedralNeighborCounts;
    
    // 表面邻接表数据
    TArray<uint32> SurfaceAdjacencyIndices;
    TArray<uint32> SurfaceNeighborOffsets;
    TArray<uint32> SurfaceNeighborCounts;

    // 内部邻接表数据
    TArray<uint32> InternalAdjacencyIndices;
    TArray<uint32> InternalNeighborOffsets;
    TArray<uint32> InternalNeighborCounts;
    
    TArray<uint32> VertexTriangleOffsets;   // CPU 端缓存
    TArray<uint32> VertexTriangleCounts;
    TArray<uint32> VertexTriangleIndices;
    
    // 高模法线回读数组
    TArray<FVector> HighResCurrentNormals;
    // [新增] 缓存高模每个顶点对应的法线 Element IDs (空间换时间)
    TArray<TArray<int32>> CachedHighResNormalElements;
    
    // 高模三角形缓存
    TArray<FSoftBodyTriangle> HighResSimulationTriangles;
    TArray<uint32> HighResVertexTriangleOffsets;
    TArray<uint32> HighResVertexTriangleCounts;
    TArray<uint32> HighResVertexTriangleIndices;
    
    
    // 双缓冲 异步回读
    // 初始模型顶点回读
    TArray<FVector3f> CachedGPUPositions;      // 渲染线程写入
    TArray<FVector3f> ReadyGPUPositions;       // 游戏线程读取
    FRHIGPUBufferReadback* ParticleReadback = nullptr;
    bool bReadbackPending = false;
    std::atomic<bool> bNewDataReady{false};
    // 简化模型顶点回读
    TArray<FVector3f> CachedProxyPositions;
    TArray<FVector3f> ReadyProxyPositions;
    FRHIGPUBufferReadback* ProxyReadback = nullptr;
    bool bProxyReadbackPending = false;
    std::atomic<bool> bNewProxyDataReady{false};
    // 低模法线回读
    TArray<FVector3f> CachedGPUNormals;
    TArray<FVector3f> ReadyGPUNormals;
    FRHIGPUBufferReadback* NormalReadback = nullptr;
    bool bNormalReadbackPending = false;
    std::atomic<bool> bNewNormalDataReady{false};
    // 体积回读
    TArray<int32> CachedGPUVolume;
    TArray<int32> ReadyGPUVolume;
    FRHIGPUBufferReadback* VolumeReadback = nullptr;
    bool bVolumeReadbackPending = false;
    std::atomic<bool> bNewVolumeDataReady{false};
    // 高模回读控制变量
    TArray<FVector3f> CachedHighResGPUNormals;
    TArray<FVector3f> ReadyHighResGPUNormals;
    FRHIGPUBufferReadback* HighResNormalReadback = nullptr;
    bool bHighResNormalReadbackPending = false;
    std::atomic<bool> bNewHighResNormalDataReady{false};
    
    // GPU 函数
    void InitGPUResources();
    void ReleaseGPUResources();
    void BuildAdjacencyData();            // 构建邻接表
    //void BuildHighResAdjacencyData();     // 构建高模邻接表函数
    void UploadDataToGPU();               // 上传数据到 GPU
    void DispatchGPUCompute(bool bIsLastSubstep=true);            // 调度 GPU 计算
    void ReadbackGPUPositions();          // 读回位置数据
    //void BuildHighResConstraints(const UE::Geometry::FDynamicMesh3& Mesh);
    void BuildHighResTopologyCache(const UE::Geometry::FDynamicMesh3& Mesh);
    
    void GenerateProxyAndMapping(); // 生成并建立关系
};
