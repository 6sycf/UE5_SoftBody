// Fill out your copyright notice in the Description page of Project Settings.


#include "OpenHapticsComponent.h"

// Sets default values for this component's properties
UOpenHapticsComponent::UOpenHapticsComponent()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;
	// 初始化数据
	bSharedBtn1 = false;
	bSharedBtn2 = false;
	// 创建可视化网格组件
	VisualMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("HapticDeviceMesh"));
	// 默认不碰撞 (物理逻辑由 PBD 处理，这个只是用来看的)
	VisualMesh->SetCollisionProfileName(TEXT("NoCollision"));
	VisualMesh->SetGenerateOverlapEvents(false);
	// 默认偏移 (你可以根据模型调整)
	VisualOffset = FTransform::Identity;
	
	Tool_Visual.Radius = StylusRadius;
	Tool_Device.Radius = StylusRadius;
	Tool_Force.Radius  = StylusRadius;
	// ...
}

UOpenHapticsComponent::~UOpenHapticsComponent()
{
	// 确保析构时彻底关闭设备
	if (bIsDeviceValid)
	{
		hdStopScheduler();//停止 1000Hz 的调度线程
		if (hUpdateCallback != HD_INVALID_HANDLE)
		{
			hdUnschedule(hUpdateCallback);//把我们的函数从调度列表中移除
		}
		hdDisableDevice(hHD);//关闭设备句柄
		bIsDeviceValid = false;//防止重复销毁 
	}
}

// Called when the game starts
void UOpenHapticsComponent::BeginPlay()
{
	Super::BeginPlay();
	
	// 1. 初始化设备
	HDErrorInfo error;
	// hdInitDevice: OpenHaptics API
	// 尝试连接默认的触觉设备 (HD_DEFAULT_DEVICE)。
	// 返回一个句柄 (HHD)，类似于 "设备ID"。如果连接失败，返回 HD_INVALID_HANDLE。
	hHD = hdInitDevice(HD_DEFAULT_DEVICE);
	// 检查错误：如果句柄无效或有其他错误
	if (HD_DEVICE_ERROR(error = hdGetError()))
	{
		UE_LOG(LogTemp, Error, TEXT("Haptics: Failed to init device. Error Code: %X"), error.errorCode);
		bIsDeviceValid = false;
		return;
	}
	// 标记初始化成功
	bIsDeviceValid = true;

	// 2. 开启力反馈输出
	// hdEnable: OpenHaptics API
	// 默认情况下设备可能只是作为“输入设备”(只读位置)。
	// 必须显式开启 HD_FORCE_OUTPUT，才能向电机发送力指令。
	hdEnable(HD_FORCE_OUTPUT);

	// 3. 注册异步回调 (1000Hz 循环)
	// 将 "this" 指针传给回调函数，以便访问成员变量
	hUpdateCallback = hdScheduleAsynchronous(DeviceLoopCallback, this, HD_DEFAULT_SCHEDULER_PRIORITY);

	// 4. 启动调度器
	// 正式启动那个 1000Hz 的后台线程。
	hdStartScheduler();
	// 最后检查一下调度器是否启动成功
	if (HD_DEVICE_ERROR(error = hdGetError()))
	{
		UE_LOG(LogTemp, Error, TEXT("Haptics: Failed to start scheduler."));
	}
	else
	{
		UE_LOG(LogTemp, Log, TEXT("Haptics: Device Initialized and Scheduler Started!"));
	}
}

void UOpenHapticsComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	Super::EndPlay(EndPlayReason);

	if (bIsDeviceValid)
	{
		hdStopScheduler();
		hdUnschedule(hUpdateCallback);
		hdDisableDevice(hHD);
		bIsDeviceValid = false;
		UE_LOG(LogTemp, Log, TEXT("Haptics: Device Stopped."));
	}
}
void UOpenHapticsComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
    
    if (!bIsDeviceValid) return;

	// 1. 【读取】从触觉线程获取最新的硬件位置
	FHapticToolState TargetState;
	{
    	FScopeLock Lock(&DataGuard);
    	TargetState = Shared_FromHaptic_State;
	}
	Tool_Visual.Radius = StylusRadius;
	
	// 2. 【计算】使用插值算法计算这一帧的新位置
	FVector DesiredStartPos = FMath::VInterpTo(Tool_Visual.StartPos, TargetState.StartPos, DeltaTime, VisualFollowSpeed);
	FVector DesiredEndPos   = FMath::VInterpTo(Tool_Visual.EndPos,   TargetState.EndPos,   DeltaTime, VisualFollowSpeed);
	// B. 计算这一帧想移动的向量
	FVector DeltaStart = DesiredStartPos - Tool_Visual.StartPos;
	FVector DeltaEnd   = DesiredEndPos   - Tool_Visual.EndPos;
	float DynamicMaxSpeedCM = MaxPhysicalVelocity * ScaleMMToCM;

	// 2. 转换为这一帧的最大位移 (cm)
	float MaxStepDist = DynamicMaxSpeedCM * DeltaTime;

	// 3. 安全钳制：防止 Scale 极小或 DeltaTime 极小时出问题
	// 至少允许移动一个微小的距离 (比如 0.01cm)，避免卡死
	MaxStepDist = FMath::Max(MaxStepDist, 0.01f);

	// D. 钳制移动距离 (Clamp)
	if (DeltaStart.SizeSquared() > MaxStepDist * MaxStepDist)
	{
		DeltaStart = DeltaStart.GetSafeNormal() * MaxStepDist;
	}
    
	if (DeltaEnd.SizeSquared() > MaxStepDist * MaxStepDist)
	{
		DeltaEnd = DeltaEnd.GetSafeNormal() * MaxStepDist;
	}

	// E. 应用最终位置
	FVector NewStartPos = Tool_Visual.StartPos + DeltaStart;
	FVector NewEndPos   = Tool_Visual.EndPos   + DeltaEnd;
	
	// 3. 【核心修复】计算精确的物理速度 (v = dx / dt)
	if (DeltaTime > 1e-6f)
	{
		Tool_Visual.StartVel = (NewStartPos - Tool_Visual.StartPos) / DeltaTime;
		Tool_Visual.EndVel   = (NewEndPos   - Tool_Visual.EndPos)   / DeltaTime;
	}
	else
	{
		Tool_Visual.StartVel = FVector::ZeroVector;
		Tool_Visual.EndVel   = FVector::ZeroVector;
	}
	// 4. 【应用】更新 Tool_Visual 的位置
	Tool_Visual.StartPos = NewStartPos;
	Tool_Visual.EndPos   = NewEndPos;
	
	// 5. 【写入】将带有速度信息的 Visual 状态发回给触觉线程
	{
    	FScopeLock Lock(&DataGuard);
    	Shared_FromGame_VisualState = Tool_Visual;
	}
	
	// 4. 【可视化】
	if (VisualMesh)
	{
		// VisualMesh 跟随 Tool_Visual
		FTransform CompTransform = GetComponentTransform();
		FVector WStart = CompTransform.TransformPosition(Tool_Visual.StartPos);
		FVector WEnd   = CompTransform.TransformPosition(Tool_Visual.EndPos);
		FVector Center = (WStart + WEnd) * 0.5f;
		FVector Dir    = (WEnd - WStart).GetSafeNormal();
		if (Dir.IsZero()) Dir = FVector::UpVector;
		FQuat Rot = FQuat::FindBetweenNormals(FVector::UpVector, Dir);
        
		VisualMesh->SetWorldTransform(VisualOffset * FTransform(Rot, Center));
	}

	if (bShowDebugMesh)
	{
		FTransform CompTransform = GetComponentTransform();

		// 定义一个 Lambda 函数来复用绘制逻辑
		// 参数: State(工具状态), Color(颜色)
		auto DrawToolCapsule = [&](const FHapticToolState& State, FColor Color)
		{
			// 1. 转世界坐标
			FVector WStart = CompTransform.TransformPosition(State.StartPos);
			FVector WEnd   = CompTransform.TransformPosition(State.EndPos);
            
			// 2. 计算几何中心
			FVector Center = (WStart + WEnd) * 0.5f;

			// 3. 计算旋转 (胶囊体默认是竖直的，需要转到棍子方向)
			FVector Dir = (WEnd - WStart).GetSafeNormal();
			if (Dir.IsZero()) Dir = FVector::UpVector;
			FQuat Rot = FQuat::FindBetweenNormals(FVector::UpVector, Dir);

			// 4. 计算半高 (半长 + 半径)
			float HalfHeight = (WEnd - WStart).Size() * 0.5f + State.Radius;

			// 5. 绘制
			DrawDebugCapsule(
				GetWorld(), 
				Center, 
				HalfHeight, 
				State.Radius, 
				Rot, 
				Color, 
				false, -1.0f, 0, 1.0f // 1.0f 是线条厚度
			);
		};

		// --- 绘制 图形工具 (Visual Tool) ---
		// 颜色：红色（Red）
		// 代表：物理世界中的代理点，会被软体阻挡，有平滑效果
		DrawToolCapsule(Tool_Visual, FColor::Red);

		// --- 绘制 力反馈工具 (Force Tool / Hardware) ---
		// 颜色：绿色 (Green)
		// 代表：你手里的真实设备位置，无视碰撞，绝对跟随硬件
		// 注意：TargetState 是你在函数开头读取的 Shared_FromHaptic_State
		DrawToolCapsule(TargetState, FColor::Green);
	}
}
// =========================================================
// 触觉线程回调 (运行在独立的 1000Hz 线程中)
// =========================================================
HDCallbackCode HDCALLBACK UOpenHapticsComponent::DeviceLoopCallback(void* pUserData)
{
    UOpenHapticsComponent* Comp = static_cast<UOpenHapticsComponent*>(pUserData);
    if (!Comp) return HD_CALLBACK_DONE;

    hdBeginFrame(hdGetCurrentDevice());

    // -----------------------------------------------------
    // A. 获取硬件信息 -> 更新 [力反馈工具 Tool_Force]
    // -----------------------------------------------------
	double transform[16];
	hdGetDoublev(HD_CURRENT_TRANSFORM, transform);
	int buttons = 0;
	hdGetIntegerv(HD_CURRENT_BUTTONS, &buttons);
	
	// 解析坐标 (Local mm)
	double TipX = transform[12]; double TipY = transform[13]; double TipZ = transform[14];
	double DirX = transform[8];  double DirY = transform[9];  double DirZ = transform[10];

    // 转换到 UE 坐标系 (Local cm)
	float S = Comp->ScaleMMToCM;
    FVector NewStart(-TipZ * S, TipX * S, TipY * S); // UE Local Pos
    
    // 计算笔尾 (基于方向)
    double Len_mm = Comp->StylusLength / Comp->ScaleMMToCM; // convert cm back to raw scale unit if needed, or just math directly
    // 简单起见，直接在 UE 空间算方向向量
    FVector UE_Dir(-DirZ, DirX, DirY); // 旋转 basis
    FVector NewEnd = NewStart + (UE_Dir * Comp->StylusLength);

    // 计算速度 (简单差分)
    FVector NewVel = (NewStart - Comp->Tool_Force.StartPos) / 0.001f; // 假设1ms

    // 更新 Haptic 线程本地的 Tool_Force
    Comp->Tool_Force.StartPos = NewStart;
    Comp->Tool_Force.EndPos   = NewEnd;
    Comp->Tool_Force.StartVel = NewVel;
    Comp->Tool_Force.Radius   = Comp->StylusRadius;

    // -----------------------------------------------------
    // B. 数据交换与力计算
    // -----------------------------------------------------
    FVector VisualPos = FVector::ZeroVector;
    FVector ExternalForce = FVector::ZeroVector;
    
	{
    	FScopeLock Lock(&Comp->DataGuard);

        // 1. [上传] 把当前的 Tool_Force 发送给 GameThread
        // 这样 GameThread 里的 Tool_Visual 下一帧才能知道要往哪走
        Comp->Shared_FromHaptic_State = Comp->Tool_Force;
        
        // 更新按钮状态
        Comp->bSharedBtn1 = (buttons & HD_DEVICE_BUTTON_1) != 0;
        Comp->bSharedBtn2 = (buttons & HD_DEVICE_BUTTON_2) != 0;

        // 2. [下载] 获取 GameThread 传下来的 Tool_Visual 位置
        // 这是上一帧游戏线程计算完物理后的结果
        VisualPos = Comp->Shared_FromGame_VisualState.StartPos;
        ExternalForce = Comp->SharedExternalForce;
	}
	float CurrentStiffnessMult = 0.0f;
	{
    	FScopeLock Lock(&Comp->DataGuard);
    	// ... (原有读取代码) ...
        
    	// [新增] 读取刚度倍率
    	CurrentStiffnessMult = Comp->Shared_StiffnessMultiplier;
	}
	
    // -----------------------------------------------------
    // C. 核心力计算：F = k * (Visual - Force)
    // -----------------------------------------------------
    // 如果这是第一帧，VisualPos 可能是 0，避免猛烈弹跳
    if (VisualPos.IsZero()) VisualPos = NewStart;

    // 计算弹簧力 (UE 坐标系)
    // 方向：从 Tool_Force (当前手的位置) 指向 Tool_Visual (被墙挡住的位置)
    // 效果：把手拉向那个被挡住的位置 -> 产生阻力感
	FVector Diff = VisualPos - Comp->Tool_Force.StartPos;
	float EffectiveStiffness = Comp->VirtualSpringStiffness * (CurrentStiffnessMult + 0.01f);
	
    FVector SpringForce = Diff * EffectiveStiffness;
	
	//FVector DampingForce = -DeviceVelocity_UE * Comp->HapticDamping * 1000.0f;
	
    // 叠加外部力 (如重力、特效力)
    FVector TotalForce = SpringForce + ExternalForce;

    // 钳制最大力 (安全保护)
    float MaxForce = 10.0f; // 5 牛顿
    if (TotalForce.SizeSquared() > MaxForce * MaxForce)
    {
        TotalForce = TotalForce.GetSafeNormal() * MaxForce;
    }

    // -----------------------------------------------------
    // D. 输出力 (转换回 OpenHaptics 坐标系)
    // -----------------------------------------------------
    // UE: X(Front), Y(Right), Z(Up)
    // OH: X(Right), Y(Up),    Z(Back/User)
    // Mapping: OH.X = UE.Y, OH.Y = UE.Z, OH.Z = -UE.X
	double forceOutput[3];
	forceOutput[0] = TotalForce.Y * Comp->ForceMultiplier;
	forceOutput[1] = TotalForce.Z * Comp->ForceMultiplier;
	forceOutput[2] = -TotalForce.X * Comp->ForceMultiplier;

	hdSetDoublev(HD_CURRENT_FORCE, forceOutput);

	hdEndFrame(hdGetCurrentDevice());
	return HD_CALLBACK_CONTINUE;
}

// [新增函数] 供物理引擎调用
void UOpenHapticsComponent::SetCollisionState(bool bIsColliding)
{
	// 目标权重：撞到了就是 1.0，没撞到就是 0.0
	float TargetWeight = bIsColliding ? 1.0f : 0.0f;
    
	// 使用 FMath::FInterpTo 进行平滑过渡 (防止力突变)
	// 这里的 DeltaTime 我们直接取 World 的，或者存一个成员变量
	float Dt = GetWorld()->GetDeltaSeconds();
	CollisionBlendWeight = FMath::FInterpTo(CollisionBlendWeight, TargetWeight, Dt, StiffnessBlendSpeed);

	// 更新共享数据
	{
		FScopeLock Lock(&DataGuard);
		Shared_StiffnessMultiplier = CollisionBlendWeight;
	}
}

bool UOpenHapticsComponent::IsButton1Pressed() const
{
	FScopeLock Lock(&DataGuard);
	return bSharedBtn1;
}

bool UOpenHapticsComponent::IsButton2Pressed() const
{
	FScopeLock Lock(&DataGuard);
	return bSharedBtn2;
}


FTransform UOpenHapticsComponent::GetDeviceTransform() const
{
	// 获取当前组件的世界变换
	FTransform CompTransform = GetComponentTransform();
	FVector LocalPos = GetDevicePosition();
    
	// 将设备的局部位置转为世界位置
	FVector WorldPos = CompTransform.TransformPosition(LocalPos);
    
	// 这里暂时没处理旋转 (OpenHaptics 也可以读取 Gimbal 角度，但为了简化先不写)
	return FTransform(CompTransform.GetRotation(), WorldPos, CompTransform.GetScale3D());
}
FHapticToolState UOpenHapticsComponent::GetHapticToolState() const
{
	return Tool_Visual;
}

/** 获取设备在 UE 坐标系中的位置 (通常是相对于组件原点的局部坐标) */
FVector UOpenHapticsComponent::GetDevicePosition() const
{
	// 加锁：防止在读取时，触觉线程正好在写入新的位置
	FScopeLock Lock(&DataGuard);
    
	// 返回最新的硬件位置 (Force Tool 的位置)
	// 注意：这是 UE 坐标系下的 Local Position (cm)
	return Shared_FromHaptic_State.StartPos;
}
void UOpenHapticsComponent::UpdateVisualStateFromPhysics(const FVector& NewStart, const FVector& NewEnd)
{
	// 1. 更新游戏线程本地的图形工具
	Tool_Visual.StartPos = NewStart;
	Tool_Visual.EndPos   = NewEnd;
	{
		FScopeLock Lock(&DataGuard);
		Shared_FromGame_VisualState = Tool_Visual;
	}
}