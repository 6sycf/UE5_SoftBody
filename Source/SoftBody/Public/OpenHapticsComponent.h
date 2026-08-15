// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
#include <HD/hd.h>
#include "OpenHapticsComponent.generated.h"

struct FHapticToolState
{
	FVector StartPos = FVector::ZeroVector; // 笔尖
	FVector EndPos = FVector::ZeroVector;   // 笔尾
	FVector StartVel = FVector::ZeroVector;
	FVector EndVel = FVector::ZeroVector;
	float Radius = 1.0f;
};

UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class SOFTBODY_API UOpenHapticsComponent : public USceneComponent
{
	GENERATED_BODY()

public:	
	// Sets default values for this component's properties
	UOpenHapticsComponent();
	virtual ~UOpenHapticsComponent() override;
	/** * [双向耦合接口]
	 * 当软体物理引擎检测到碰撞并把图形工具(Visual Tool)推开后，
	 * 调用此函数将修正后的位置写回组件。
	 * 参数均为 Local Space (相对于组件的坐标) 
	 */
	void UpdateVisualStateFromPhysics(const FVector& NewStart, const FVector& NewEnd);
protected:
	// Called when the game starts
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	
public:	
	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	// --- 蓝图接口 (Getter/Setter) ---

	// 棍子长度 (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Haptics Settings")
	float StylusLength = 100.0f;

	// 棍子半径 (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Haptics Settings")
	float StylusRadius = 20.0f;

	// 坐标转换比例：OpenHaptics(mm) -> Unreal(cm)
	// 默认 0.1 (1mm = 0.1cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Haptics Settings")
	float ScaleMMToCM = 0.1f;
    
	// 输出的总力反馈的比例
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Haptics Settings")
	float ForceMultiplier = 0.1f;

	// 虚拟弹簧刚度 (Visual 和 Force 工具之间的拉力)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Haptics Settings")
	float VirtualSpringStiffness = 0.5f;

	// 图形工具跟随硬件的速度，值越大，跟得越紧（越快）；值越小，拖尾感越强（越慢）推荐范围：5.0 ~ 20.0
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Haptics Settings")
	float VisualFollowSpeed = 20.0f;

	

	// 获取当前的棍子状态 
	FHapticToolState GetHapticToolState() const;

	/** 获取设备在 UE 世界空间中的位置 (已转换坐标系) */
	UFUNCTION(BlueprintCallable, Category = "Haptics")
	FVector GetDevicePosition() const;
	
	/** 获取设备当前的世界变换 (位置+旋转) */
	UFUNCTION(BlueprintCallable, Category = "Haptics")
	FTransform GetDeviceTransform() const;

	

	/** 获取按钮状态 */
	UFUNCTION(BlueprintCallable, Category = "Haptics")
	bool IsButton1Pressed() const;

	UFUNCTION(BlueprintCallable, Category = "Haptics")
	bool IsButton2Pressed() const;

	// --- 调试设置 ---
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Haptics Debug")
	bool bShowDebugMesh = true;

	// 可视化网格组件
	// 这是一个可选的组件，用来显示笔的 3D 模型
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Haptics Visuals")
	UStaticMeshComponent* VisualMesh;

	// 视觉偏移 (Visual Offset)
	// 有时候模型原点不在笔尖，需要手动调一下位置/旋转偏移
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Haptics Visuals")
	FTransform VisualOffset;

	/** 设置是否发生碰撞 (由软体物理引擎每帧调用) */
	void SetCollisionState(bool bIsColliding);
	
protected:
	// [新增] 碰撞混合权重 (0.0 = 空气, 1.0 = 完全接触)
	float CollisionBlendWeight = 0.0f;

	// [新增] 状态切换速度 (值越大，从空气进入软体感觉越硬，但太大会抖动)
	// 建议：10.0f ~ 20.0f
	float StiffnessBlendSpeed = 15.0f;
	
private:
	// --- OpenHaptics 句柄 ---
	HHD hHD = HD_INVALID_HANDLE;
	HDSchedulerHandle hUpdateCallback = HD_INVALID_HANDLE;

	// 三个独立的工具状态
	// 1. 定位工具 (Device Tool):  
	FHapticToolState Tool_Device;

	// 2. 图形工具 (Visual Tool): 对应屏幕上看到的笔，也是传给软体做变形的依据
	FHapticToolState Tool_Visual;

	// 3. 力反馈工具 (Force Tool): 运行在1000Hz线程。直接对应硬件位置，可以穿透模型。
	FHapticToolState Tool_Force;
	
	// --- 线程安全数据 ---
	// 这些数据会被两个线程同时访问，必须加锁保护
	mutable FCriticalSection DataGuard;

	// 用于把 Haptic线程 的最新硬件状态传给 Game线程
	FHapticToolState Shared_FromHaptic_State;
	
	// 用于把 Game线程 计算好的 Visual 位置传回给 Haptic线程 用于算力
	FHapticToolState Shared_FromGame_VisualState;

	// 传给 Haptic 线程的最终刚度倍率
	float Shared_StiffnessMultiplier = 0.0f;
	
	float MaxPhysicalVelocity = 300.0f;
	
	// 按钮状态
	bool bSharedBtn1;
	bool bSharedBtn2;
	FVector SharedExternalForce = FVector::ZeroVector;
	// --- 内部状态 ---
	bool bIsDeviceValid = false;

	// --- 静态回调函数 ---
	// 这是运行在 1000Hz 触觉线程里的函数
	static HDCallbackCode HDCALLBACK DeviceLoopCallback(void* pUserData);
};
