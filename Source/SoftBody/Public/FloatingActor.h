#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "FloatingActor.generated.h"

/**
 * 上下浮动的测试 Actor。
 *
 * 直接拖进场景即可（bAutoStart = true 会在 BeginPlay 自动开始浮动），
 * 也可以在蓝图中调用 StartFloating / StopFloating / SetFloatingEnabled 手动控制。
 *
 * 用途：配合软体的全局距离场 (GDF) / Sweep 碰撞，验证移动的静态网格体
 * 能否被软体实时碰撞到。
 */
UCLASS()
class SOFTBODY_API AFloatingActor : public AActor
{
	GENERATED_BODY()

public:
	AFloatingActor();

	virtual void Tick(float DeltaTime) override;

	// =========================================================
	// 蓝图节点
	// =========================================================

	// 开始上下浮动（重新从当前相位起点开始计时）
	UFUNCTION(BlueprintCallable, Category = "Floating")
	void StartFloating();

	// 停止浮动（停在当前位置，不归位）
	UFUNCTION(BlueprintCallable, Category = "Floating")
	void StopFloating();

	// 直接开关浮动
	UFUNCTION(BlueprintCallable, Category = "Floating")
	void SetFloatingEnabled(bool bEnabled);

	// 立即归位到 BeginPlay 时的初始位置
	UFUNCTION(BlueprintCallable, Category = "Floating")
	void ResetToInitialLocation();

protected:
	virtual void BeginPlay() override;

public:
	// =========================================================
	// 可编辑参数
	// =========================================================

	// 显示用的静态网格组件（在 Details 面板里给它指定一个 StaticMesh）
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Floating")
	UStaticMeshComponent* MeshComponent;

	// 是否在 BeginPlay 时自动开始浮动
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Floating")
	bool bAutoStart = true;

	// 浮动幅度 (cm)，即上下各偏离中心的最大距离
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Floating", meta = (ClampMin = "0.0"))
	float FloatAmplitude = 20.0f;

	// 浮动频率 (Hz)，每秒往返的次数
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Floating", meta = (ClampMin = "0.0"))
	float FloatFrequency = 1.0f;

	// 初始相位 (弧度)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Floating")
	float FloatPhase = 0.0f;

	// 浮动方向（默认 Z 轴上下）
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Floating")
	FVector FloatAxis = FVector(0.0f, 0.0f, 1.0f);

private:
	// 是否正在浮动
	UPROPERTY(VisibleInstanceOnly, BlueprintReadOnly, Category = "Floating", meta = (AllowPrivateAccess = "true"))
	bool bIsFloating = false;

	// 累积时间
	float RunningTime = 0.0f;

	// BeginPlay 时记录的初始位置（浮动的中心基准点）
	FVector InitialLocation = FVector::ZeroVector;
};
