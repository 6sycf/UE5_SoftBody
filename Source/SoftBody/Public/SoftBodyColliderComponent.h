#pragma once

#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
#include "SoftBodyColliderComponent.generated.h"

// 碰撞体形状 (数值顺序必须与 shader 里的 Shape 约定一致: 0=球, 1=盒, 2=胶囊)
UENUM(BlueprintType)
enum class ESoftBodyColliderShape : uint8
{
	Sphere  UMETA(DisplayName = "Sphere"),
	Box     UMETA(DisplayName = "Box"),
	Capsule UMETA(DisplayName = "Capsule")
};

// CPU 侧的世界空间碰撞体快照（软体每帧收集用）
struct FSoftBodyColliderPrimitive
{
	ESoftBodyColliderShape Shape = ESoftBodyColliderShape::Sphere;
	FVector Center = FVector::ZeroVector;       // 球心 / 盒心 / 胶囊中点
	FQuat Rotation = FQuat::Identity;           // 盒朝向 / 胶囊轴向
	FVector HalfExtents = FVector::ZeroVector;  // 盒半尺寸
	float Radius = 10.0f;                       // 球 / 胶囊半径
	float CapsuleHalfHeight = 0.0f;             // 胶囊半长 (沿本地 Z 轴)
	FVector Velocity = FVector::ZeroVector;     // 线速度 (世界)
	float Friction = 0.2f;
	float Restitution = 0.2f;
};

/**
 * 与软体实时交互的动态碰撞体组件。
 *
 * 挂在任意 Actor 上，用球 / 盒 / 胶囊三种基本体之一作为软体 (GPU 路径) 的实时碰撞体。
 * 软体端会自动扫描场景里的该组件，或通过 RegisterCollider 运行时接入。
 * 碰撞为刚性单向：软体变形、碰撞体本身不动。
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class SOFTBODY_API USoftBodyColliderComponent : public USceneComponent
{
	GENERATED_BODY()

public:
	USoftBodyColliderComponent();

	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	// 把当前组件状态转成世界空间碰撞体快照（软体每帧调用）
	void GetWorldCollider(FSoftBodyColliderPrimitive& Out) const;

	// 开/关该碰撞体
	UFUNCTION(BlueprintCallable, Category = "SoftBodyCollider")
	void SetColliderEnabled(bool bEnable);

public:
	// 碰撞体形状
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SoftBodyCollider")
	ESoftBodyColliderShape Shape = ESoftBodyColliderShape::Sphere;

	// 球 / 胶囊半径 (cm)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SoftBodyCollider", meta = (ClampMin = "0.0"))
	float Radius = 10.0f;

	// 胶囊半长 (cm)，沿组件本地 Z 轴
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SoftBodyCollider", meta = (ClampMin = "0.0"))
	float CapsuleHalfHeight = 20.0f;

	// 盒半尺寸 (cm)，即盒从中心沿各轴延伸的距离 (UE 约定与 UBoxComponent 一致)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SoftBodyCollider")
	FVector BoxExtent = FVector(10.0f, 10.0f, 10.0f);

	// 摩擦系数 (0.0 ~ 1.0)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SoftBodyCollider", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float Friction = 0.2f;

	// 反弹系数 (0.0 ~ 1.0)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SoftBodyCollider", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float Restitution = 0.2f;

	// 是否启用该碰撞体
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SoftBodyCollider")
	bool bEnabled = true;

	// 是否在运行时绘制调试可视化
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SoftBodyCollider")
	bool bShowDebug = true;

private:
	// 上一帧世界位置，用于差分计算线速度
	FVector PrevWorldLocation = FVector::ZeroVector;
	FVector CachedVelocity = FVector::ZeroVector;
	bool bPrevLocationValid = false;
};
