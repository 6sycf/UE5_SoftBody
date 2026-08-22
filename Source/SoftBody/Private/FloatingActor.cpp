#include "FloatingActor.h"
#include "Components/StaticMeshComponent.h"

AFloatingActor::AFloatingActor()
{
	PrimaryActorTick.bCanEverTick = true;

	// 根组件：静态网格组件
	MeshComponent = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("MeshComponent"));
	RootComponent = MeshComponent;

	// 因为是运行时移动的物体，设成 Movable，保证 GDF 会跟着更新
	MeshComponent->SetMobility(EComponentMobility::Movable);
}

void AFloatingActor::BeginPlay()
{
	Super::BeginPlay();

	// 记录初始位置，作为浮动的中心基准点
	InitialLocation = GetActorLocation();

	if (bAutoStart)
	{
		bIsFloating = true;
	}
}

void AFloatingActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (!bIsFloating)
	{
		return;
	}

	RunningTime += DeltaTime;

	// 归一化浮动轴，避免传入非单位向量导致幅度被放大
	FVector Axis = FloatAxis.GetSafeNormal();
	if (Axis.IsNearlyZero())
	{
		Axis = FVector::UpVector;
	}

	// 正弦偏移：Offset = A * sin(2π * f * t + φ)
	float Offset = FloatAmplitude * FMath::Sin(2.0f * PI * FloatFrequency * RunningTime + FloatPhase);

	SetActorLocation(InitialLocation + Axis * Offset);
}

void AFloatingActor::StartFloating()
{
	bIsFloating = true;
	RunningTime = 0.0f; // 重新从相位起点开始
}

void AFloatingActor::StopFloating()
{
	bIsFloating = false;
}

void AFloatingActor::SetFloatingEnabled(bool bEnabled)
{
	bIsFloating = bEnabled;
	if (bEnabled)
	{
		RunningTime = 0.0f;
	}
}

void AFloatingActor::ResetToInitialLocation()
{
	SetActorLocation(InitialLocation);
	RunningTime = 0.0f;
}
