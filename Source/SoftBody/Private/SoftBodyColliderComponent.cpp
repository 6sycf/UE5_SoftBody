#include "SoftBodyColliderComponent.h"
#include "DrawDebugHelpers.h"

USoftBodyColliderComponent::USoftBodyColliderComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
}

void USoftBodyColliderComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	FVector WorldLoc = GetComponentLocation();

	// 速度 = 位移差分 (USceneComponent 没有 GetComponentVelocity)
	if (bPrevLocationValid && DeltaTime > 1e-6f)
	{
		CachedVelocity = (WorldLoc - PrevWorldLocation) / DeltaTime;
	}
	else
	{
		CachedVelocity = FVector::ZeroVector;
	}
	PrevWorldLocation = WorldLoc;
	bPrevLocationValid = true;

	// 调试可视化
	if (bShowDebug)
	{
		UWorld* World = GetWorld();
		if (!World) return;

		const FQuat Rot = GetComponentQuat();
		switch (Shape)
		{
		case ESoftBodyColliderShape::Sphere:
			DrawDebugSphere(World, WorldLoc, Radius, 16, FColor::Orange, false, -1.0f, 0, 1.0f);
			break;
		case ESoftBodyColliderShape::Box:
			DrawDebugBox(World, WorldLoc, BoxExtent, Rot, FColor::Orange, false, -1.0f, 0, 1.0f);
			break;
		case ESoftBodyColliderShape::Capsule:
			// DrawDebugCapsule 的 HalfHeight 是含半球的总半高 = 半长 + 半径
			DrawDebugCapsule(World, WorldLoc, CapsuleHalfHeight + Radius, Radius, Rot, FColor::Orange, false, -1.0f, 0, 1.0f);
			break;
		}
	}
}

void USoftBodyColliderComponent::GetWorldCollider(FSoftBodyColliderPrimitive& Out) const
{
	Out.Shape = Shape;
	Out.Center = GetComponentLocation();
	Out.Rotation = GetComponentQuat();
	Out.Friction = Friction;
	Out.Restitution = Restitution;
	Out.Velocity = CachedVelocity;

	switch (Shape)
	{
	case ESoftBodyColliderShape::Sphere:
		Out.Radius = Radius;
		Out.CapsuleHalfHeight = 0.0f;
		Out.HalfExtents = FVector::ZeroVector;
		break;
	case ESoftBodyColliderShape::Box:
		Out.HalfExtents = BoxExtent;
		Out.Radius = 0.0f;
		Out.CapsuleHalfHeight = 0.0f;
		break;
	case ESoftBodyColliderShape::Capsule:
		Out.Radius = Radius;
		Out.CapsuleHalfHeight = CapsuleHalfHeight;
		Out.HalfExtents = FVector::ZeroVector;
		break;
	}
}

void USoftBodyColliderComponent::SetColliderEnabled(bool bEnable)
{
	bEnabled = bEnable;
}
