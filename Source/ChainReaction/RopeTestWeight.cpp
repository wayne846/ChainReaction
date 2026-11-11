#include "RopeTestWeight.h"

#include "Components/StaticMeshComponent.h"
#include "Engine/CollisionProfile.h"
#include "Engine/StaticMesh.h"
#include "Materials/MaterialInterface.h"
#include "UObject/ConstructorHelpers.h"

ARopeTestWeight::ARopeTestWeight()
{
	Mesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Mesh"));
	RootComponent = Mesh;

	static ConstructorHelpers::FObjectFinder<UStaticMesh> CubeMesh(TEXT("/Engine/BasicShapes/Cube.Cube"));
	if (CubeMesh.Succeeded())
	{
		Mesh->SetStaticMesh(CubeMesh.Object);
	}
	static ConstructorHelpers::FObjectFinder<UMaterialInterface> Mat(TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
	if (Mat.Succeeded())
	{
		Mesh->SetMaterial(0, Mat.Object);
	}

	Mesh->SetMobility(EComponentMobility::Movable);
	Mesh->SetCollisionProfileName(UCollisionProfile::PhysicsActor_ProfileName);
	Mesh->SetSimulatePhysics(true);
	// Same iteration budget as the rope so the hanging pair solves in one well-converged island.
	Mesh->BodyInstance.SetOverrideIterationCounts(true);
	Mesh->BodyInstance.SetPositionSolverIterationCount(16);
	Mesh->BodyInstance.SetVelocitySolverIterationCount(4);
	Mesh->BodyInstance.SetProjectionSolverIterationCount(2);
	Mesh->BodyInstance.AngularDamping = 0.5f;
}

void ARopeTestWeight::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);
	Mesh->SetWorldScale3D(FVector(Size / 100.f));
	Mesh->SetMassOverrideInKg(NAME_None, MassKg, true);
}

FVector ARopeTestWeight::GetTopAttachPoint() const
{
	return GetActorLocation() + GetActorUpVector() * (Size * 0.5f);
}
