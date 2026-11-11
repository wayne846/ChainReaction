#include "RopeTestWeight.h"

#include "Components/StaticMeshComponent.h"
#include "Engine/CollisionProfile.h"
#include "Engine/StaticMesh.h"
#include "Materials/MaterialInterface.h"
#include "UObject/ConstructorHelpers.h"

ARopeTestWeight::ARopeTestWeight()
{
	Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	RootComponent = Root;

	Mesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Mesh"));
	Mesh->SetupAttachment(Root);

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
	// Basic cube is 100 cm. Scale the child mesh, never the root (the actor transform owns the root).
	Mesh->SetRelativeScale3D(FVector(Size / 100.f));
	Mesh->SetMassOverrideInKg(NAME_None, MassKg, true);
}

FVector ARopeTestWeight::GetTopAttachPoint() const
{
	// Mesh detaches from Root once it simulates, so follow the mesh, not the actor.
	return Mesh->GetComponentLocation() + Mesh->GetUpVector() * (Size * 0.5f);
}
