#include "PulleyTestActors.h"

#include "Components/BoxComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/CollisionProfile.h"
#include "Engine/Engine.h"
#include "Engine/StaticMesh.h"
#include "Materials/MaterialInterface.h"
#include "PulleySystem.h"
#include "TimerManager.h"
#include "UObject/ConstructorHelpers.h"

DEFINE_LOG_CATEGORY_STATIC(LogPulleyTest, Log, All);

// ---------------------------------------------------------------------------
// Bucket
// ---------------------------------------------------------------------------

APulleyTestBucket::APulleyTestBucket()
{
	Base = CreateDefaultSubobject<UBoxComponent>(TEXT("Base"));
	RootComponent = Base;
	Base->SetCollisionProfileName(UCollisionProfile::PhysicsActor_ProfileName);
	Base->SetMobility(EComponentMobility::Movable);
	Base->SetSimulatePhysics(true);
	Base->SetHiddenInGame(true);
	Base->BodyInstance.AngularDamping = 0.5f;

	static ConstructorHelpers::FObjectFinder<UStaticMesh> Cube(TEXT("/Engine/BasicShapes/Cube.Cube"));
	static ConstructorHelpers::FObjectFinder<UMaterialInterface> Mat(TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));

	for (int32 i = 0; i < 4; ++i)
	{
		UBoxComponent* Wall = CreateDefaultSubobject<UBoxComponent>(*FString::Printf(TEXT("Wall%d"), i));
		Wall->SetupAttachment(Base);
		Wall->SetCollisionProfileName(UCollisionProfile::PhysicsActor_ProfileName);
		Wall->SetHiddenInGame(true);
		Walls.Add(Wall);
	}
	for (int32 i = 0; i < 5; ++i)
	{
		UStaticMeshComponent* Vis = CreateDefaultSubobject<UStaticMeshComponent>(*FString::Printf(TEXT("Visual%d"), i));
		Vis->SetupAttachment(Base);
		Vis->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		if (Cube.Succeeded()) { Vis->SetStaticMesh(Cube.Object); }
		if (Mat.Succeeded()) { Vis->SetMaterial(0, Mat.Object); }
		Visuals.Add(Vis);
	}
}

void APulleyTestBucket::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);

	const float T = 2.f;            // wall thickness
	const float Half = Width * 0.5f + T;
	Base->SetBoxExtent(FVector(Half, Half, T), false);

	// Walls stand on the base, around the inside.
	const FVector WallExt[4] = { FVector(T, Half, Height * 0.5f), FVector(T, Half, Height * 0.5f), FVector(Half, T, Height * 0.5f), FVector(Half, T, Height * 0.5f) };
	const FVector WallLoc[4] = { FVector(Half - T, 0, T + Height * 0.5f), FVector(-(Half - T), 0, T + Height * 0.5f), FVector(0, Half - T, T + Height * 0.5f), FVector(0, -(Half - T), T + Height * 0.5f) };
	for (int32 i = 0; i < 4; ++i)
	{
		Walls[i]->SetBoxExtent(WallExt[i], false);
		Walls[i]->SetRelativeLocation(WallLoc[i]);
		Visuals[i]->SetRelativeLocation(WallLoc[i]);
		Visuals[i]->SetRelativeScale3D(WallExt[i] / 50.f);
	}
	Visuals[4]->SetRelativeLocation(FVector::ZeroVector);
	Visuals[4]->SetRelativeScale3D(FVector(Half, Half, T) / 50.f);

	ApplyMass();
}

void APulleyTestBucket::ApplyMass()
{
	// Walls are welded into the base body, and each welded shape keeps its own mass, so split MassKg over all parts.
	Base->SetMassOverrideInKg(NAME_None, MassKg * 0.6f, true);
	for (UBoxComponent* Wall : Walls) { Wall->SetMassOverrideInKg(NAME_None, MassKg * 0.1f, true); }
}

void APulleyTestBucket::BeginPlay()
{
	ApplyMass();
	Super::BeginPlay();
	// How welded shapes combine their overrides is engine-internal; measure and rescale until the total matches.
	float Scale = 1.f;
	for (int32 Iter = 0; Iter < 4; ++Iter)
	{
		const float M = Base->GetMass();
		if (M <= KINDA_SMALL_NUMBER || FMath::IsNearlyEqual(M, MassKg, 0.01f)) { break; }
		Scale *= MassKg / M;
		Base->SetMassOverrideInKg(NAME_None, MassKg * 0.6f * Scale, true);
		for (UBoxComponent* Wall : Walls) { Wall->SetMassOverrideInKg(NAME_None, MassKg * 0.1f * Scale, true); }
	}
	UE_LOG(LogPulleyTest, Log, TEXT("%s: body mass %.2f kg (target %.2f)"), *GetName(), Base->GetMass(), MassKg);
}

// ---------------------------------------------------------------------------
// Driver
// ---------------------------------------------------------------------------

APulleyTestDriver::APulleyTestDriver()
{
	RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
}

void APulleyTestDriver::BeginPlay()
{
	Super::BeginPlay();

	if (Pulley)
	{
		Pulley->OnEndReachedHeight.AddDynamic(this, &APulleyTestDriver::HandleReachedHeight);
	}
	if (UPrimitiveComponent* BallPrim = Ball ? Cast<UPrimitiveComponent>(Ball->GetRootComponent()) : nullptr)
	{
		BallPrim->SetSimulatePhysics(false);
		GetWorldTimerManager().SetTimer(DropTimer, this, &APulleyTestDriver::DropBall, FMath::Max(DropDelay, 0.01f), false);
	}
	UE_LOG(LogPulleyTest, Log, TEXT("PulleyTest: driver ready (pulley %s, ball %s, drop in %.1f s)"), *GetNameSafe(Pulley), *GetNameSafe(Ball), DropDelay);
}

void APulleyTestDriver::DropBall()
{
	UPrimitiveComponent* BallPrim = Ball ? Cast<UPrimitiveComponent>(Ball->GetRootComponent()) : nullptr;
	if (!BallPrim) { return; }
	BallPrim->SetMobility(EComponentMobility::Movable);
	BallPrim->SetSimulatePhysics(true);
	BallPrim->SetMassOverrideInKg(NAME_None, BallMassKg, true);
	BallPrim->WakeRigidBody();
	UE_LOG(LogPulleyTest, Log, TEXT("PulleyTest: ball dropped (%.2f kg)"), BallPrim->GetMass());
}

void APulleyTestDriver::HandleReachedHeight(AActor* EndActor, bool bIsStartEnd, float Height)
{
	const FString Msg = FString::Printf(TEXT("PulleyTest: EVENT %s end %s reached %.0f"), bIsStartEnd ? TEXT("start") : TEXT("end"), *GetNameSafe(EndActor), Height);
	UE_LOG(LogPulleyTest, Log, TEXT("%s"), *Msg);
	if (GEngine) { GEngine->AddOnScreenDebugMessage(-1, 10.f, FColor::Green, Msg); }

	if (EndActor && TriggeredMaterial)
	{
		TArray<UStaticMeshComponent*> Meshes;
		EndActor->GetComponents<UStaticMeshComponent>(Meshes);
		for (UStaticMeshComponent* M : Meshes) { M->SetMaterial(0, TriggeredMaterial); }
	}
}
