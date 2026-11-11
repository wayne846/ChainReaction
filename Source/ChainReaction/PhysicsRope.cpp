#include "PhysicsRope.h"

#include "Components/CapsuleComponent.h"
#include "Components/InputComponent.h"
#include "Components/SplineMeshComponent.h"
#include "Engine/CollisionProfile.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"
#include "HAL/IConsoleManager.h"
#include "Materials/MaterialInterface.h"
#include "PhysicsEngine/PhysicsConstraintComponent.h"
#include "TimerManager.h"
#include "UObject/ConstructorHelpers.h"

DEFINE_LOG_CATEGORY_STATIC(LogPhysicsRope, Log, All);

// Tuning / diagnostics switches (read when constraints are created at BeginPlay).
static TAutoConsoleVariable<int32> CVarRopeProjection(TEXT("rope.Projection"), 0,
	TEXT("Rope joint projection: 0 = off, 1 = engine default (linear alpha 1, 5 cm tolerance)."));
static TAutoConsoleVariable<int32> CVarRopeParentTowardRoot(TEXT("rope.ParentTowardRoot"), 1,
	TEXT("1 = constraint parent is the body closer to the held end (recommended), 0 = reversed."));
static TAutoConsoleVariable<int32> CVarRopeAngularLimited(TEXT("rope.AngularLimited"), 1,
	TEXT("1 = limit bend/twist between segments, 0 = free angular motion."));
static TAutoConsoleVariable<float> CVarRopeShock(TEXT("rope.ShockPropagation"), 0.f,
	TEXT("Shock propagation alpha on rope joints (0 = off)."));
static TAutoConsoleVariable<int32> CVarRopeDebug(TEXT("rope.Debug"), 0,
	TEXT("1 = log rope state (end position, max joint gap) every 0.25 s while playing."));

APhysicsRope::APhysicsRope()
{
	PrimaryActorTick.bCanEverTick = true;
	// Read segment transforms after physics has moved them this frame.
	PrimaryActorTick.TickGroup = TG_PostPhysics;

	Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	RootComponent = Root;

	static ConstructorHelpers::FObjectFinder<UStaticMesh> CylinderMesh(TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
	if (CylinderMesh.Succeeded())
	{
		RopeMesh = CylinderMesh.Object;
	}
	static ConstructorHelpers::FObjectFinder<UMaterialInterface> DefaultMat(TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
	if (DefaultMat.Succeeded())
	{
		RopeMaterial = DefaultMat.Object;
	}

	StartAnchor.Mode = ERopeAnchorMode::WorldPoint;
	EndAnchor.Mode = ERopeAnchorMode::Free;
}

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

void APhysicsRope::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);
	BuildRope();
}

void APhysicsRope::DestroyRope()
{
	for (UCapsuleComponent* Seg : Segments)
	{
		if (IsValid(Seg)) { Seg->DestroyComponent(); }
	}
	for (USplineMeshComponent* Vis : Visuals)
	{
		if (IsValid(Vis)) { Vis->DestroyComponent(); }
	}
	for (UPhysicsConstraintComponent* Joint : Joints)
	{
		if (IsValid(Joint)) { Joint->DestroyComponent(); }
	}
	Segments.Reset();
	Visuals.Reset();
	Joints.Reset();
	JointBroken.Reset();
}

void APhysicsRope::ComputeInitialPoints(TArray<FVector>& OutPoints) const
{
	const int32 N = FMath::Max(NumSegments, 2);
	const FTransform& T = GetActorTransform();
	const FVector S = T.TransformPosition(StartPoint);
	const FVector E = T.TransformPosition(EndPoint);
	const float L = FMath::Max(RopeLength, 1.f);
	const float D = FVector::Dist(S, E);

	OutPoints.Reset(N + 1);

	if (L <= D + 0.5f)
	{
		// Rope not longer than the gap: straight line from start toward end, of length L.
		const FVector Dir = D > KINDA_SMALL_NUMBER ? (E - S) / D : FVector::DownVector;
		for (int32 i = 0; i <= N; ++i)
		{
			OutPoints.Add(S + Dir * (L * i / N));
		}
		return;
	}

	// Rope longer than the gap: lay it out as a "V" sagging downward (perpendicular to the span),
	// so the rope starts with exactly its rest length and no constraint is violated at spawn.
	const FVector Span = D > KINDA_SMALL_NUMBER ? (E - S) / D : FVector::ForwardVector;
	FVector SagDir = FVector::DownVector - Span * FVector::DotProduct(FVector::DownVector, Span);
	if (!SagDir.Normalize())
	{
		// Span is vertical: sag sideways.
		SagDir = FVector::CrossProduct(Span, FVector::RightVector).GetSafeNormal();
		if (SagDir.IsNearlyZero()) { SagDir = FVector::ForwardVector; }
	}
	const float Half = L * 0.5f;
	const float SagDepth = FMath::Sqrt(FMath::Max(Half * Half - (D * 0.5f) * (D * 0.5f), 0.f));
	const FVector Mid = (S + E) * 0.5f + SagDir * SagDepth;

	for (int32 i = 0; i <= N; ++i)
	{
		const float Arc = L * i / N;
		if (Arc <= Half)
		{
			OutPoints.Add(FMath::Lerp(S, Mid, Arc / Half));
		}
		else
		{
			OutPoints.Add(FMath::Lerp(Mid, E, (Arc - Half) / Half));
		}
	}
}

void APhysicsRope::BuildRope()
{
	DestroyRope();

	const int32 N = FMath::Max(NumSegments, 2);
	const bool bInConstruction = !HasActorBegunPlay();
	const EComponentCreationMethod Method = bInConstruction ? EComponentCreationMethod::UserConstructionScript : EComponentCreationMethod::Instance;

	TArray<FVector> Points;
	ComputeInitialPoints(Points);

	SegmentLength = FMath::Max(RopeLength, 1.f) / N;
	const float Radius = FMath::Max(RopeThickness * 0.5f, 0.1f);
	// Slightly shorter than the segment so neighbouring capsules never start overlapping.
	const float HalfHeight = FMath::Max(SegmentLength * 0.45f, Radius + 0.01f);
	const float VisualScale = RopeThickness / 100.f; // Basic cylinder is 100 cm wide.

	for (int32 i = 0; i < N; ++i)
	{
		const FVector A = Points[i];
		const FVector B = Points[i + 1];
		FVector Dir = (B - A).GetSafeNormal();
		if (Dir.IsNearlyZero()) { Dir = FVector::DownVector; }
		const FVector Center = (A + B) * 0.5f;
		const FRotator Rot = FRotationMatrix::MakeFromZ(Dir).Rotator();

		// ---- physics body ----
		UCapsuleComponent* Seg = NewObject<UCapsuleComponent>(this);
		Seg->CreationMethod = Method;
		Seg->SetupAttachment(Root);
		Seg->SetUsingAbsoluteLocation(true);
		Seg->SetUsingAbsoluteRotation(true);
		Seg->SetUsingAbsoluteScale(true);
		Seg->SetRelativeLocationAndRotation(Center, Rot);
		Seg->InitCapsuleSize(Radius, HalfHeight);
		Seg->SetCollisionProfileName(UCollisionProfile::PhysicsActor_ProfileName);
		Seg->SetHiddenInGame(true);
		Seg->SetCanEverAffectNavigation(false);
		Seg->BodyInstance.SetOverrideIterationCounts(true);
		Seg->BodyInstance.SetPositionSolverIterationCount((uint8)FMath::Clamp(PositionIterations, 1, 255));
		Seg->BodyInstance.SetVelocitySolverIterationCount((uint8)FMath::Clamp(VelocityIterations, 1, 255));
		Seg->BodyInstance.SetProjectionSolverIterationCount((uint8)FMath::Clamp(ProjectionIterations, 0, 255));
		Seg->BodyInstance.LinearDamping = LinearDamping;
		Seg->BodyInstance.AngularDamping = AngularDamping;
		Seg->SetSimulatePhysics(true);
		Seg->RegisterComponent();
		Seg->SetMassOverrideInKg(NAME_None, SegmentMass, true);
		Segments.Add(Seg);

		// ---- visual ----
		USplineMeshComponent* Vis = NewObject<USplineMeshComponent>(this);
		Vis->CreationMethod = Method;
		Vis->SetMobility(EComponentMobility::Movable);
		Vis->SetupAttachment(Root);
		// Identity world transform, so spline-mesh local space == world space.
		Vis->SetUsingAbsoluteLocation(true);
		Vis->SetUsingAbsoluteRotation(true);
		Vis->SetUsingAbsoluteScale(true);
		Vis->SetRelativeTransform(FTransform::Identity);
		Vis->SetStaticMesh(RopeMesh);
		if (RopeMaterial) { Vis->SetMaterial(0, RopeMaterial); }
		Vis->SetForwardAxis(ESplineMeshAxis::Z, false);
		Vis->SetStartScale(FVector2D(VisualScale, VisualScale), false);
		Vis->SetEndScale(FVector2D(VisualScale, VisualScale), false);
		Vis->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		Vis->SetCanEverAffectNavigation(false);
		Vis->RegisterComponent();
		Visuals.Add(Vis);
	}

	JointBroken.Init(false, N + 1);
	UpdateVisuals();
}

// ---------------------------------------------------------------------------
// Play
// ---------------------------------------------------------------------------

void APhysicsRope::BeginPlay()
{
	Super::BeginPlay();

	bool bValid = Segments.Num() == FMath::Max(NumSegments, 2) && Visuals.Num() == Segments.Num();
	for (int32 i = 0; bValid && i < Segments.Num(); ++i)
	{
		bValid = IsValid(Segments[i]) && IsValid(Visuals[i]);
	}
	if (!bValid)
	{
		BuildRope();
	}

	CreateConstraints();
	TryBindDebugInput();

	if (DebugAutoCutDelay > 0.f)
	{
		GetWorldTimerManager().SetTimer(AutoCutTimer, this, &APhysicsRope::HandleDebugCut, DebugAutoCutDelay, false);
	}
}

void APhysicsRope::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	if (!bDebugInputBound) { TryBindDebugInput(); }
	UpdateVisuals();

	if (CVarRopeDebug.GetValueOnGameThread() != 0 && Segments.Num() > 1)
	{
		// Log 4x per second: end point, max gap across intact joints, total stretch.
		const double Now = GetWorld()->GetTimeSeconds();
		if (FMath::FloorToInt(Now * 4.0) != FMath::FloorToInt((Now - DeltaSeconds) * 4.0))
		{
			float MaxGap = 0.f, SumGap = 0.f;
			for (int32 k = 1; k < Segments.Num(); ++k)
			{
				if (!IsJointIntact(k)) { continue; }
				FVector S0, E0, S1, E1;
				GetSegmentEnds(k - 1, S0, E0);
				GetSegmentEnds(k, S1, E1);
				const float Gap = FVector::Dist(E0, S1);
				MaxGap = FMath::Max(MaxGap, Gap);
				SumGap += Gap;
			}
			const FVector EndLoc = GetJointLocation(Segments.Num());
			const FVector StartLoc = GetJointLocation(0);
			const float EndSpeed = Segments.Last()->GetPhysicsLinearVelocity().Size();
			UE_LOG(LogPhysicsRope, Log, TEXT("RopeDbg t=%.2f start=(%.1f,%.1f,%.1f) end=(%.1f,%.1f,%.1f) endSpeed=%.1f maxGap=%.2f sumGap=%.2f"),
				Now, StartLoc.X, StartLoc.Y, StartLoc.Z, EndLoc.X, EndLoc.Y, EndLoc.Z, EndSpeed, MaxGap, SumGap);
		}
	}
}

UPrimitiveComponent* APhysicsRope::ResolveAnchorComponent(const FRopeAnchor& Anchor) const
{
	AActor* Target = Anchor.AttachActor;
	if (!IsValid(Target)) { return nullptr; }

	TArray<UPrimitiveComponent*> Prims;
	Target->GetComponents<UPrimitiveComponent>(Prims);

	if (!Anchor.ComponentName.IsNone())
	{
		for (UPrimitiveComponent* P : Prims)
		{
			if (P && P->GetFName() == Anchor.ComponentName) { return P; }
		}
		UE_LOG(LogPhysicsRope, Warning, TEXT("%s: component '%s' not found on %s"), *GetName(), *Anchor.ComponentName.ToString(), *Target->GetName());
		return nullptr;
	}
	if (UPrimitiveComponent* RootPrim = Cast<UPrimitiveComponent>(Target->GetRootComponent()))
	{
		return RootPrim;
	}
	for (UPrimitiveComponent* P : Prims)
	{
		if (P && P->IsSimulatingPhysics()) { return P; }
	}
	return Prims.Num() > 0 ? Prims[0] : nullptr;
}

UPhysicsConstraintComponent* APhysicsRope::MakeConstraint(const FVector& Location, const FVector& RopeDir, UPrimitiveComponent* Child, FName ChildBone, UPrimitiveComponent* Parent, FName ParentBone, bool bIsAnchor)
{
	UPhysicsConstraintComponent* C = NewObject<UPhysicsConstraintComponent>(this);
	C->CreationMethod = EComponentCreationMethod::Instance;
	C->SetupAttachment(Root);
	C->SetUsingAbsoluteLocation(true);
	C->SetUsingAbsoluteRotation(true);
	// Constraint X axis (twist) along the rope.
	C->SetRelativeLocationAndRotation(Location, FRotationMatrix::MakeFromX(RopeDir.IsNearlyZero() ? FVector::DownVector : RopeDir).Rotator());

	// Rope does not stretch: lock all linear motion.
	C->SetLinearXLimit(LCM_Locked, 0.f);
	C->SetLinearYLimit(LCM_Locked, 0.f);
	C->SetLinearZLimit(LCM_Locked, 0.f);

	if (bIsAnchor || CVarRopeAngularLimited.GetValueOnGameThread() == 0)
	{
		// A hanging point lets the rope swing and twist freely.
		C->SetAngularSwing1Limit(ACM_Free, 0.f);
		C->SetAngularSwing2Limit(ACM_Free, 0.f);
		C->SetAngularTwistLimit(ACM_Free, 0.f);
	}
	else
	{
		C->SetAngularSwing1Limit(ACM_Limited, MaxBendPerJoint);
		C->SetAngularSwing2Limit(ACM_Limited, MaxBendPerJoint);
		C->SetAngularTwistLimit(ACM_Limited, 30.f);
	}

	C->SetDisableCollision(true);
	if (CVarRopeProjection.GetValueOnGameThread() == 1)
	{
		C->ConstraintInstance.SetProjectionParams(true, 1.0f, 0.0f, 5.0f, 180.0f);
	}
	else
	{
		C->ConstraintInstance.DisableProjection();
	}
	const float Shock = CVarRopeShock.GetValueOnGameThread();
	C->ConstraintInstance.SetShockPropagationParams(Shock > 0.f, FMath::Clamp(Shock, 0.f, 1.f));
	if (bUseMassConditioning)
	{
		C->ConstraintInstance.EnableMassConditioning();
	}
	else
	{
		C->ConstraintInstance.DisableMassConditioning();
	}

	C->RegisterComponent();
	// The joint frames are computed from the component's world transform inside SetConstrainedComponents.
	// Set it again now that the component is registered so ComponentToWorld is guaranteed current;
	// otherwise every joint can end up at the actor origin and the rope collapses onto one point.
	const FRotator JointRot = FRotationMatrix::MakeFromX(RopeDir.IsNearlyZero() ? FVector::DownVector : RopeDir).Rotator();
	C->SetWorldLocationAndRotation(Location, JointRot);
	C->UpdateComponentToWorld();
	if (!C->GetComponentLocation().Equals(Location, 0.1f))
	{
		UE_LOG(LogPhysicsRope, Warning, TEXT("%s: joint placed at %s, expected %s"), *GetName(), *C->GetComponentLocation().ToString(), *Location.ToString());
	}
	C->SetConstrainedComponents(Child, ChildBone, Parent, ParentBone);
	return C;
}

void APhysicsRope::CreateConstraints()
{
	for (UPhysicsConstraintComponent* Joint : Joints)
	{
		if (IsValid(Joint)) { Joint->DestroyComponent(); }
	}
	const int32 N = Segments.Num();
	Joints.Init(nullptr, N + 1);
	JointBroken.Init(false, N + 1);
	if (N < 2) { return; }

	TArray<FVector> Starts, Ends;
	Starts.SetNum(N);
	Ends.SetNum(N);
	for (int32 i = 0; i < N; ++i)
	{
		GetSegmentEnds(i, Starts[i], Ends[i]);
	}

	// FConstraintInstance convention: Component1 = child, Component2 = parent, and projection
	// treats the parent as infinitely heavy. The parent must therefore always be the body closer
	// to the held end of the rope; the reverse pumps energy up the chain (growing swing).
	// The rope is rooted at the start unless only the end is held.
	bool bRootAtEnd = StartAnchor.Mode == ERopeAnchorMode::Free && EndAnchor.Mode != ERopeAnchorMode::Free;
	if (CVarRopeParentTowardRoot.GetValueOnGameThread() == 0) { bRootAtEnd = !bRootAtEnd; } // diagnostics only

	// Between segments.
	for (int32 k = 1; k < N; ++k)
	{
		const FVector Loc = (Ends[k - 1] + Starts[k]) * 0.5f;
		const FVector Dir = (Segments[k]->GetComponentLocation() - Segments[k - 1]->GetComponentLocation()).GetSafeNormal();
		UCapsuleComponent* Child  = bRootAtEnd ? Segments[k - 1] : Segments[k];
		UCapsuleComponent* Parent = bRootAtEnd ? Segments[k] : Segments[k - 1];
		Joints[k] = MakeConstraint(Loc, Dir, Child, NAME_None, Parent, NAME_None, false);
	}

	// Anchors. On the root end the anchor (world / target) is the parent of the rope;
	// on the other end the attached target hangs from the rope as its child.
	auto MakeAnchor = [this](const FRopeAnchor& Anchor, UCapsuleComponent* Seg, const FVector& Loc, const FVector& Dir, bool bIsRootEnd) -> UPhysicsConstraintComponent*
	{
		switch (Anchor.Mode)
		{
		case ERopeAnchorMode::WorldPoint:
			// Null component = the world, which is always the parent.
			return MakeConstraint(Loc, Dir, Seg, NAME_None, nullptr, NAME_None, true);
		case ERopeAnchorMode::AttachToActor:
			if (UPrimitiveComponent* Target = ResolveAnchorComponent(Anchor))
			{
				return bIsRootEnd
					? MakeConstraint(Loc, Dir, Seg, NAME_None, Target, Anchor.BoneName, true)
					: MakeConstraint(Loc, Dir, Target, Anchor.BoneName, Seg, NAME_None, true);
			}
			UE_LOG(LogPhysicsRope, Warning, TEXT("%s: AttachToActor anchor has no valid target; end left free."), *GetName());
			return nullptr;
		default:
			return nullptr;
		}
	};

	const FVector StartDir = (Ends[0] - Starts[0]).GetSafeNormal();
	const FVector EndDir = (Ends[N - 1] - Starts[N - 1]).GetSafeNormal();
	Joints[0] = MakeAnchor(StartAnchor, Segments[0], Starts[0], StartDir, !bRootAtEnd);
	Joints[N] = MakeAnchor(EndAnchor, Segments[N - 1], Ends[N - 1], EndDir, bRootAtEnd);
}

// ---------------------------------------------------------------------------
// Cutting
// ---------------------------------------------------------------------------

bool APhysicsRope::IsJointIntact(int32 JointIndex) const
{
	return Joints.IsValidIndex(JointIndex) && IsValid(Joints[JointIndex])
		&& JointBroken.IsValidIndex(JointIndex) && !JointBroken[JointIndex];
}

bool APhysicsRope::CutAtJoint(int32 JointIndex)
{
	if (!IsJointIntact(JointIndex)) { return false; }

	const FVector Loc = GetJointLocation(JointIndex);
	Joints[JointIndex]->BreakConstraint();
	JointBroken[JointIndex] = true;

	// Make sure both sides react immediately.
	for (int32 SegIdx : { JointIndex - 1, JointIndex })
	{
		if (Segments.IsValidIndex(SegIdx) && IsValid(Segments[SegIdx]))
		{
			Segments[SegIdx]->WakeAllRigidBodies();
		}
	}

	OnRopeCut.Broadcast(JointIndex, Loc);
	return true;
}

bool APhysicsRope::CutAtLocation(FVector WorldLocation, float MaxDistance)
{
	int32 Best = INDEX_NONE;
	float BestDistSq = TNumericLimits<float>::Max();
	for (int32 k = 0; k < Joints.Num(); ++k)
	{
		if (!IsJointIntact(k)) { continue; }
		const float DistSq = FVector::DistSquared(GetJointLocation(k), WorldLocation);
		if (DistSq < BestDistSq)
		{
			BestDistSq = DistSq;
			Best = k;
		}
	}
	if (Best == INDEX_NONE) { return false; }
	if (MaxDistance >= 0.f && BestDistSq > MaxDistance * MaxDistance) { return false; }
	return CutAtJoint(Best);
}

// ---------------------------------------------------------------------------
// Queries & visuals
// ---------------------------------------------------------------------------

void APhysicsRope::GetSegmentEnds(int32 Index, FVector& OutStart, FVector& OutEnd) const
{
	const FTransform& T = Segments[Index]->GetComponentTransform();
	const FVector Up = T.GetUnitAxis(EAxis::Z);
	const FVector C = T.GetLocation();
	OutStart = C - Up * (SegmentLength * 0.5f);
	OutEnd = C + Up * (SegmentLength * 0.5f);
}

FVector APhysicsRope::GetJointLocation(int32 JointIndex) const
{
	const int32 N = Segments.Num();
	if (N == 0 || JointIndex < 0 || JointIndex > N) { return GetActorLocation(); }
	FVector S, E;
	if (JointIndex == 0)
	{
		GetSegmentEnds(0, S, E);
		return S;
	}
	if (JointIndex == N)
	{
		GetSegmentEnds(N - 1, S, E);
		return E;
	}
	FVector S0, E0, S1, E1;
	GetSegmentEnds(JointIndex - 1, S0, E0);
	GetSegmentEnds(JointIndex, S1, E1);
	return (E0 + S1) * 0.5f;
}

UCapsuleComponent* APhysicsRope::GetSegment(int32 SegmentIndex) const
{
	return Segments.IsValidIndex(SegmentIndex) ? Segments[SegmentIndex].Get() : nullptr;
}

void APhysicsRope::UpdateVisuals()
{
	const int32 N = Segments.Num();
	if (N == 0 || Visuals.Num() != N) { return; }
	for (int32 i = 0; i < N; ++i)
	{
		if (!IsValid(Segments[i]) || !IsValid(Visuals[i])) { return; }
	}

	auto IsConnected = [this, N](int32 k) -> bool
	{
		return k > 0 && k < N && !(JointBroken.IsValidIndex(k) && JointBroken[k]);
	};

	TArray<FVector> S, E, PS, PE;
	S.SetNum(N); E.SetNum(N); PS.SetNum(N); PE.SetNum(N);
	for (int32 i = 0; i < N; ++i)
	{
		GetSegmentEnds(i, S[i], E[i]);
	}
	// Shared joint points make the visual rope continuous across intact joints.
	for (int32 i = 0; i < N; ++i)
	{
		PS[i] = IsConnected(i) ? (E[i - 1] + S[i]) * 0.5f : S[i];
		PE[i] = IsConnected(i + 1) ? (E[i] + S[i + 1]) * 0.5f : E[i];
	}
	for (int32 i = 0; i < N; ++i)
	{
		const FVector Chord = PE[i] - PS[i];
		// Catmull-Rom style tangents across intact joints for a smooth curve.
		const FVector TStart = IsConnected(i) ? (PE[i] - PS[i - 1]) * 0.5f : Chord;
		const FVector TEnd = IsConnected(i + 1) ? (PE[i + 1] - PS[i]) * 0.5f : Chord;
		Visuals[i]->SetStartAndEnd(PS[i], TStart, PE[i], TEnd, true);
	}
}

// ---------------------------------------------------------------------------
// Debug
// ---------------------------------------------------------------------------

int32 APhysicsRope::ResolveDebugJoint() const
{
	const int32 N = Segments.Num();
	if (DebugCutJoint < 0) { return N / 2; }
	return FMath::Clamp(DebugCutJoint, 0, N);
}

void APhysicsRope::HandleDebugCut()
{
	const int32 Joint = ResolveDebugJoint();
	const bool bCut = CutAtJoint(Joint);
	UE_LOG(LogPhysicsRope, Log, TEXT("%s: debug cut at joint %d -> %s"), *GetName(), Joint, bCut ? TEXT("cut") : TEXT("already cut / no joint"));
}

void APhysicsRope::TryBindDebugInput()
{
	if (bDebugInputBound || !DebugCutKey.IsValid() || !GetWorld()) { return; }
	APlayerController* PC = GetWorld()->GetFirstPlayerController();
	if (!PC) { return; }
	EnableInput(PC);
	if (InputComponent)
	{
		InputComponent->BindKey(DebugCutKey, IE_Pressed, this, &APhysicsRope::HandleDebugCut);
		bDebugInputBound = true;
		UE_LOG(LogPhysicsRope, Log, TEXT("%s: debug cut key '%s' bound (player controller %s)"), *GetName(), *DebugCutKey.ToString(), *PC->GetName());
	}
}
