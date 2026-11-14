#include "PulleySystem.h"

#include "Chaos/ChaosScene.h"
#include "Chaos/SimCallbackObject.h"
#include "Components/PrimitiveComponent.h"
#include "Components/SplineMeshComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "HAL/IConsoleManager.h"
#include "Materials/MaterialInterface.h"
#include "PBDRigidsSolver.h"
#include "Physics/Experimental/PhysScene_Chaos.h"
#include "PhysicsEngine/BodyInstance.h"
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"
#include "UObject/ConstructorHelpers.h"

#include <atomic>

DEFINE_LOG_CATEGORY_STATIC(LogPulley, Log, All);

static TAutoConsoleVariable<int32> CVarPulleyDebug(TEXT("pulley.Debug"), 0,
	TEXT("1 = log pulley state (hang lengths, tension, end heights) every 0.25 s while playing."));

// ===========================================================================
// Geometry shared by the game thread (drawing) and the physics thread (forces)
// ===========================================================================

/** Fixed pulley layout. All wheels lie in one plane (normal N, in-plane axes U, V; V = N x U). */
struct FPulleyGeometry
{
	TArray<FVector> Centers;
	/** Turn direction of the rope at each wheel: +1 = counter-clockwise about N, -1 = clockwise. */
	TArray<float> Signs;
	float Radius = 15.f;
	FVector N = FVector::YAxisVector;
	FVector U = FVector::XAxisVector;
	FVector V = FVector::ZAxisVector;
};

/** Rope path for given end points. "In" = where the rope meets a wheel, "Out" = where it leaves (start-to-end order). */
struct FPulleyPath
{
	TArray<FVector> In;
	TArray<FVector> Out;
	TArray<float> Arc; // wrap angle (radians) per wheel
	float HangStart = 0.f;  // straight length start object -> first wheel
	float HangEnd = 0.f;    // straight length last wheel -> end object
	FVector DirStart = FVector::DownVector; // unit vector from the first wheel's tangent point toward the start object
	FVector DirEnd = FVector::DownVector;
	float Total = 0.f;
	float TotalWrap = 0.f;
	/** Start-side / end-side lengths up to a point fixed on the pulleys (used to measure rope travel). */
	float SideStart = 0.f;
	float SideEnd = 0.f;
};

namespace PulleyMath
{
	static FVector2D Rot(const FVector2D& P, double A)
	{
		const double C = FMath::Cos(A), S = FMath::Sin(A);
		return FVector2D(P.X * C - P.Y * S, P.X * S + P.Y * C);
	}
	static FVector2D Left(const FVector2D& D) { return FVector2D(-D.Y, D.X); }
	static double Cross(const FVector2D& A, const FVector2D& B) { return A.X * B.Y - A.Y * B.X; }

	static FVector2D To2D(const FPulleyGeometry& G, const FVector& P)
	{
		const FVector D = P - G.Centers[0];
		return FVector2D(FVector::DotProduct(D, G.U), FVector::DotProduct(D, G.V));
	}
	static FVector To3D(const FPulleyGeometry& G, const FVector2D& P)
	{
		return G.Centers[0] + G.U * P.X + G.V * P.Y;
	}

	/** Tangent point on circle (C, R) for a rope travelling from P toward the circle and then turning with sign S. */
	static FVector2D PointTangent(const FVector2D& P, const FVector2D& C, double R, double S)
	{
		FVector2D W = C - P;
		double D = W.Size();
		if (D < KINDA_SMALL_NUMBER) { W = FVector2D(0, 1); D = 1.0; }
		const FVector2D WN = W / D;
		D = FMath::Max(D, R * 1.0001);
		const double Alpha = FMath::Asin(FMath::Clamp(R / D, 0.0, 1.0));
		const FVector2D Dir = Rot(WN, -S * Alpha);
		return C - Left(Dir) * (S * R);
	}

	/** Wrap angle from In to Out around C, travelling with sign S. Negative wraps (rope lifted off) count as 0. */
	static double WrapAngle(const FVector2D& C, const FVector2D& In, const FVector2D& Out, double S)
	{
		const double AIn = FMath::Atan2(In.Y - C.Y, In.X - C.X);
		const double AOut = FMath::Atan2(Out.Y - C.Y, Out.X - C.X);
		double A = S * (AOut - AIn);
		// Normalise to [-pi/2, 3pi/2): a half wrap (single Atwood pulley) stays at pi without flipping.
		while (A < -HALF_PI) { A += TWO_PI; }
		while (A >= 1.5 * PI) { A -= TWO_PI; }
		return FMath::Max(A, 0.0);
	}

	static void ComputePath(const FPulleyGeometry& G, const FVector& PStart, const FVector& PEnd, FPulleyPath& Out)
	{
		const int32 N = G.Centers.Num();
		const double R = G.Radius;
		Out.In.SetNum(N);
		Out.Out.SetNum(N);
		Out.Arc.SetNum(N);

		TArray<FVector2D, TInlineAllocator<8>> C2, In2, Out2;
		C2.SetNum(N); In2.SetNum(N); Out2.SetNum(N);
		for (int32 i = 0; i < N; ++i) { C2[i] = To2D(G, G.Centers[i]); }

		// Ends: rope from the start object onto wheel 0, and (reversed) from the end object onto the last wheel.
		In2[0] = PointTangent(To2D(G, PStart), C2[0], R, G.Signs[0]);
		Out2[N - 1] = PointTangent(To2D(G, PEnd), C2[N - 1], R, -G.Signs[N - 1]);

		// Spans between wheels: external tangent for equal turn directions, crossed (internal) tangent otherwise.
		double Spans = 0.0;
		for (int32 i = 0; i + 1 < N; ++i)
		{
			const double Si = G.Signs[i], Sj = G.Signs[i + 1];
			FVector2D W = C2[i + 1] - C2[i];
			const double D = FMath::Max(W.Size(), KINDA_SMALL_NUMBER);
			const FVector2D WN = W / D;
			FVector2D Dir = WN;
			if (Si != Sj)
			{
				Dir = Rot(WN, -Sj * FMath::Asin(FMath::Clamp(2.0 * R / D, 0.0, 0.999)));
			}
			Out2[i] = C2[i] - Left(Dir) * (Si * R);
			In2[i + 1] = C2[i + 1] - Left(Dir) * (Sj * R);
			Spans += (In2[i + 1] - Out2[i]).Size();
		}

		double Wraps = 0.0;
		for (int32 i = 0; i < N; ++i)
		{
			Out.Arc[i] = (float)WrapAngle(C2[i], In2[i], Out2[i], G.Signs[i]);
			Wraps += Out.Arc[i];
			Out.In[i] = To3D(G, In2[i]);
			Out.Out[i] = To3D(G, Out2[i]);
		}

		const FVector DS = PStart - Out.In[0];
		const FVector DE = PEnd - Out.Out[N - 1];
		Out.HangStart = (float)DS.Size();
		Out.HangEnd = (float)DE.Size();
		Out.DirStart = Out.HangStart > KINDA_SMALL_NUMBER ? DS / Out.HangStart : FVector::DownVector;
		Out.DirEnd = Out.HangEnd > KINDA_SMALL_NUMBER ? DE / Out.HangEnd : FVector::DownVector;
		Out.TotalWrap = (float)Wraps;
		Out.Total = (float)(Out.HangStart + Out.HangEnd + Spans + Wraps * R);

		const float FirstShare = N == 1 ? 0.5f : 1.f;
		Out.SideStart = Out.HangStart + Out.Arc[0] * R * FirstShare;
		Out.SideEnd = Out.HangEnd + Out.Arc[N - 1] * R * FirstShare;
	}
}

// ===========================================================================
// Physics-thread solver
// ===========================================================================

/** Data handed from the game thread to the physics callback. Guarded by Lock. */
struct FPulleySimShared
{
	FCriticalSection Lock;
	bool bActive = false;
	FPulleyGeometry Geom;
	Chaos::FSingleParticlePhysicsProxy* Proxy[2] = { nullptr, nullptr };
	FVector LocalAttach[2];
	/** World tie point for an end that is not a simulated body (acts as a fixed anchor). */
	FVector FixedAttach[2];
	float FreeLength = 0.f;
	float MinHang = 10.f;
	float StiffnessHz = 15.f;
	float DampingRatio = 1.f;
	float Friction = 0.f;

	// Outputs (written by the physics thread).
	std::atomic<float> TensionN { 0.f };
};

class FPulleySimCallback : public Chaos::TSimCallbackObject<>
{
public:
	TSharedPtr<FPulleySimShared, ESPMode::ThreadSafe> Shared;

	virtual void OnPreSimulate_Internal() override
	{
		if (!Shared.IsValid()) { return; }
		FScopeLock ScopeLock(&Shared->Lock);
		FPulleySimShared& S = *Shared;
		if (!S.bActive || S.Geom.Centers.Num() == 0) { return; }

		const double Dt = GetDeltaTime_Internal();
		if (Dt <= 0.0) { return; }

		struct FEnd
		{
			Chaos::FRigidBodyHandle_Internal* H = nullptr;
			bool bDynamic = false;
			bool bSleeping = false;
			FVector P, Com, V, W;
			FQuat InertiaRot;
			FVector InvI;
			double InvM = 0.0;
		};
		FEnd E[2];

		for (int32 k = 0; k < 2; ++k)
		{
			E[k].P = S.FixedAttach[k];
			Chaos::FSingleParticlePhysicsProxy* Proxy = S.Proxy[k];
			Chaos::FRigidBodyHandle_Internal* H = Proxy ? Proxy->GetPhysicsThreadAPI() : nullptr;
			if (!H) { continue; }
			const FVector X = H->GetX();
			const FQuat R = H->GetR();
			E[k].P = X + R.RotateVector(S.LocalAttach[k]);
			const Chaos::EObjectStateType State = H->ObjectState();
			if (State != Chaos::EObjectStateType::Dynamic && State != Chaos::EObjectStateType::Sleeping) { continue; }
			E[k].H = H;
			E[k].bDynamic = true;
			E[k].bSleeping = State == Chaos::EObjectStateType::Sleeping;
			E[k].Com = X + R.RotateVector(H->CenterOfMass());
			E[k].V = H->GetV();
			E[k].W = H->GetW();
			E[k].InvM = H->InvM();
			E[k].InertiaRot = R * FQuat(H->RotationOfMass());
			const Chaos::TVec3<Chaos::FRealSingle> II = H->InvI();
			E[k].InvI = FVector(II.X, II.Y, II.Z);
		}

		if (!E[0].bDynamic && !E[1].bDynamic) { S.TensionN = 0.f; return; }

		auto ApplyInvI = [](const FEnd& B, const FVector& X) -> FVector
		{
			const FVector L = B.InertiaRot.UnrotateVector(X);
			return B.InertiaRot.RotateVector(L * B.InvI);
		};
		// Inverse effective mass of body B for a pull along Dir at its tie point.
		auto InvEffMass = [&](const FEnd& B, const FVector& Dir) -> double
		{
			if (!B.bDynamic) { return 0.0; }
			const FVector RxD = FVector::CrossProduct(B.P - B.Com, Dir);
			return B.InvM + FVector::DotProduct(RxD, ApplyInvI(B, RxD));
		};
		auto PointVel = [](const FEnd& B) -> FVector
		{
			return B.bDynamic ? B.V + FVector::CrossProduct(B.W, B.P - B.Com) : FVector::ZeroVector;
		};
		// Impulse Imp (vector) at the tie point.
		auto ApplyImpulse = [&](FEnd& B, const FVector& Imp)
		{
			if (!B.bDynamic) { return; }
			B.V += Imp * B.InvM;
			B.W += ApplyInvI(B, FVector::CrossProduct(B.P - B.Com, Imp));
		};
		// Soft (spring-damper) constraint parameters for an effective inverse mass K, Box2D style.
		auto SoftParams = [&](double K, double& OutGamma, double& OutBiasPerC)
		{
			const double M = 1.0 / FMath::Max(K, 1e-9);
			const double Omega = TWO_PI * S.StiffnessHz;
			const double Stiff = M * Omega * Omega;
			const double Damp = 2.0 * M * S.DampingRatio * Omega;
			const double Denom = Dt * (Damp + Dt * Stiff);
			OutGamma = Denom > 0.0 ? 1.0 / Denom : 0.0;
			OutBiasPerC = Dt * Stiff * OutGamma;
		};

		FPulleyPath Path;
		PulleyMath::ComputePath(S.Geom, E[0].P, E[1].P, Path);
		const FVector Dir[2] = { Path.DirStart, Path.DirEnd };
		const double Hang[2] = { Path.HangStart, Path.HangEnd };

		// Wake a sleeping end if the other end is moving and the rope is under load.
		const double C = Path.Total - S.FreeLength;
		for (int32 k = 0; k < 2; ++k)
		{
			if (E[k].bDynamic && E[k].bSleeping)
			{
				const FEnd& Other = E[1 - k];
				const bool bOtherAwake = Other.bDynamic && !Other.bSleeping;
				if (bOtherAwake && C > -0.5)
				{
					E[k].H->SetObjectState(Chaos::EObjectStateType::Dynamic);
					E[k].bSleeping = false;
				}
			}
		}
		bool bChanged[2] = { false, false };

		// ---- Rope tension (pull only, while the rope is taut) ----
		double Tension = 0.0;
		if (C > 0.0)
		{
			const FVector V0 = PointVel(E[0]);
			const FVector V1 = PointVel(E[1]);
			const double Rate0 = FVector::DotProduct(Dir[0], V0); // start hang lengthening speed
			const double Rate1 = FVector::DotProduct(Dir[1], V1);

			// Capstan friction: the side the rope slides toward carries more tension.
			const double MaxRatio = FMath::Exp(S.Friction * Path.TotalWrap);
			const double Slip = Rate0 - Rate1; // > 0: rope runs toward the start object
			const double Ratio = FMath::Pow(MaxRatio, FMath::Tanh(Slip / 2.0));
			const double Wt[2] = { 2.0 * Ratio / (1.0 + Ratio), 2.0 / (1.0 + Ratio) };

			const double K = Wt[0] * InvEffMass(E[0], Dir[0]) + Wt[1] * InvEffMass(E[1], Dir[1]);
			if (K > 0.0)
			{
				double Gamma, BiasPerC;
				SoftParams(K, Gamma, BiasPerC);
				const double Lambda = (Rate0 + Rate1 + C * BiasPerC) / (K + Gamma); // pull impulse magnitude
				if (Lambda > 0.0)
				{
					for (int32 k = 0; k < 2; ++k)
					{
						if (E[k].bDynamic && !E[k].bSleeping)
						{
							ApplyImpulse(E[k], -Dir[k] * (Lambda * Wt[k]));
							bChanged[k] = true;
						}
					}
					Tension = Lambda / Dt;
				}
			}
		}

		// ---- Stops: an end may not be pulled into its wheel ----
		for (int32 k = 0; k < 2; ++k)
		{
			if (!E[k].bDynamic || E[k].bSleeping) { continue; }
			const double Cs = Hang[k] - S.MinHang; // want >= 0
			if (Cs >= 0.0) { continue; }
			const double K = InvEffMass(E[k], Dir[k]);
			double Gamma, BiasPerC;
			SoftParams(K, Gamma, BiasPerC);
			const double Rate = FVector::DotProduct(Dir[k], PointVel(E[k]));
			const double Push = -(Rate + Cs * BiasPerC) / (K + Gamma);
			if (Push > 0.0)
			{
				ApplyImpulse(E[k], Dir[k] * Push);
				bChanged[k] = true;
			}
		}

		for (int32 k = 0; k < 2; ++k)
		{
			if (bChanged[k])
			{
				E[k].H->SetV(E[k].V);
				E[k].H->SetW(E[k].W);
			}
		}

		// Unreal units: kg*cm/s^2 -> N.
		S.TensionN = (float)(Tension / 100.0);
	}
};

// ===========================================================================
// Actor
// ===========================================================================

APulleySystem::APulleySystem()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.TickGroup = TG_PostPhysics;

	Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	RootComponent = Root;

	static ConstructorHelpers::FObjectFinder<UStaticMesh> Cylinder(TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
	static ConstructorHelpers::FObjectFinder<UStaticMesh> Cube(TEXT("/Engine/BasicShapes/Cube.Cube"));
	static ConstructorHelpers::FObjectFinder<UMaterialInterface> Mat(TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
	if (Cylinder.Succeeded()) { RopeMesh = Cylinder.Object; WheelMesh = Cylinder.Object; }
	if (Cube.Succeeded()) { SpokeMesh = Cube.Object; }
	if (Mat.Succeeded()) { RopeMaterial = Mat.Object; WheelMaterial = Mat.Object; }

	Pulleys.SetNum(2);
	Pulleys[0].Location = FVector(-75.f, 0.f, 0.f);
	Pulleys[1].Location = FVector(75.f, 0.f, 0.f);
}

float APulleySystem::GetRopeTension() const
{
	return SimShared.IsValid() ? SimShared->TensionN.load() : 0.f;
}

// ---------------------------------------------------------------------------
// Ends and pulleys
// ---------------------------------------------------------------------------

UPrimitiveComponent* APulleySystem::ResolveEndComponent(const FPulleyRopeEnd& End) const
{
	AActor* A = End.Actor;
	if (!A) { return nullptr; }

	TArray<UPrimitiveComponent*> Prims;
	A->GetComponents<UPrimitiveComponent>(Prims);
	if (!End.ComponentName.IsNone())
	{
		for (UPrimitiveComponent* P : Prims)
		{
			if (P && P->GetFName() == End.ComponentName) { return P; }
		}
	}
	if (UPrimitiveComponent* RootPrim = Cast<UPrimitiveComponent>(A->GetRootComponent()))
	{
		return RootPrim;
	}
	for (UPrimitiveComponent* P : Prims)
	{
		if (P && P->BodyInstance.bSimulatePhysics) { return P; }
	}
	for (UPrimitiveComponent* P : Prims)
	{
		if (P && P->GetCollisionEnabled() != ECollisionEnabled::NoCollision) { return P; }
	}
	return Prims.Num() > 0 ? Prims[0] : nullptr;
}

FVector APulleySystem::GetEndAttachWorld(const FPulleyRopeEnd& End) const
{
	UPrimitiveComponent* Comp = ResolveEndComponent(End);
	if (!Comp)
	{
		return End.Actor ? End.Actor->GetActorTransform().TransformPosition(End.Offset) : GetActorLocation();
	}
	FTransform T = (!End.SocketName.IsNone() && Comp->DoesSocketExist(End.SocketName))
		? Comp->GetSocketTransform(End.SocketName)
		: Comp->GetComponentTransform();
	return T.GetLocation() + T.GetRotation().RotateVector(End.Offset);
}

FVector APulleySystem::GetEndAttachRuntime(int32 Index) const
{
	const FEndRuntime& R = Ends[Index];
	if (R.Body && R.Comp.IsValid())
	{
		const FTransform BT = R.Body->GetUnrealWorldTransform();
		return BT.GetLocation() + BT.GetRotation().RotateVector(R.LocalAttach);
	}
	return GetEndAttachWorld(Index == 0 ? StartEnd : EndEnd);
}

USceneComponent* APulleySystem::GetPulleyWheelComponent(int32 Index) const
{
	if (!Pulleys.IsValidIndex(Index)) { return nullptr; }
	const FPulleyEntry& P = Pulleys[Index];
	if (P.PulleyActor)
	{
		if (!P.WheelComponentName.IsNone())
		{
			TArray<USceneComponent*> Comps;
			P.PulleyActor->GetComponents<USceneComponent>(Comps);
			for (USceneComponent* C : Comps)
			{
				if (C && C->GetFName() == P.WheelComponentName) { return C; }
			}
		}
		return P.PulleyActor->GetRootComponent();
	}
	return WheelPivots.IsValidIndex(Index) ? WheelPivots[Index].Get() : nullptr;
}

FVector APulleySystem::GetPulleyCenter(int32 Index) const
{
	if (bCentersCached && CachedCenters.IsValidIndex(Index)) { return CachedCenters[Index]; }
	const FPulleyEntry& P = Pulleys[Index];
	if (P.PulleyActor)
	{
		if (USceneComponent* Wheel = GetPulleyWheelComponent(Index)) { return Wheel->GetComponentLocation(); }
		return P.PulleyActor->GetActorLocation();
	}
	return GetActorTransform().TransformPosition(P.Location);
}

bool APulleySystem::GatherGeometry(const FVector& StartPoint, const FVector& EndPoint, FPulleyGeometry& G) const
{
	const int32 N = Pulleys.Num();
	if (N == 0) { return false; }

	G.Radius = PulleyRadius;
	G.Centers.SetNum(N);
	for (int32 i = 0; i < N; ++i) { G.Centers[i] = GetPulleyCenter(i); }

	// Plane: user override, else Newell normal of the polyline start -> wheels -> end, else the actor's right vector.
	FVector Normal = RopePlaneNormal;
	if (Normal.IsNearlyZero())
	{
		TArray<FVector, TInlineAllocator<10>> Poly;
		Poly.Add(StartPoint);
		Poly.Append(G.Centers);
		Poly.Add(EndPoint);
		FVector Sum = FVector::ZeroVector;
		for (int32 i = 0; i < Poly.Num(); ++i)
		{
			const FVector& A = Poly[i];
			const FVector& B = Poly[(i + 1) % Poly.Num()];
			Sum.X += (A.Y - B.Y) * (A.Z + B.Z);
			Sum.Y += (A.Z - B.Z) * (A.X + B.X);
			Sum.Z += (A.X - B.X) * (A.Y + B.Y);
		}
		Normal = Sum;
		if (Normal.SizeSquared() < 1.0) { Normal = GetActorRightVector(); }
	}
	Normal.Normalize();
	G.N = Normal;
	G.U = FVector::VectorPlaneProject(FVector::ForwardVector, Normal);
	if (!G.U.Normalize())
	{
		G.U = FVector::VectorPlaneProject(FVector::UpVector, Normal).GetSafeNormal();
	}
	G.V = FVector::CrossProduct(G.N, G.U);

	// Turn direction at each wheel, measured from the current layout.
	G.Signs.SetNum(N);
	for (int32 i = 0; i < N; ++i)
	{
		const FVector Prev = i == 0 ? StartPoint : G.Centers[i - 1];
		const FVector Next = i == N - 1 ? EndPoint : G.Centers[i + 1];
		const FVector2D A = PulleyMath::To2D(G, G.Centers[i]) - PulleyMath::To2D(G, Prev);
		const FVector2D B = PulleyMath::To2D(G, Next) - PulleyMath::To2D(G, G.Centers[i]);
		const double Cr = PulleyMath::Cross(A, B);
		// Straight-through layouts default to clockwise (rope over the top when running in +U).
		G.Signs[i] = Cr > KINDA_SMALL_NUMBER ? 1.f : -1.f;
	}
	return true;
}

// ---------------------------------------------------------------------------
// Components
// ---------------------------------------------------------------------------

void APulleySystem::DestroyComponents()
{
	for (USplineMeshComponent* C : RopePieces) { if (IsValid(C)) { C->DestroyComponent(); } }
	for (UStaticMeshComponent* C : WheelParts) { if (IsValid(C)) { C->DestroyComponent(); } }
	for (USceneComponent* C : WheelPivots) { if (IsValid(C)) { C->DestroyComponent(); } }
	RopePieces.Reset();
	WheelParts.Reset();
	WheelPivots.Reset();
}

void APulleySystem::BuildComponents()
{
	DestroyComponents();
	const int32 N = Pulleys.Num();
	if (N == 0) { return; }

	const EComponentCreationMethod Method = HasActorBegunPlay() ? EComponentCreationMethod::Instance : EComponentCreationMethod::UserConstructionScript;

	// Rope: 2 hangs + (N-1) spans + N * ArcPieces.
	const int32 NumPieces = 2 + (N - 1) + N * ArcPieces;
	const float VisScale = RopeThickness / 100.f;
	for (int32 i = 0; i < NumPieces; ++i)
	{
		USplineMeshComponent* Vis = NewObject<USplineMeshComponent>(this);
		Vis->CreationMethod = Method;
		Vis->SetMobility(EComponentMobility::Movable);
		Vis->SetupAttachment(Root);
		Vis->SetUsingAbsoluteLocation(true);
		Vis->SetUsingAbsoluteRotation(true);
		Vis->SetUsingAbsoluteScale(true);
		Vis->SetRelativeTransform(FTransform::Identity);
		Vis->SetStaticMesh(RopeMesh);
		if (RopeMaterial) { Vis->SetMaterial(0, RopeMaterial); }
		Vis->SetForwardAxis(ESplineMeshAxis::Z, false);
		Vis->SetStartScale(FVector2D(VisScale, VisScale), false);
		Vis->SetEndScale(FVector2D(VisScale, VisScale), false);
		Vis->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		Vis->SetCanEverAffectNavigation(false);
		Vis->RegisterComponent();
		RopePieces.Add(Vis);
	}

	// Wheels for pulleys without an actor: pivot (unscaled, Z = axle) + disc + two spokes. No collision.
	WheelPivots.SetNum(N);
	for (int32 i = 0; i < N; ++i)
	{
		if (Pulleys[i].PulleyActor) { WheelPivots[i] = nullptr; continue; }

		USceneComponent* Pivot = NewObject<USceneComponent>(this);
		Pivot->CreationMethod = Method;
		Pivot->SetMobility(EComponentMobility::Movable);
		Pivot->SetupAttachment(Root);
		Pivot->SetUsingAbsoluteRotation(true);
		Pivot->SetUsingAbsoluteScale(true);
		Pivot->SetRelativeLocation(Pulleys[i].Location);
		Pivot->RegisterComponent();
		WheelPivots[i] = Pivot;

		auto AddPart = [&](UStaticMesh* Mesh, const FVector& Scale, UMaterialInterface* Mat)
		{
			UStaticMeshComponent* Part = NewObject<UStaticMeshComponent>(this);
			Part->CreationMethod = Method;
			Part->SetMobility(EComponentMobility::Movable);
			Part->SetupAttachment(Pivot);
			Part->SetStaticMesh(Mesh);
			if (Mat) { Part->SetMaterial(0, Mat); }
			Part->SetRelativeScale3D(Scale);
			Part->SetCollisionEnabled(ECollisionEnabled::NoCollision);
			Part->SetCanEverAffectNavigation(false);
			Part->RegisterComponent();
			WheelParts.Add(Part);
		};
		const float D = PulleyRadius * 2.f / 100.f;
		const float W = WheelWidth / 100.f;
		UMaterialInterface* SpokeMat = SpokeMaterial ? SpokeMaterial.Get() : WheelMaterial.Get();
		AddPart(WheelMesh, FVector(D, D, W), WheelMaterial);
		// Spokes stick out of both faces so they stay visible.
		AddPart(SpokeMesh, FVector(D * 0.95f, D * 0.12f, W * 1.6f), SpokeMat);
		AddPart(SpokeMesh, FVector(D * 0.12f, D * 0.95f, W * 1.6f), SpokeMat);
	}
}

void APulleySystem::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);
	bCentersCached = false;
	CachedCenters.Reset();
	BuildComponents();

	const FVector PS = GetEndAttachWorld(StartEnd);
	const FVector PE = GetEndAttachWorld(EndEnd);
	FPulleyGeometry G;
	if (GatherGeometry(PS, PE, G))
	{
		PlaneNormal = G.N;
		WheelBaseRotations.SetNum(Pulleys.Num());
		for (int32 i = 0; i < Pulleys.Num(); ++i)
		{
			WheelBaseRotations[i] = FRotationMatrix::MakeFromZ(G.N).ToQuat();
		}
		UpdateWheels(0.f);
	}
	UpdateRopeVisual(PS, PE);
}

// ---------------------------------------------------------------------------
// Visuals
// ---------------------------------------------------------------------------

void APulleySystem::UpdateRopeVisual(const FVector& StartPoint, const FVector& EndPoint)
{
	// During play the layout is frozen (SimShared->Geom is written once in BeginPlay and only read afterwards).
	FPulleyGeometry G;
	if (SimShared.IsValid() && SimShared->Geom.Centers.Num() > 0) { G = SimShared->Geom; }
	else if (!GatherGeometry(StartPoint, EndPoint, G)) { return; }

	FPulleyPath Path;
	PulleyMath::ComputePath(G, StartPoint, EndPoint, Path);

	const int32 N = G.Centers.Num();
	if (RopePieces.Num() != 2 + (N - 1) + N * ArcPieces) { return; }

	int32 Piece = 0;
	auto SetPiece = [&](const FVector& A, const FVector& B)
	{
		USplineMeshComponent* Vis = RopePieces[Piece++];
		if (!IsValid(Vis)) { return; }
		const FVector T = B - A;
		Vis->SetStartAndEnd(A, T, B, T, true);
	};

	SetPiece(StartPoint, Path.In[0]);
	for (int32 i = 0; i < N; ++i)
	{
		// Arc: rotate the radius from In about the axle by the wrap angle, in the wheel's turn direction.
		const FVector C = G.Centers[i];
		const FVector R0 = Path.In[i] - C;
		FVector Prev = Path.In[i];
		for (int32 j = 1; j <= ArcPieces; ++j)
		{
			const float A = G.Signs[i] * Path.Arc[i] * j / ArcPieces;
			const FVector P = j == ArcPieces ? Path.Out[i] : C + FQuat(G.N, A).RotateVector(R0);
			SetPiece(Prev, P);
			Prev = P;
		}
		if (i + 1 < N) { SetPiece(Path.Out[i], Path.In[i + 1]); }
	}
	SetPiece(Path.Out[N - 1], EndPoint);
}

void APulleySystem::UpdateWheels(float Travel)
{
	for (int32 i = 0; i < Pulleys.Num(); ++i)
	{
		USceneComponent* Wheel = GetPulleyWheelComponent(i);
		if (!Wheel || !WheelBaseRotations.IsValidIndex(i)) { continue; }
		if (Pulleys[i].PulleyActor && Wheel->Mobility != EComponentMobility::Movable) { continue; }
		const float Sign = WrapSigns.IsValidIndex(i) ? WrapSigns[i] : -1.f;
		const float Angle = Sign * Travel / FMath::Max(PulleyRadius, 1.f);
		Wheel->SetWorldRotation(FQuat(PlaneNormal, Angle) * WheelBaseRotations[i]);
	}
}

// ---------------------------------------------------------------------------
// Play
// ---------------------------------------------------------------------------

void APulleySystem::BeginPlay()
{
	Super::BeginPlay();

	if (Pulleys.Num() == 0)
	{
		UE_LOG(LogPulley, Warning, TEXT("%s: no pulleys, nothing to simulate."), *GetName());
		return;
	}
	if (RopePieces.Num() == 0) { BuildComponents(); }

	// Cache pulley centres (fixed) and the base rotation of every wheel.
	bCentersCached = false;
	CachedCenters.Reset();
	for (int32 i = 0; i < Pulleys.Num(); ++i) { CachedCenters.Add(GetPulleyCenter(i)); }
	bCentersCached = true;

	// Resolve the bodies at both ends.
	const FPulleyRopeEnd* Defs[2] = { &StartEnd, &EndEnd };
	FVector Attach[2];
	for (int32 k = 0; k < 2; ++k)
	{
		FEndRuntime& R = Ends[k];
		R = FEndRuntime();
		Attach[k] = GetEndAttachWorld(*Defs[k]);
		UPrimitiveComponent* Comp = ResolveEndComponent(*Defs[k]);
		if (!Comp)
		{
			UE_LOG(LogPulley, Warning, TEXT("%s: %s end has no actor/primitive; it acts as a fixed point."), *GetName(), k == 0 ? TEXT("Start") : TEXT("End"));
			continue;
		}
		R.Comp = Comp;
		R.Body = Comp->GetBodyInstance(Defs[k]->SocketName);
		if (!R.Body) { R.Body = Comp->GetBodyInstance(); }
		if (R.Body)
		{
			FTransform BT = R.Body->GetUnrealWorldTransform();
			BT.RemoveScaling();
			R.LocalAttach = BT.InverseTransformPosition(Attach[k]);
		}
		if (!Comp->IsSimulatingPhysics(Defs[k]->SocketName))
		{
			UE_LOG(LogPulley, Warning, TEXT("%s: %s end (%s) does not simulate physics; it acts as a fixed point."),
				*GetName(), k == 0 ? TEXT("Start") : TEXT("End"), *GetNameSafe(Defs[k]->Actor));
		}
	}

	FPulleyGeometry G;
	GatherGeometry(Attach[0], Attach[1], G);
	PlaneNormal = G.N;
	WrapSigns = G.Signs;

	WheelBaseRotations.SetNum(Pulleys.Num());
	for (int32 i = 0; i < Pulleys.Num(); ++i)
	{
		USceneComponent* Wheel = GetPulleyWheelComponent(i);
		WheelBaseRotations[i] = (Pulleys[i].PulleyActor && Wheel) ? Wheel->GetComponentQuat() : FRotationMatrix::MakeFromZ(G.N).ToQuat();
		if (Pulleys[i].PulleyActor && Wheel && Wheel->Mobility != EComponentMobility::Movable)
		{
			UE_LOG(LogPulley, Warning, TEXT("%s: pulley %d actor %s is not Movable, its wheel will not spin."), *GetName(), i, *GetNameSafe(Pulleys[i].PulleyActor));
		}
	}

	FPulleyPath Path;
	PulleyMath::ComputePath(G, Attach[0], Attach[1], Path);
	FreeLength = RopeLength > 0.f ? RopeLength : Path.Total;
	TravelRef = (Path.SideEnd - Path.SideStart) * 0.5f;
	RopeTravel = 0.f;
	if (RopeLength > 0.f && RopeLength < Path.Total - 1.f)
	{
		UE_LOG(LogPulley, Warning, TEXT("%s: RopeLength %.1f is shorter than the current path %.1f; the ends will be yanked on the first frame."), *GetName(), RopeLength, Path.Total);
	}
	UE_LOG(LogPulley, Log, TEXT("%s: %d pulleys, rope %.1f cm (start hang %.1f, end hang %.1f, wrap %.0f deg), friction ratio %.3f"),
		*GetName(), Pulleys.Num(), FreeLength, Path.HangStart, Path.HangEnd, FMath::RadiansToDegrees(Path.TotalWrap), FMath::Exp(PulleyFriction * Path.TotalWrap));

	// Physics-thread callback.
	SimShared = MakeShared<FPulleySimShared, ESPMode::ThreadSafe>();
	{
		FScopeLock ScopeLock(&SimShared->Lock);
		SimShared->Geom = G;
		SimShared->FreeLength = FreeLength;
		SimShared->MinHang = MinHangLength;
		SimShared->StiffnessHz = RopeStiffnessHz;
		SimShared->DampingRatio = RopeDampingRatio;
		SimShared->Friction = PulleyFriction;
		for (int32 k = 0; k < 2; ++k)
		{
			SimShared->FixedAttach[k] = Attach[k];
			SimShared->LocalAttach[k] = Ends[k].LocalAttach;
		}
	}
	PushBodiesToSim();

	if (UWorld* World = GetWorld())
	{
		if (FPhysScene* Scene = World->GetPhysicsScene())
		{
			if (Chaos::FPhysicsSolver* Solver = Scene->GetSolver())
			{
				SimCallback = Solver->CreateAndRegisterSimCallbackObject_External<FPulleySimCallback>();
				SimCallback->Shared = SimShared;
			}
		}
	}
	if (!SimCallback)
	{
		UE_LOG(LogPulley, Error, TEXT("%s: could not register the physics callback; the rope will not pull."), *GetName());
	}
}

void APulleySystem::PushBodiesToSim()
{
	if (!SimShared.IsValid()) { return; }
	FScopeLock ScopeLock(&SimShared->Lock);
	bool bAny = false;
	for (int32 k = 0; k < 2; ++k)
	{
		const FEndRuntime& R = Ends[k];
		Chaos::FSingleParticlePhysicsProxy* Proxy = nullptr;
		if (R.Body && R.Comp.IsValid() && R.Comp->IsSimulatingPhysics())
		{
			Proxy = R.Body->GetPhysicsActor();
		}
		SimShared->Proxy[k] = Proxy;
		bAny |= Proxy != nullptr;
		if (!Proxy && R.Comp.IsValid())
		{
			// Not simulating (e.g. kinematic): follow it as a moving fixed point.
			SimShared->FixedAttach[k] = GetEndAttachRuntime(k);
		}
	}
	SimShared->bActive = bAny;
}

void APulleySystem::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (SimCallback)
	{
		if (SimShared.IsValid())
		{
			FScopeLock ScopeLock(&SimShared->Lock);
			SimShared->bActive = false;
			SimShared->Proxy[0] = SimShared->Proxy[1] = nullptr;
		}
		if (UWorld* World = GetWorld())
		{
			if (FPhysScene* Scene = World->GetPhysicsScene())
			{
				if (Chaos::FPhysicsSolver* Solver = Scene->GetSolver())
				{
					Solver->UnregisterAndFreeSimCallbackObject_External(SimCallback);
				}
			}
		}
		SimCallback = nullptr;
	}
	Super::EndPlay(EndPlayReason);
}

void APulleySystem::CheckTriggers()
{
	const bool bEnabled[2] = { bTriggerStartEnd, bTriggerEndEnd };
	const float Heights[2] = { StartTriggerHeight, EndTriggerHeight };
	const FPulleyRopeEnd* Defs[2] = { &StartEnd, &EndEnd };
	for (int32 k = 0; k < 2; ++k)
	{
		FEndRuntime& R = Ends[k];
		if (!bEnabled[k] || !R.Comp.IsValid()) { continue; }
		const float Z = R.Comp->GetComponentLocation().Z;
		if (R.bArmed && Z >= Heights[k])
		{
			R.bArmed = false;
			UE_LOG(LogPulley, Log, TEXT("%s: %s end (%s) reached height %.1f (z = %.1f)"), *GetName(), k == 0 ? TEXT("Start") : TEXT("End"), *GetNameSafe(Defs[k]->Actor), Heights[k], Z);
			OnEndReachedHeight.Broadcast(Defs[k]->Actor, k == 0, Heights[k]);
		}
		else if (!R.bArmed && !bTriggerOnce && Z < Heights[k] - 5.f)
		{
			R.bArmed = true;
		}
	}
}

void APulleySystem::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	if (Pulleys.Num() == 0 || !SimShared.IsValid()) { return; }

	// Bodies can be recreated (e.g. physics toggled at runtime): refresh proxies every frame.
	for (int32 k = 0; k < 2; ++k)
	{
		if (Ends[k].Comp.IsValid())
		{
			const FPulleyRopeEnd& Def = k == 0 ? StartEnd : EndEnd;
			Ends[k].Body = Ends[k].Comp->GetBodyInstance(Def.SocketName);
			if (!Ends[k].Body) { Ends[k].Body = Ends[k].Comp->GetBodyInstance(); }
		}
		else
		{
			Ends[k].Body = nullptr;
		}
	}
	PushBodiesToSim();

	const FVector PS = GetEndAttachRuntime(0);
	const FVector PE = GetEndAttachRuntime(1);
	UpdateRopeVisual(PS, PE);

	const FPulleyGeometry& G = SimShared->Geom;
	if (G.Centers.Num() > 0)
	{
		FPulleyPath Path;
		PulleyMath::ComputePath(G, PS, PE, Path);
		RopeTravel = (Path.SideEnd - Path.SideStart) * 0.5f - TravelRef;
		UpdateWheels(RopeTravel);

		if (CVarPulleyDebug.GetValueOnGameThread() != 0)
		{
			DebugTimer += DeltaSeconds;
			if (DebugTimer >= 0.25f)
			{
				DebugTimer = 0.f;
				const float ZS = Ends[0].Comp.IsValid() ? Ends[0].Comp->GetComponentLocation().Z : 0.f;
				const float ZE = Ends[1].Comp.IsValid() ? Ends[1].Comp->GetComponentLocation().Z : 0.f;
				UE_LOG(LogPulley, Log, TEXT("PulleyDbg %s t=%.2f startZ=%.1f endZ=%.1f hangS=%.1f hangE=%.1f stretch=%.2f tension=%.1fN travel=%.1f"),
					*GetName(), GetWorld()->GetTimeSeconds(), ZS, ZE, Path.HangStart, Path.HangEnd, Path.Total - FreeLength, GetRopeTension(), RopeTravel);
			}
		}
	}

	CheckTriggers();
}
