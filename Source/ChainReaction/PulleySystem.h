// Pulley system: one rope running over any number of fixed pulleys, joining two physics objects.
//
// The rope is not simulated as rigid bodies. The part between the pulleys never changes length, so
// the only physical rule left is "start hang + end hang <= free length". A physics-thread callback
// enforces it every substep as a very stiff, damped, pull-only spring and applies the resulting
// tension to both end objects (using their own mass and inertia). Pulley friction follows the
// capstan law: the tension ratio across the pulleys is at most e^(mu * total wrap angle).

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "PulleySystem.generated.h"

class UPrimitiveComponent;
class USplineMeshComponent;
class UStaticMeshComponent;
class UStaticMesh;
class UMaterialInterface;
class FPulleySimCallback;
struct FPulleySimShared;
struct FBodyInstance;
struct FPulleyGeometry;

/** One end of the rope. */
USTRUCT(BlueprintType)
struct FPulleyRopeEnd
{
	GENERATED_BODY()

	/** Object the rope is tied to. Pick any actor in the level; it should simulate physics (otherwise it acts as a fixed anchor). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pulley")
	TObjectPtr<AActor> Actor = nullptr;

	/** Optional: primitive component on Actor. Empty = root primitive, else the first simulating primitive. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pulley")
	FName ComponentName;

	/** Optional: socket (or bone) to tie to. Offset is then relative to the socket. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pulley")
	FName SocketName;

	/** Tie point in cm, along the component's (or socket's) local axes, ignoring scale. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pulley")
	FVector Offset = FVector::ZeroVector;
};

/** One fixed pulley. Use a level actor, or leave it empty and place the wheel with Location. */
USTRUCT(BlueprintType)
struct FPulleyEntry
{
	GENERATED_BODY()

	/** Optional pulley actor in the level. Its location (or WheelComponentName's) is the wheel centre, and that part spins. Must be Movable to spin. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pulley")
	TObjectPtr<AActor> PulleyActor = nullptr;

	/** Optional: component of PulleyActor that is the wheel. Empty = root component. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pulley")
	FName WheelComponentName;

	/** Wheel centre relative to this actor, used when PulleyActor is empty (a wheel is drawn here). Drag the widget in the viewport. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pulley", meta = (MakeEditWidget = "true"))
	FVector Location = FVector::ZeroVector;
};

DECLARE_DYNAMIC_MULTICAST_DELEGATE_ThreeParams(FOnPulleyEndReachedHeight, AActor*, EndActor, bool, bIsStartEnd, float, Height);

UCLASS(Blueprintable)
class CHAINREACTION_API APulleySystem : public AActor
{
	GENERATED_BODY()

public:
	APulleySystem();

	// ---------------- Setup ----------------

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Pulley")
	FPulleyRopeEnd StartEnd;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Pulley")
	FPulleyRopeEnd EndEnd;

	/** Pulleys in the order the rope runs over them, from StartEnd to EndEnd. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Pulley")
	TArray<FPulleyEntry> Pulleys;

	/** Wheel radius in cm (all pulleys). */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Pulley", meta = (ClampMin = "1.0", UIMin = "2.0", UIMax = "100.0"))
	float PulleyRadius = 15.f;

	/** Total rope length in cm. <= 0 = use the length measured at Play start (rope exactly taut). */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Pulley")
	float RopeLength = 0.f;

	/** Capstan friction coefficient between rope and pulleys. 0 = ideal frictionless pulleys. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Pulley", meta = (ClampMin = "0.0", UIMax = "1.0"))
	float PulleyFriction = 0.05f;

	/** Shortest hang length in cm (end to pulley). An end pulled closer is stopped there instead of passing through the wheel. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Pulley", meta = (ClampMin = "0.0"))
	float MinHangLength = 10.f;

	// ---------------- Height events ----------------

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Pulley|Events")
	bool bTriggerStartEnd = true;

	/** World Z (cm) the StartEnd object (its physics body) must rise to. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Pulley|Events", meta = (EditCondition = "bTriggerStartEnd"))
	float StartTriggerHeight = 200.f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Pulley|Events")
	bool bTriggerEndEnd = false;

	/** World Z (cm) the EndEnd object (its physics body) must rise to. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Pulley|Events", meta = (EditCondition = "bTriggerEndEnd"))
	float EndTriggerHeight = 200.f;

	/** Fire only the first time. Otherwise it re-arms once the object drops 5 cm below the height. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Pulley|Events")
	bool bTriggerOnce = true;

	/** Fired when an end object rises to its trigger height. */
	UPROPERTY(BlueprintAssignable, Category = "Pulley|Events")
	FOnPulleyEndReachedHeight OnEndReachedHeight;

	// ---------------- Advanced ----------------

	/** Rope stiffness as a spring frequency (Hz). Higher = less stretch. Keep well below the substep rate. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Pulley|Advanced", meta = (ClampMin = "1.0", ClampMax = "60.0"))
	float RopeStiffnessHz = 15.f;

	/** Rope damping ratio. 1 = critically damped (no bounce). */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Pulley|Advanced", meta = (ClampMin = "0.0", ClampMax = "5.0"))
	float RopeDampingRatio = 1.f;

	/** Normal of the plane the rope runs in (= wheel axle). Zero = detect from the end and pulley positions. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Pulley|Advanced")
	FVector RopePlaneNormal = FVector::ZeroVector;

	// ---------------- Visual ----------------

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Pulley|Visual", meta = (ClampMin = "0.2"))
	float RopeThickness = 2.f;

	/** Mesh modelled along +Z (like /Engine/BasicShapes/Cylinder), bent along the rope. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Pulley|Visual")
	TObjectPtr<UStaticMesh> RopeMesh;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Pulley|Visual")
	TObjectPtr<UMaterialInterface> RopeMaterial;

	/** Wheel mesh for pulleys without a PulleyActor (cylinder along Z, 100 cm). */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Pulley|Visual")
	TObjectPtr<UStaticMesh> WheelMesh;

	/** Spoke mesh (cube, 100 cm) so the wheel's rotation is visible. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Pulley|Visual")
	TObjectPtr<UStaticMesh> SpokeMesh;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Pulley|Visual")
	TObjectPtr<UMaterialInterface> WheelMaterial;

	/** Material for the cross spokes. Empty = use WheelMaterial. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Pulley|Visual")
	TObjectPtr<UMaterialInterface> SpokeMaterial;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Pulley|Visual", meta = (ClampMin = "0.5"))
	float WheelWidth = 4.f;

	/** Straight pieces used to draw each wrap around a wheel. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Pulley|Visual", meta = (ClampMin = "2", ClampMax = "32"))
	int32 ArcPieces = 8;

	// ---------------- Read-only state ----------------

	/** Current rope tension in Newtons (averaged over both sides). */
	UFUNCTION(BlueprintPure, Category = "Pulley")
	float GetRopeTension() const;

	/** How far the rope has run over the pulleys since Play started (cm, + = toward EndEnd). */
	UFUNCTION(BlueprintPure, Category = "Pulley")
	float GetRopeTravel() const { return RopeTravel; }

	virtual void OnConstruction(const FTransform& Transform) override;
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void Tick(float DeltaSeconds) override;

protected:
	UPROPERTY(VisibleAnywhere, Category = "Pulley")
	TObjectPtr<USceneComponent> Root;

	UPROPERTY()
	TArray<TObjectPtr<USplineMeshComponent>> RopePieces;

	/** One pivot per pulley without a PulleyActor; wheel + spokes are attached to it. Null for actor pulleys. */
	UPROPERTY()
	TArray<TObjectPtr<USceneComponent>> WheelPivots;

	UPROPERTY()
	TArray<TObjectPtr<UStaticMeshComponent>> WheelParts;

private:
	struct FEndRuntime
	{
		TWeakObjectPtr<UPrimitiveComponent> Comp;
		FBodyInstance* Body = nullptr;
		/** Tie point in the body's unscaled frame. */
		FVector LocalAttach = FVector::ZeroVector;
		bool bArmed = true;
	};

	void BuildComponents();
	void DestroyComponents();
	UPrimitiveComponent* ResolveEndComponent(const FPulleyRopeEnd& End) const;
	FVector GetEndAttachWorld(const FPulleyRopeEnd& End) const;
	FVector GetEndAttachRuntime(int32 Index) const;
	bool GatherGeometry(const FVector& StartPoint, const FVector& EndPoint, FPulleyGeometry& OutGeom) const;
	FVector GetPulleyCenter(int32 Index) const;
	USceneComponent* GetPulleyWheelComponent(int32 Index) const;
	void UpdateRopeVisual(const FVector& StartPoint, const FVector& EndPoint);
	void UpdateWheels(float Travel);
	void CheckTriggers();
	void PushBodiesToSim();

	FEndRuntime Ends[2];
	TArray<FQuat> WheelBaseRotations;
	TArray<FVector> CachedCenters;
	bool bCentersCached = false;
	TArray<float> WrapSigns;
	FVector PlaneNormal = FVector::YAxisVector;
	float FreeLength = 0.f;
	float RopeTravel = 0.f;
	float TravelRef = 0.f;
	float DebugTimer = 0.f;

	TSharedPtr<FPulleySimShared, ESPMode::ThreadSafe> SimShared;
	FPulleySimCallback* SimCallback = nullptr;
};
