// Physics rope: a chain of capsule rigid bodies joined by physics constraints,
// rendered with spline meshes that follow the simulated segments.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "InputCoreTypes.h"
#include "PhysicsRope.generated.h"

class UCapsuleComponent;
class USplineMeshComponent;
class UPhysicsConstraintComponent;
class UPrimitiveComponent;
class UStaticMesh;
class UMaterialInterface;

UENUM(BlueprintType)
enum class ERopeAnchorMode : uint8
{
	Free          UMETA(DisplayName = "Free (not attached)"),
	WorldPoint    UMETA(DisplayName = "Fixed World Point"),
	AttachToActor UMETA(DisplayName = "Attach To Actor"),
};

/** How one end of the rope is held. The anchor location is the StartPoint / EndPoint widget. */
USTRUCT(BlueprintType)
struct FRopeAnchor
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope")
	ERopeAnchorMode Mode = ERopeAnchorMode::Free;

	/** Actor to attach to. Its root primitive (or ComponentName) must simulate physics or be static/movable collision. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope", meta = (EditCondition = "Mode == ERopeAnchorMode::AttachToActor", EditConditionHides))
	TObjectPtr<AActor> AttachActor = nullptr;

	/** Optional: name of the primitive component on AttachActor. Empty = root primitive, else first primitive found. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope", meta = (EditCondition = "Mode == ERopeAnchorMode::AttachToActor", EditConditionHides))
	FName ComponentName;

	/** Optional: bone name when attaching to a skeletal mesh. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope", meta = (EditCondition = "Mode == ERopeAnchorMode::AttachToActor", EditConditionHides))
	FName BoneName;
};

DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FOnRopeCut, int32, JointIndex, FVector, Location);

UCLASS(Blueprintable)
class CHAINREACTION_API APhysicsRope : public AActor
{
	GENERATED_BODY()

public:
	APhysicsRope();

	// ---------------- Shape ----------------

	/** Total rope length in cm. If longer than the start-end distance the rope is laid out with a sag. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope", meta = (ClampMin = "10.0", UIMin = "10.0"))
	float RopeLength = 300.f;

	/** Number of physics segments. More = smoother but heavier and less stable. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope", meta = (ClampMin = "2", ClampMax = "200", UIMin = "2", UIMax = "100"))
	int32 NumSegments = 20;

	/** Rope diameter in cm (visual and collision). */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope", meta = (ClampMin = "0.2", UIMin = "0.5", UIMax = "20.0"))
	float RopeThickness = 3.f;

	/** Mass of each segment in kg. Heavier segments stretch less under heavy loads. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope", meta = (ClampMin = "0.01", UIMin = "0.05", UIMax = "5.0"))
	float SegmentMass = 0.3f;

	// ---------------- Anchors ----------------

	/** Rope start, relative to the actor. Drag the widget in the viewport. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Anchors", meta = (MakeEditWidget = "true"))
	FVector StartPoint = FVector::ZeroVector;

	/** Rope end, relative to the actor. Drag the widget in the viewport. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Anchors", meta = (MakeEditWidget = "true"))
	FVector EndPoint = FVector(0.f, 0.f, -300.f);

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Anchors")
	FRopeAnchor StartAnchor;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Anchors")
	FRopeAnchor EndAnchor;

	// ---------------- Physics tuning ----------------

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Physics", meta = (ClampMin = "0.0"))
	float LinearDamping = 0.2f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Physics", meta = (ClampMin = "0.0"))
	float AngularDamping = 2.0f;

	/** Max bend between two neighbouring segments (degrees). */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Physics", meta = (ClampMin = "5.0", ClampMax = "90.0"))
	float MaxBendPerJoint = 60.f;

	/** Per-body solver iterations for rope segments (overrides project settings). Higher = stiffer, less stretch. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Physics", meta = (ClampMin = "1", ClampMax = "64"))
	int32 PositionIterations = 16;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Physics", meta = (ClampMin = "1", ClampMax = "32"))
	int32 VelocityIterations = 4;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Physics", meta = (ClampMin = "0", ClampMax = "8"))
	int32 ProjectionIterations = 2;

	/**
	 * Chaos joint mass conditioning: artificially narrows the mass ratio between joined bodies.
	 * Off by default: it lets the rope push heavy attached objects as if they were light (e.g. dragging
	 * a heavy crate off a shelf). Turn on only if a very light rope must hold a heavy load in mid-air.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Physics")
	bool bUseMassConditioning = false;

	// ---------------- Visual ----------------

	/** Mesh deformed along each segment. Must be modelled along +Z (like /Engine/BasicShapes/Cylinder). */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Visual")
	TObjectPtr<UStaticMesh> RopeMesh;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Visual")
	TObjectPtr<UMaterialInterface> RopeMaterial;

	// ---------------- Debug ----------------

	/** If set, pressing this key during Play cuts the rope at DebugCutJoint. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Debug")
	FKey DebugCutKey;

	/** Joint cut by the debug key / auto cut. -1 = middle of the rope. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Debug")
	int32 DebugCutJoint = -1;

	/** If > 0, the rope cuts itself at DebugCutJoint this many seconds after BeginPlay (for automated tests). */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Debug", meta = (ClampMin = "0.0"))
	float DebugAutoCutDelay = 0.f;

	// ---------------- API ----------------

	/** Fired whenever a joint is broken. JointIndex 0 = start anchor, NumSegments = end anchor. */
	UPROPERTY(BlueprintAssignable, Category = "Rope")
	FOnRopeCut OnRopeCut;

	/** Cut the rope at the intact joint closest to WorldLocation. MaxDistance < 0 = no limit. Returns true if a joint was cut. */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	bool CutAtLocation(FVector WorldLocation, float MaxDistance = -1.f);

	/** Cut a specific joint. 0 = start anchor, 1..NumSegments-1 = between segments, NumSegments = end anchor. */
	UFUNCTION(BlueprintCallable, Category = "Rope")
	bool CutAtJoint(int32 JointIndex);

	UFUNCTION(BlueprintPure, Category = "Rope")
	int32 GetNumJoints() const { return Joints.Num(); }

	UFUNCTION(BlueprintPure, Category = "Rope")
	bool IsJointIntact(int32 JointIndex) const;

	/** Current world location of a joint (follows the simulation). */
	UFUNCTION(BlueprintPure, Category = "Rope")
	FVector GetJointLocation(int32 JointIndex) const;

	/** Physics body of a segment, e.g. to apply forces or test overlaps (fire). */
	UFUNCTION(BlueprintPure, Category = "Rope")
	UCapsuleComponent* GetSegment(int32 SegmentIndex) const;

	virtual void OnConstruction(const FTransform& Transform) override;
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaSeconds) override;

protected:
	UPROPERTY(VisibleAnywhere, Category = "Rope")
	TObjectPtr<USceneComponent> Root;

	/** Physics bodies, start to end. */
	UPROPERTY()
	TArray<TObjectPtr<UCapsuleComponent>> Segments;

	/** Visual pieces, one per segment. */
	UPROPERTY()
	TArray<TObjectPtr<USplineMeshComponent>> Visuals;

	/** Constraints indexed by joint: 0 = start anchor, i = between segment i-1 and i, N = end anchor. Null = no joint. */
	UPROPERTY(Transient)
	TArray<TObjectPtr<UPhysicsConstraintComponent>> Joints;

	UPROPERTY(Transient)
	TArray<bool> JointBroken;

	/** Length of one segment, computed at build time. */
	UPROPERTY()
	float SegmentLength = 0.f;

private:
	void BuildRope();
	void DestroyRope();
	void CreateConstraints();
	void ComputeInitialPoints(TArray<FVector>& OutPoints) const;
	/** Child = Component1, Parent = Component2 (FConstraintInstance convention). Parent may be null (= world). */
	UPhysicsConstraintComponent* MakeConstraint(const FVector& Location, const FVector& RopeDir, UPrimitiveComponent* Child, FName ChildBone, UPrimitiveComponent* Parent, FName ParentBone, bool bIsAnchor);
	UPrimitiveComponent* ResolveAnchorComponent(const FRopeAnchor& Anchor) const;
	void UpdateVisuals();
	void GetSegmentEnds(int32 Index, FVector& OutStart, FVector& OutEnd) const;
	void TryBindDebugInput();
	void HandleDebugCut();
	int32 ResolveDebugJoint() const;

	bool bDebugInputBound = false;
	FTimerHandle AutoCutTimer;
};
