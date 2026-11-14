// Domino look variants for BP_DominoChain.
//
// Each domino is a hidden UBoxComponent (the physics body, sized by DominoSize) with a
// collision-free UStaticMeshComponent attached for the look. These helpers pick a variant
// per domino and set up both components, so the physics never depends on the chosen mesh.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "DominoVisuals.generated.h"

class UBoxComponent;
class UStaticMesh;
class UStaticMeshComponent;
class UMaterialInterface;
class UPhysicalMaterial;

UENUM(BlueprintType)
enum class EDominoVariantMode : uint8
{
	/** Variant 0, 1, 2, ... then start over. */
	Sequential,
	/** Random per domino, fixed by RandomSeed and the domino's index. */
	Random,
	/** Every domino uses SingleVariantIndex. */
	Single
};

/** One look a domino can have. Positions are relative to the centre of the collision box. */
USTRUCT(BlueprintType)
struct FDominoVisualVariant
{
	GENERATED_BODY()

	/** Mesh to show. Empty = engine cube stretched to the collision box (the original look). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Domino")
	TObjectPtr<UStaticMesh> Mesh = nullptr;

	/** Material per slot. Empty entries keep the mesh's own material. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Domino")
	TArray<TObjectPtr<UMaterialInterface>> MaterialOverrides;

	/** Fit the mesh into the collision box from its bounds and stand it on the box's bottom. Off = use Scale. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Domino")
	bool bAutoFit = true;

	/** With AutoFit: stretch each axis to fill the box instead of keeping the mesh's proportions. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Domino", meta = (EditCondition = "bAutoFit"))
	bool bStretchToFill = false;

	/** Extra offset (cm) from the fitted position, or from the box centre when AutoFit is off. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Domino")
	FVector LocationOffset = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Domino")
	FRotator RotationOffset = FRotator::ZeroRotator;

	/** Used when AutoFit is off. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Domino", meta = (EditCondition = "!bAutoFit"))
	FVector Scale = FVector::OneVector;
};

/** Look settings of a whole domino chain (one Details entry on BP_DominoChain). */
USTRUCT(BlueprintType)
struct FDominoVisualSettings
{
	GENERATED_BODY()

	/** Looks to rotate through. Empty = every domino is the original white cube. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Domino")
	TArray<FDominoVisualVariant> Variants;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Domino")
	EDominoVariantMode Mode = EDominoVariantMode::Sequential;

	/** Random mode: same seed = same look for every domino, however the spline or spacing changes. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Domino", meta = (EditCondition = "Mode == EDominoVariantMode::Random"))
	int32 RandomSeed = 0;

	/** Single mode: which variant every domino uses. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Domino", meta = (ClampMin = "0", EditCondition = "Mode == EDominoVariantMode::Single"))
	int32 SingleVariantIndex = 0;

	/** Draw the collision boxes (orange outline) in the editor and in game. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Domino")
	bool bShowCollision = false;

	/** Warn in the Output Log when a look sticks out of its box by more than this fraction of the box size. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Domino", meta = (ClampMin = "0.0", UIMax = "1.0"))
	float OversizeWarningTolerance = 0.1f;
};

UCLASS()
class CHAINREACTION_API UDominoVisualLibrary : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	/** Which variant domino number DominoIndex uses. -1 if there are no variants. */
	UFUNCTION(BlueprintPure, Category = "Domino")
	static int32 PickVariantIndex(EDominoVariantMode Mode, int32 VariantCount, int32 DominoIndex, int32 RandomSeed, int32 SingleVariantIndex);

	/** Sets up the physics body: box of BoxSize (cm), fixed mass, physical material, simulating. */
	UFUNCTION(BlueprintCallable, Category = "Domino")
	static void SetupDominoBody(UBoxComponent* Body, FVector BoxSize, float MassKg, UPhysicalMaterial* PhysMaterial, bool bShowCollision);

	/** Sets up the look: mesh, materials, no collision or physics, transform relative to the box it is attached to. */
	UFUNCTION(BlueprintCallable, Category = "Domino")
	static void SetupDominoVisual(UStaticMeshComponent* Visual, const FDominoVisualVariant& Variant, FVector BoxSize);

	/** Logs a warning for each variant whose mesh sticks out of the box by more than Tolerance (fraction of the box size). */
	UFUNCTION(BlueprintCallable, Category = "Domino", meta = (DefaultToSelf = "Context"))
	static void WarnOversizedVariants(UObject* Context, const TArray<FDominoVisualVariant>& Variants, FVector BoxSize, float Tolerance = 0.1f);

	/** Picks the variant for DominoIndex from Settings and sets up Visual with it. Returns the variant index (-1 = original cube). */
	UFUNCTION(BlueprintCallable, Category = "Domino")
	static int32 SetupDominoVisualFromSettings(UStaticMeshComponent* Visual, const FDominoVisualSettings& Settings, int32 DominoIndex, FVector BoxSize);

	/** WarnOversizedVariants for the variants and tolerance in Settings. */
	UFUNCTION(BlueprintCallable, Category = "Domino", meta = (DefaultToSelf = "Context"))
	static void CheckVisualSettings(UObject* Context, const FDominoVisualSettings& Settings, FVector BoxSize);

	/** Visual transform relative to the box centre. */
	static FTransform ComputeVisualTransform(const FDominoVisualVariant& Variant, const UStaticMesh* Mesh, const FVector& BoxSize);
};
