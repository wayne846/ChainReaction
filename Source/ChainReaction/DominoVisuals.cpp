#include "DominoVisuals.h"

#include "Components/BoxComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/CollisionProfile.h"
#include "Engine/StaticMesh.h"
#include "Materials/MaterialInterface.h"
#include "PhysicalMaterials/PhysicalMaterial.h"

DEFINE_LOG_CATEGORY_STATIC(LogDomino, Log, All);

namespace DominoVisuals
{
	static UStaticMesh* GetDefaultCube()
	{
		static TWeakObjectPtr<UStaticMesh> Cached;
		if (!Cached.IsValid())
		{
			Cached = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
		}
		return Cached.Get();
	}

	static float SafeRatio(float Want, float Have)
	{
		return Have > KINDA_SMALL_NUMBER ? Want / Have : 1.f;
	}
}

int32 UDominoVisualLibrary::PickVariantIndex(EDominoVariantMode Mode, int32 VariantCount, int32 DominoIndex, int32 RandomSeed, int32 SingleVariantIndex)
{
	if (VariantCount <= 0) { return -1; }
	switch (Mode)
	{
	case EDominoVariantMode::Random:
	{
		// Depends only on seed and index, so moving the spline or adding dominoes never reshuffles existing ones.
		const uint32 Hash = HashCombine(GetTypeHash(RandomSeed), GetTypeHash(DominoIndex * 2654435761u));
		FRandomStream Stream(static_cast<int32>(Hash));
		return Stream.RandRange(0, VariantCount - 1);
	}
	case EDominoVariantMode::Single:
		return FMath::Clamp(SingleVariantIndex, 0, VariantCount - 1);
	case EDominoVariantMode::Sequential:
	default:
		return ((DominoIndex % VariantCount) + VariantCount) % VariantCount;
	}
}

void UDominoVisualLibrary::SetupDominoBody(UBoxComponent* Body, FVector BoxSize, float MassKg, UPhysicalMaterial* PhysMaterial, bool bShowCollision)
{
	if (!Body) { return; }
	Body->SetBoxExtent(BoxSize * 0.5f, false);
	// Same collision as the original cube mesh dominoes.
	Body->SetCollisionProfileName(UCollisionProfile::BlockAllDynamic_ProfileName);
	Body->SetPhysMaterialOverride(PhysMaterial);
	Body->SetMassOverrideInKg(NAME_None, MassKg, true);
	Body->SetSimulatePhysics(true);

	Body->ShapeColor = FColor(255, 140, 0);
	Body->SetLineThickness(1.f);
	Body->SetVisibility(bShowCollision);
	Body->SetHiddenInGame(!bShowCollision);
}

FTransform UDominoVisualLibrary::ComputeVisualTransform(const FDominoVisualVariant& Variant, const UStaticMesh* Mesh, const FVector& BoxSize)
{
	const FQuat Rot = Variant.RotationOffset.Quaternion();
	const bool bDefaultCube = (Variant.Mesh == nullptr);

	if (!bDefaultCube && !Variant.bAutoFit)
	{
		return FTransform(Rot, Variant.LocationOffset, Variant.Scale);
	}
	if (!Mesh)
	{
		return FTransform(Rot, Variant.LocationOffset, FVector::OneVector);
	}

	const FBox Local = Mesh->GetBoundingBox();
	const FVector LocalSize = Local.GetSize();
	FVector Scale;

	if (bDefaultCube || Variant.bStretchToFill)
	{
		// Scale each mesh axis by the box size along the world axis it ends up closest to.
		for (int32 Axis = 0; Axis < 3; ++Axis)
		{
			FVector Dir = FVector::ZeroVector;
			Dir[Axis] = 1.f;
			const FVector R = Rot.RotateVector(Dir).GetAbs();
			const int32 Target = (R.X >= R.Y && R.X >= R.Z) ? 0 : (R.Y >= R.Z ? 1 : 2);
			Scale[Axis] = DominoVisuals::SafeRatio(BoxSize[Target], LocalSize[Axis]);
		}
	}
	else
	{
		const FVector RotSize = Local.TransformBy(FTransform(Rot)).GetSize();
		float S = TNumericLimits<float>::Max();
		for (int32 Axis = 0; Axis < 3; ++Axis)
		{
			if (RotSize[Axis] > KINDA_SMALL_NUMBER) { S = FMath::Min(S, BoxSize[Axis] / RotSize[Axis]); }
		}
		Scale = FVector(S == TNumericLimits<float>::Max() ? 1.f : S);
	}

	// Centre in X/Y, stand on the bottom face of the box.
	const FBox Placed = Local.TransformBy(FTransform(Rot, FVector::ZeroVector, Scale));
	const FVector Center = Placed.GetCenter();
	const FVector Loc(-Center.X, -Center.Y, -BoxSize.Z * 0.5f - Placed.Min.Z);
	return FTransform(Rot, Loc + Variant.LocationOffset, Scale);
}

void UDominoVisualLibrary::SetupDominoVisual(UStaticMeshComponent* Visual, const FDominoVisualVariant& Variant, FVector BoxSize)
{
	if (!Visual) { return; }
	UStaticMesh* Mesh = Variant.Mesh ? Variant.Mesh.Get() : DominoVisuals::GetDefaultCube();

	// Look only: never collides, never simulates, never adds mass.
	Visual->SetSimulatePhysics(false);
	Visual->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	Visual->SetCollisionProfileName(UCollisionProfile::NoCollision_ProfileName);
	Visual->SetGenerateOverlapEvents(false);
	Visual->SetCanEverAffectNavigation(false);
	Visual->SetCastShadow(true);

	Visual->SetStaticMesh(Mesh);
	for (int32 Slot = 0; Slot < Variant.MaterialOverrides.Num(); ++Slot)
	{
		if (UMaterialInterface* Mat = Variant.MaterialOverrides[Slot])
		{
			Visual->SetMaterial(Slot, Mat);
		}
	}
	Visual->SetRelativeTransform(ComputeVisualTransform(Variant, Mesh, BoxSize));
}

void UDominoVisualLibrary::WarnOversizedVariants(UObject* Context, const TArray<FDominoVisualVariant>& Variants, FVector BoxSize, float Tolerance)
{
	const FVector Half = BoxSize * 0.5f;
	for (int32 i = 0; i < Variants.Num(); ++i)
	{
		const FDominoVisualVariant& V = Variants[i];
		const UStaticMesh* Mesh = V.Mesh ? V.Mesh.Get() : DominoVisuals::GetDefaultCube();
		if (!Mesh) { continue; }

		const FBox Placed = Mesh->GetBoundingBox().TransformBy(ComputeVisualTransform(V, Mesh, BoxSize));
		static const TCHAR* AxisNames[3] = { TEXT("X"), TEXT("Y"), TEXT("Z") };
		FString Over;
		for (int32 Axis = 0; Axis < 3; ++Axis)
		{
			const float Out = FMath::Max(Placed.Max[Axis] - Half[Axis], -Half[Axis] - Placed.Min[Axis]);
			if (Out > Tolerance * BoxSize[Axis])
			{
				Over += FString::Printf(TEXT(" %s +%.1f cm"), AxisNames[Axis], Out);
			}
		}
		if (!Over.IsEmpty())
		{
			UE_LOG(LogDomino, Warning, TEXT("%s: visual variant %d (%s) sticks out of the %.0fx%.0fx%.0f cm collision box:%s. It may overlap neighbouring dominoes."),
				*GetNameSafe(Context), i, *Mesh->GetName(), BoxSize.X, BoxSize.Y, BoxSize.Z, *Over);
		}
	}
}

int32 UDominoVisualLibrary::SetupDominoVisualFromSettings(UStaticMeshComponent* Visual, const FDominoVisualSettings& Settings, int32 DominoIndex, FVector BoxSize)
{
	const int32 Index = PickVariantIndex(Settings.Mode, Settings.Variants.Num(), DominoIndex, Settings.RandomSeed, Settings.SingleVariantIndex);
	SetupDominoVisual(Visual, Settings.Variants.IsValidIndex(Index) ? Settings.Variants[Index] : FDominoVisualVariant(), BoxSize);
	return Index;
}

void UDominoVisualLibrary::CheckVisualSettings(UObject* Context, const FDominoVisualSettings& Settings, FVector BoxSize)
{
	WarnOversizedVariants(Context, Settings.Variants, BoxSize, Settings.OversizeWarningTolerance);
}
