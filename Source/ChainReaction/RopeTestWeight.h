// Simple physics weight used to test BP_PhysicsRope.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "RopeTestWeight.generated.h"

class UStaticMeshComponent;

UCLASS(Blueprintable)
class CHAINREACTION_API ARopeTestWeight : public AActor
{
	GENERATED_BODY()

public:
	ARopeTestWeight();

	/** Mass in kg. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Weight", meta = (ClampMin = "0.1"))
	float MassKg = 10.f;

	/** Cube edge length in cm. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Weight", meta = (ClampMin = "1.0"))
	float Size = 30.f;

	/** Plain root so the actor's own scale never changes the cube size. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Weight")
	TObjectPtr<USceneComponent> Root;

	/** The physics body. Sized by Size via its relative scale. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Weight")
	TObjectPtr<UStaticMeshComponent> Mesh;

	/** World location of the top-centre of the cube (where a rope should attach). */
	UFUNCTION(BlueprintPure, Category = "Weight")
	FVector GetTopAttachPoint() const;

	virtual void OnConstruction(const FTransform& Transform) override;
};
