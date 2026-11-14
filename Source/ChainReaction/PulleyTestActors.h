// Test-only actors for the PulleyTest level. Not used by BP_PulleySystem.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "PulleyTestActors.generated.h"

class UBoxComponent;
class UStaticMeshComponent;
class UMaterialInterface;
class APulleySystem;

/** Open-top bucket: one simulated box body with welded walls. */
UCLASS(Blueprintable)
class CHAINREACTION_API APulleyTestBucket : public AActor
{
	GENERATED_BODY()

public:
	APulleyTestBucket();

	/** Total mass of the empty bucket in kg. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Bucket", meta = (ClampMin = "0.1"))
	float MassKg = 6.f;

	/** Inner width (cm). */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Bucket", meta = (ClampMin = "10.0"))
	float Width = 40.f;

	/** Wall height (cm). */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Bucket", meta = (ClampMin = "5.0"))
	float Height = 30.f;

	/** Simulated body (the floor of the bucket). Tie the rope here with Offset (0,0,Height+15). */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Bucket")
	TObjectPtr<UBoxComponent> Base;

	UPROPERTY(VisibleAnywhere, Category = "Bucket")
	TArray<TObjectPtr<UBoxComponent>> Walls;

	UPROPERTY(VisibleAnywhere, Category = "Bucket")
	TArray<TObjectPtr<UStaticMeshComponent>> Visuals;

	virtual void OnConstruction(const FTransform& Transform) override;
	virtual void BeginPlay() override;

private:
	void ApplyMass();
};

/** Drops a ball after a delay and reports the pulley's height event. */
UCLASS(Blueprintable)
class CHAINREACTION_API APulleyTestDriver : public AActor
{
	GENERATED_BODY()

public:
	APulleyTestDriver();

	UPROPERTY(EditAnywhere, Category = "Test")
	TObjectPtr<APulleySystem> Pulley;

	/** Actor whose root primitive is the ball. Held still until the drop. */
	UPROPERTY(EditAnywhere, Category = "Test")
	TObjectPtr<AActor> Ball;

	UPROPERTY(EditAnywhere, Category = "Test", meta = (ClampMin = "0.1"))
	float BallMassKg = 6.f;

	UPROPERTY(EditAnywhere, Category = "Test", meta = (ClampMin = "0.0"))
	float DropDelay = 1.f;

	/** Applied to the risen object when the event fires. */
	UPROPERTY(EditAnywhere, Category = "Test")
	TObjectPtr<UMaterialInterface> TriggeredMaterial;

	virtual void BeginPlay() override;

private:
	UFUNCTION()
	void HandleReachedHeight(AActor* EndActor, bool bIsStartEnd, float Height);

	void DropBall();

	FTimerHandle DropTimer;
};
