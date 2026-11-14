// Camera sync tool 1: a box area that records the first time a physics object enters it.
// BP_CameraSync wait points refer to it by TriggerName. Place, scale and name it in the level.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "SyncTrigger.generated.h"

class UBoxComponent;
class UPrimitiveComponent;
class UTextRenderComponent;

DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FOnSyncTriggered, FName, TriggerName, AActor*, ByActor);

UCLASS(Blueprintable)
class CHAINREACTION_API ASyncTrigger : public AActor
{
	GENERATED_BODY()

public:
	ASyncTrigger();

	/** The trigger area. Scale the actor to resize it. Shown in the editor only. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sync Trigger")
	TObjectPtr<UBoxComponent> Box;

	/** Shows TriggerName above the box in the editor. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sync Trigger")
	TObjectPtr<UTextRenderComponent> Label;

	/** Name used by BP_CameraSync wait points to find this trigger. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Sync Trigger")
	FName TriggerName = TEXT("Trigger");

	/** Only this actor counts. Empty = any object that simulates physics. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Sync Trigger")
	TObjectPtr<AActor> WatchedActor;

	/** Fires once, when the trigger is first entered. */
	UPROPERTY(BlueprintAssignable, Category = "Sync Trigger")
	FOnSyncTriggered OnTriggered;

	UFUNCTION(BlueprintPure, Category = "Sync Trigger")
	bool IsTriggered() const { return bTriggered; }

	/** Sequence frame when it fired (-1 = not fired or no sequence playing). */
	UFUNCTION(BlueprintPure, Category = "Sync Trigger")
	int32 GetTriggeredFrame() const { return TriggeredFrame; }

	/** Game time in seconds when it fired. */
	UFUNCTION(BlueprintPure, Category = "Sync Trigger")
	float GetTriggeredTime() const { return TriggeredTime; }

	/** Makes the trigger fire again on the next entry. */
	UFUNCTION(BlueprintCallable, Category = "Sync Trigger")
	void ResetTrigger();

	/** Warns in the Output Log about objects that can never fire this trigger. Runs at BeginPlay. */
	UFUNCTION(BlueprintCallable, Category = "Sync Trigger")
	void CheckDetectability() const;

	virtual void OnConstruction(const FTransform& Transform) override;

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:
	UFUNCTION()
	void HandleBeginOverlap(UPrimitiveComponent* OverlappedComp, AActor* OtherActor, UPrimitiveComponent* OtherComp,
		int32 OtherBodyIndex, bool bFromSweep, const FHitResult& SweepResult);

	bool Accepts(const AActor* OtherActor, const UPrimitiveComponent* OtherComp) const;
	void Fire(AActor* ByActor);

	/** Empty if Comp can generate overlap events with Box, else the reason it cannot. */
	FString WhyCannotOverlap(const UPrimitiveComponent* Comp) const;

	UPROPERTY(VisibleInstanceOnly, Category = "Sync Trigger|Runtime")
	bool bTriggered = false;

	UPROPERTY(VisibleInstanceOnly, Category = "Sync Trigger|Runtime")
	int32 TriggeredFrame = -1;

	UPROPERTY(VisibleInstanceOnly, Category = "Sync Trigger|Runtime")
	float TriggeredTime = 0.f;
};
