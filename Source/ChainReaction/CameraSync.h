// Camera sync tool 2: slows a Level Sequence down at chosen frames until a BP_SyncTrigger
// with the matching name has fired, then brings it smoothly back to normal speed.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "CameraSync.generated.h"

class AHUD;
class ALevelSequenceActor;
class UBillboardComponent;
class UCanvas;
class ULevelSequencePlayer;

/** One place in the sequence where the camera waits for a trigger. */
USTRUCT(BlueprintType)
struct FCameraSyncWaitPoint
{
	GENERATED_BODY()

	/** Trigger Name of the BP_SyncTrigger to wait for. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera Sync")
	FName TriggerName;

	/** Sequence frame where the wait starts (the frame number shown in Sequencer). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera Sync", meta = (ClampMin = "0"))
	int32 WaitFrame = 0;

	/** Lowest play speed while waiting (1 = normal, 0 = stop). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera Sync", meta = (ClampMin = "0", ClampMax = "1"))
	float MinPlayRate = 0.f;

	/** Seconds to ease from normal speed down to MinPlayRate. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera Sync", meta = (ClampMin = "0", Units = "s"))
	float SlowDownTime = 0.5f;

	/** Seconds to ease back to normal speed once the trigger fires (or the wait times out). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera Sync", meta = (ClampMin = "0", Units = "s"))
	float RecoverTime = 0.5f;

	/** Give up after this many seconds of waiting and return to normal speed. 0 = wait forever. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera Sync", meta = (ClampMin = "0", Units = "s"))
	float TimeoutSeconds = 10.f;
};

UCLASS(Blueprintable)
class CHAINREACTION_API ACameraSync : public AActor
{
	GENERATED_BODY()

public:
	ACameraSync();

	/** The Level Sequence Actor whose playback speed this controls. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Camera Sync")
	TObjectPtr<ALevelSequenceActor> SequenceActor;

	/** Where to wait and for which trigger. Order does not matter. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Camera Sync", meta = (TitleProperty = "{TriggerName} @ frame {WaitFrame}"))
	TArray<FCameraSyncWaitPoint> WaitPoints;

	/** Show frame, speed, what is being waited for and the state of every trigger in the top-left corner. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera Sync")
	bool bShowDebug = false;

	/** Print each wait, pass, resume and timeout to the Output Log. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera Sync", AdvancedDisplay)
	bool bLogEvents = true;

	/** Current sequence frame (-1 if no sequence). */
	UFUNCTION(BlueprintPure, Category = "Camera Sync")
	int32 GetCurrentFrame() const { return CurrentFrame; }

	/** Current speed multiplier applied to the sequence (1 = normal). */
	UFUNCTION(BlueprintPure, Category = "Camera Sync")
	float GetSpeed() const { return Speed; }

	/** Trigger name being waited for, or None. */
	UFUNCTION(BlueprintPure, Category = "Camera Sync")
	FName GetWaitingFor() const;

	virtual void Tick(float DeltaSeconds) override;

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:
	enum class EPhase : uint8 { Normal, Waiting, Recovering };

	ULevelSequencePlayer* GetPlayer() const;
	void Validate();
	void ResetPoints();
	void StartRamp(float Target, float Duration, float FullRange);
	void UpdateSpeed(float DeltaSeconds);
	void LogEvent(const FString& Text, bool bWarning = false);
	void DrawDebug(AHUD* HUD, UCanvas* Canvas);

	UPROPERTY(VisibleAnywhere, Category = "Camera Sync")
	TObjectPtr<USceneComponent> Root;

#if WITH_EDITORONLY_DATA
	UPROPERTY()
	TObjectPtr<UBillboardComponent> Sprite;
#endif

	EPhase Phase = EPhase::Normal;
	int32 ActiveIndex = INDEX_NONE;
	TArray<int32> Order;       // WaitPoints indices sorted by WaitFrame
	TArray<bool> Reached;
	int32 CurrentFrame = -1;
	int32 LastFrame = -1;
	float Speed = 1.f;
	float BaseRate = 1.f;
	bool bHasBaseRate = false;
	bool bValidated = false;
	float WaitElapsed = 0.f;

	float RampFrom = 1.f;
	float RampTo = 1.f;
	float RampDuration = 0.f;
	float RampTime = 0.f;

	FString LastEvent;
	FDelegateHandle HudHandle;
};
