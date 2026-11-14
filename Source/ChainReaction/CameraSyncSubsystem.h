// Shared registry for the camera sync tools (BP_SyncTrigger / BP_CameraSync).
// Triggers and camera syncs find each other only by trigger name, through this subsystem.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "CameraSyncSubsystem.generated.h"

class ASyncTrigger;
class ACameraSync;

CHAINREACTION_API DECLARE_LOG_CATEGORY_EXTERN(LogCameraSync, Log, All);

UCLASS()
class CHAINREACTION_API UCameraSyncSubsystem : public UWorldSubsystem
{
	GENERATED_BODY()

public:
	void RegisterTrigger(ASyncTrigger* Trigger);
	void UnregisterTrigger(ASyncTrigger* Trigger);
	void RegisterSync(ACameraSync* Sync);
	void UnregisterSync(ACameraSync* Sync);

	/** True if any trigger with this name has fired. */
	UFUNCTION(BlueprintPure, Category = "Camera Sync")
	bool IsTriggered(FName TriggerName) const;

	/** True if a trigger with this name is in the level. */
	UFUNCTION(BlueprintPure, Category = "Camera Sync")
	bool HasTrigger(FName TriggerName) const;

	/** The first trigger with this name, or null. Prefers one that has fired. */
	ASyncTrigger* FindTrigger(FName TriggerName) const;

	/** All triggers in the level. */
	TArray<ASyncTrigger*> GetTriggers() const;

	/** Current frame (display rate) of the sequence a BP_CameraSync controls, or else of any playing Level Sequence. */
	bool GetSequenceFrame(int32& OutFrame) const;

private:
	TArray<TWeakObjectPtr<ASyncTrigger>> Triggers;
	TArray<TWeakObjectPtr<ACameraSync>> Syncs;
};
