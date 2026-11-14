#include "CameraSyncSubsystem.h"

#include "CameraSync.h"
#include "EngineUtils.h"
#include "LevelSequenceActor.h"
#include "LevelSequencePlayer.h"
#include "SyncTrigger.h"

DEFINE_LOG_CATEGORY(LogCameraSync);

void UCameraSyncSubsystem::RegisterTrigger(ASyncTrigger* Trigger)
{
	if (!Trigger) { return; }
	Triggers.RemoveAll([](const TWeakObjectPtr<ASyncTrigger>& T) { return !T.IsValid(); });
	for (const TWeakObjectPtr<ASyncTrigger>& Other : Triggers)
	{
		if (Other.Get() != Trigger && Other->TriggerName == Trigger->TriggerName)
		{
			UE_LOG(LogCameraSync, Warning, TEXT("SyncTrigger name '%s' is used by both %s and %s. Either one firing counts for wait points with this name."),
				*Trigger->TriggerName.ToString(), *Other->GetActorNameOrLabel(), *Trigger->GetActorNameOrLabel());
		}
	}
	Triggers.AddUnique(Trigger);
}

void UCameraSyncSubsystem::UnregisterTrigger(ASyncTrigger* Trigger)
{
	Triggers.Remove(Trigger);
}

void UCameraSyncSubsystem::RegisterSync(ACameraSync* Sync)
{
	Syncs.AddUnique(Sync);
}

void UCameraSyncSubsystem::UnregisterSync(ACameraSync* Sync)
{
	Syncs.Remove(Sync);
}

bool UCameraSyncSubsystem::IsTriggered(FName TriggerName) const
{
	for (const TWeakObjectPtr<ASyncTrigger>& T : Triggers)
	{
		if (T.IsValid() && T->TriggerName == TriggerName && T->IsTriggered()) { return true; }
	}
	return false;
}

bool UCameraSyncSubsystem::HasTrigger(FName TriggerName) const
{
	return FindTrigger(TriggerName) != nullptr;
}

ASyncTrigger* UCameraSyncSubsystem::FindTrigger(FName TriggerName) const
{
	ASyncTrigger* Found = nullptr;
	for (const TWeakObjectPtr<ASyncTrigger>& T : Triggers)
	{
		if (T.IsValid() && T->TriggerName == TriggerName)
		{
			if (T->IsTriggered()) { return T.Get(); }
			if (!Found) { Found = T.Get(); }
		}
	}
	return Found;
}

TArray<ASyncTrigger*> UCameraSyncSubsystem::GetTriggers() const
{
	TArray<ASyncTrigger*> Out;
	for (const TWeakObjectPtr<ASyncTrigger>& T : Triggers)
	{
		if (T.IsValid()) { Out.Add(T.Get()); }
	}
	return Out;
}

bool UCameraSyncSubsystem::GetSequenceFrame(int32& OutFrame) const
{
	for (const TWeakObjectPtr<ACameraSync>& S : Syncs)
	{
		if (S.IsValid() && S->GetCurrentFrame() >= 0)
		{
			OutFrame = S->GetCurrentFrame();
			return true;
		}
	}
	for (TActorIterator<ALevelSequenceActor> It(GetWorld()); It; ++It)
	{
		ULevelSequencePlayer* Player = It->GetSequencePlayer();
		if (Player && Player->IsPlaying())
		{
			OutFrame = Player->GetCurrentTime().Time.FloorToFrame().Value;
			return true;
		}
	}
	return false;
}
