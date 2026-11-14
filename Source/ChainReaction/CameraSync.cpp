#include "CameraSync.h"

#include "CameraSyncSubsystem.h"
#include "Components/BillboardComponent.h"
#include "Engine/Canvas.h"
#include "Engine/Engine.h"
#include "Engine/Font.h"
#include "Engine/Texture2D.h"
#include "Engine/World.h"
#include "GameFramework/HUD.h"
#include "LevelSequenceActor.h"
#include "LevelSequencePlayer.h"
#include "SyncTrigger.h"
#include "UObject/ConstructorHelpers.h"

ACameraSync::ACameraSync()
{
	PrimaryActorTick.bCanEverTick = true;
	// After physics, so a trigger entered this frame is already seen.
	PrimaryActorTick.TickGroup = TG_PostPhysics;

	Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	RootComponent = Root;

#if WITH_EDITORONLY_DATA
	Sprite = CreateEditorOnlyDefaultSubobject<UBillboardComponent>(TEXT("Sprite"));
	if (Sprite)
	{
		static ConstructorHelpers::FObjectFinderOptional<UTexture2D> Icon(TEXT("/Engine/EditorResources/S_Trigger"));
		Sprite->Sprite = Icon.Get();
		Sprite->SetupAttachment(Root);
		Sprite->bIsScreenSizeScaled = true;
	}
#endif
}

ULevelSequencePlayer* ACameraSync::GetPlayer() const
{
	return SequenceActor ? SequenceActor->GetSequencePlayer() : nullptr;
}

FName ACameraSync::GetWaitingFor() const
{
	return (Phase == EPhase::Waiting && WaitPoints.IsValidIndex(ActiveIndex)) ? WaitPoints[ActiveIndex].TriggerName : NAME_None;
}

void ACameraSync::BeginPlay()
{
	Super::BeginPlay();
	if (UCameraSyncSubsystem* Sub = GetWorld()->GetSubsystem<UCameraSyncSubsystem>())
	{
		Sub->RegisterSync(this);
	}

	Order.Reset();
	for (int32 i = 0; i < WaitPoints.Num(); ++i) { Order.Add(i); }
	Order.StableSort([this](int32 A, int32 B) { return WaitPoints[A].WaitFrame < WaitPoints[B].WaitFrame; });
	ResetPoints();

	HudHandle = AHUD::OnHUDPostRender.AddUObject(this, &ACameraSync::DrawDebug);
}

void ACameraSync::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	AHUD::OnHUDPostRender.Remove(HudHandle);
	if (UWorld* World = GetWorld())
	{
		if (UCameraSyncSubsystem* Sub = World->GetSubsystem<UCameraSyncSubsystem>())
		{
			Sub->UnregisterSync(this);
		}
	}
	Super::EndPlay(EndPlayReason);
}

void ACameraSync::ResetPoints()
{
	Reached.Init(false, WaitPoints.Num());
	Phase = EPhase::Normal;
	ActiveIndex = INDEX_NONE;
	WaitElapsed = 0.f;
}

void ACameraSync::Validate()
{
	// Done on the first tick, after every trigger has run BeginPlay and registered.
	bValidated = true;
	const FString Me = GetActorNameOrLabel();
	if (!SequenceActor)
	{
		UE_LOG(LogCameraSync, Error, TEXT("CameraSync %s: no Sequence Actor set, nothing to control."), *Me);
		return;
	}
	const UCameraSyncSubsystem* Sub = GetWorld()->GetSubsystem<UCameraSyncSubsystem>();
	for (int32 i = 0; i < WaitPoints.Num(); ++i)
	{
		const FCameraSyncWaitPoint& P = WaitPoints[i];
		if (Sub && !Sub->HasTrigger(P.TriggerName))
		{
			UE_LOG(LogCameraSync, Warning, TEXT("CameraSync %s: wait point %d waits for '%s' at frame %d, but no BP_SyncTrigger has that name. It will %s."),
				*Me, i, *P.TriggerName.ToString(), P.WaitFrame,
				P.TimeoutSeconds > 0.f ? *FString::Printf(TEXT("wait %.1f s and time out"), P.TimeoutSeconds) : TEXT("wait forever (Timeout Seconds is 0)"));
		}
	}
}

void ACameraSync::LogEvent(const FString& Text, bool bWarning)
{
	LastEvent = FString::Printf(TEXT("[%d] %s"), CurrentFrame, *Text);
	if (!bLogEvents) { return; }
	if (bWarning)
	{
		UE_LOG(LogCameraSync, Warning, TEXT("CameraSync frame %d: %s"), CurrentFrame, *Text);
	}
	else
	{
		UE_LOG(LogCameraSync, Log, TEXT("CameraSync frame %d: %s"), CurrentFrame, *Text);
	}
}

void ACameraSync::StartRamp(float Target, float Duration, float FullRange)
{
	RampFrom = Speed;
	RampTo = Target;
	RampTime = 0.f;
	// Duration is for the full 1x <-> MinPlayRate change; a shorter change takes proportionally less time.
	RampDuration = FullRange > KINDA_SMALL_NUMBER ? Duration * FMath::Clamp(FMath::Abs(Target - Speed) / FullRange, 0.f, 1.f) : 0.f;
}

void ACameraSync::UpdateSpeed(float DeltaSeconds)
{
	RampTime += DeltaSeconds;
	const float Alpha = RampDuration > KINDA_SMALL_NUMBER ? FMath::Clamp(RampTime / RampDuration, 0.f, 1.f) : 1.f;
	// Smoothstep: speed changes gently at both ends of the ramp.
	Speed = FMath::Lerp(RampFrom, RampTo, FMath::SmoothStep(0.f, 1.f, Alpha));
}

void ACameraSync::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	ULevelSequencePlayer* Player = GetPlayer();
	if (!bValidated) { Validate(); }
	if (!Player)
	{
		CurrentFrame = -1;
		return;
	}
	if (!bHasBaseRate)
	{
		bHasBaseRate = true;
		BaseRate = Player->GetPlayRate();
	}

	CurrentFrame = Player->GetCurrentTime().Time.FloorToFrame().Value;
	if (LastFrame >= 0 && CurrentFrame < LastFrame - 1)
	{
		// The sequence looped or was restarted: arm every wait point again.
		ResetPoints();
		StartRamp(1.f, 0.f, 1.f);
		LogEvent(TEXT("sequence restarted, wait points re-armed"));
	}
	LastFrame = CurrentFrame;

	const UCameraSyncSubsystem* Sub = GetWorld()->GetSubsystem<UCameraSyncSubsystem>();

	if (Phase == EPhase::Waiting)
	{
		const FCameraSyncWaitPoint& P = WaitPoints[ActiveIndex];
		WaitElapsed += DeltaSeconds;
		if (Sub && Sub->IsTriggered(P.TriggerName))
		{
			Phase = EPhase::Recovering;
			StartRamp(1.f, P.RecoverTime, 1.f - FMath::Clamp(P.MinPlayRate, 0.f, 1.f));
			LogEvent(FString::Printf(TEXT("'%s' fired after waiting %.2f s, back to normal speed"), *P.TriggerName.ToString(), WaitElapsed));
		}
		else if (P.TimeoutSeconds > 0.f && WaitElapsed >= P.TimeoutSeconds)
		{
			Phase = EPhase::Recovering;
			StartRamp(1.f, P.RecoverTime, 1.f - FMath::Clamp(P.MinPlayRate, 0.f, 1.f));
			LogEvent(FString::Printf(TEXT("'%s' did not fire within %.1f s (timeout), back to normal speed"), *P.TriggerName.ToString(), P.TimeoutSeconds), true);
		}
	}

	if (Phase != EPhase::Waiting)
	{
		for (int32 Index : Order)
		{
			if (Reached[Index] || CurrentFrame < WaitPoints[Index].WaitFrame) { continue; }
			Reached[Index] = true;
			const FCameraSyncWaitPoint& P = WaitPoints[Index];
			if (Sub && Sub->IsTriggered(P.TriggerName))
			{
				LogEvent(FString::Printf(TEXT("reached wait frame %d, '%s' already fired, passing"), P.WaitFrame, *P.TriggerName.ToString()));
				continue;
			}
			ActiveIndex = Index;
			Phase = EPhase::Waiting;
			WaitElapsed = 0.f;
			StartRamp(FMath::Clamp(P.MinPlayRate, 0.f, 1.f), P.SlowDownTime, 1.f - FMath::Clamp(P.MinPlayRate, 0.f, 1.f));
			LogEvent(FString::Printf(TEXT("reached wait frame %d, waiting for '%s'"), P.WaitFrame, *P.TriggerName.ToString()));
			break;
		}
	}

	UpdateSpeed(DeltaSeconds);
	if (Phase == EPhase::Recovering && Speed >= 1.f - KINDA_SMALL_NUMBER)
	{
		Speed = 1.f;
		Phase = EPhase::Normal;
		ActiveIndex = INDEX_NONE;
	}

	const float Rate = BaseRate * Speed;
	if (!FMath::IsNearlyEqual(Player->GetPlayRate(), Rate))
	{
		Player->SetPlayRate(Rate);
	}
}

void ACameraSync::DrawDebug(AHUD* HUD, UCanvas* DebugCanvas)
{
	if (!bShowDebug || !HUD || HUD->GetWorld() != GetWorld() || !GEngine) { return; }

	struct FLine { FString Text; FLinearColor Color; };
	TArray<FLine> Lines;
	const FLinearColor White(1.f, 1.f, 1.f), Grey(0.6f, 0.6f, 0.6f), Green(0.3f, 1.f, 0.3f), Yellow(1.f, 0.85f, 0.2f), Red(1.f, 0.35f, 0.3f);

	const ULevelSequencePlayer* Player = GetPlayer();
	Lines.Add({ FString::Printf(TEXT("CameraSync  %s"), SequenceActor ? *SequenceActor->GetActorNameOrLabel() : TEXT("(no Sequence Actor)")), Yellow });
	if (Player)
	{
		Lines.Add({ FString::Printf(TEXT("Frame %d / %d    Speed %.2fx"), CurrentFrame, Player->GetFrameDuration(), Speed), White });
	}

	const UCameraSyncSubsystem* Sub = GetWorld()->GetSubsystem<UCameraSyncSubsystem>();
	if (Phase == EPhase::Waiting)
	{
		const FCameraSyncWaitPoint& P = WaitPoints[ActiveIndex];
		const FString Limit = P.TimeoutSeconds > 0.f ? FString::Printf(TEXT(" / %.1f"), P.TimeoutSeconds) : FString();
		Lines.Add({ FString::Printf(TEXT("Waiting for: %s  (%.1f%s s)"), *P.TriggerName.ToString(), WaitElapsed, *Limit), Red });
	}
	else if (Phase == EPhase::Recovering)
	{
		Lines.Add({ TEXT("Waiting for: -  (speeding up)"), Green });
	}
	else
	{
		Lines.Add({ TEXT("Waiting for: -"), Grey });
	}
	if (!LastEvent.IsEmpty())
	{
		Lines.Add({ FString::Printf(TEXT("Last: %s"), *LastEvent), Grey });
	}

	// Every trigger name used by a wait point or placed in the level.
	TArray<FName> Names;
	for (const FCameraSyncWaitPoint& P : WaitPoints) { Names.AddUnique(P.TriggerName); }
	if (Sub)
	{
		for (const ASyncTrigger* T : Sub->GetTriggers()) { Names.AddUnique(T->TriggerName); }
	}
	Lines.Add({ TEXT("Triggers:"), White });
	for (const FName& Name : Names)
	{
		const ASyncTrigger* T = Sub ? Sub->FindTrigger(Name) : nullptr;
		if (!T)
		{
			Lines.Add({ FString::Printf(TEXT("  [?] %s  (not in level)"), *Name.ToString()), Red });
		}
		else if (T->IsTriggered())
		{
			const FString When = T->GetTriggeredFrame() >= 0 ? FString::Printf(TEXT("frame %d"), T->GetTriggeredFrame()) : FString::Printf(TEXT("%.2f s"), T->GetTriggeredTime());
			Lines.Add({ FString::Printf(TEXT("  [X] %s  fired at %s"), *Name.ToString(), *When), Green });
		}
		else
		{
			Lines.Add({ FString::Printf(TEXT("  [ ] %s"), *Name.ToString()), Grey });
		}
	}

	// Drawn through the HUD (its main canvas, like Blueprint HUD Draw Text), so it shows in game and in PIE.
	UFont* Font = GEngine->GetSmallFont();
	const float ScreenH = DebugCanvas ? DebugCanvas->ClipY : 720.f;
	const float Scale = FMath::Max(1.f, ScreenH / 720.f);
	float LineH = 0.f, MaxW = 0.f;
	for (const FLine& L : Lines)
	{
		float W = 0.f, H = 0.f;
		HUD->GetTextSize(L.Text, W, H, Font, Scale);
		MaxW = FMath::Max(MaxW, W);
		LineH = FMath::Max(LineH, H);
	}
	const float X = 20.f, Y = 20.f, Pad = 8.f;
	HUD->DrawRect(FLinearColor(0.f, 0.f, 0.f, 0.6f), X - Pad, Y - Pad, MaxW + 2 * Pad, LineH * Lines.Num() + 2 * Pad);
	for (int32 i = 0; i < Lines.Num(); ++i)
	{
		HUD->DrawText(Lines[i].Text, FLinearColor::Black, X + 1.f, Y + i * LineH + 1.f, Font, Scale);
		HUD->DrawText(Lines[i].Text, Lines[i].Color, X, Y + i * LineH, Font, Scale);
	}
}
