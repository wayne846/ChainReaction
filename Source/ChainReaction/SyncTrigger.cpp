#include "SyncTrigger.h"

#include "CameraSyncSubsystem.h"
#include "Components/BoxComponent.h"
#include "Components/TextRenderComponent.h"
#include "Engine/CollisionProfile.h"
#include "Engine/World.h"
#include "EngineUtils.h"

namespace SyncTriggerPrivate
{
	static FString ChannelName(ECollisionChannel Channel)
	{
		const FName Name = UCollisionProfile::Get()->ReturnChannelNameFromContainerIndex(Channel);
		return Name.IsNone() ? FString::FromInt(Channel) : Name.ToString();
	}
}

ASyncTrigger::ASyncTrigger()
{
	PrimaryActorTick.bCanEverTick = false;

	Box = CreateDefaultSubobject<UBoxComponent>(TEXT("Box"));
	Box->InitBoxExtent(FVector(50.f));
	// Query only, overlaps every channel.
	Box->SetCollisionProfileName(TEXT("Trigger"));
	Box->SetGenerateOverlapEvents(true);
	Box->SetCanEverAffectNavigation(false);
	Box->ShapeColor = FColor(255, 200, 0);
	Box->SetLineThickness(2.f);
	Box->SetHiddenInGame(true);
	RootComponent = Box;

	Label = CreateDefaultSubobject<UTextRenderComponent>(TEXT("Label"));
	Label->SetupAttachment(Box);
	Label->SetHorizontalAlignment(EHTA_Center);
	Label->SetVerticalAlignment(EVRTA_TextBottom);
	Label->SetWorldSize(24.f);
	Label->SetTextRenderColor(FColor(255, 200, 0));
	Label->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	Label->SetHiddenInGame(true);
	Label->SetCastShadow(false);
	Label->SetUsingAbsoluteScale(true);
	Label->bIsEditorOnly = true;
}

void ASyncTrigger::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);
	if (Label)
	{
		Label->SetText(FText::FromName(TriggerName));
		// Just above the top face, whatever the actor's scale.
		const float ScaleZ = FMath::Max(FMath::Abs(GetActorScale3D().Z), KINDA_SMALL_NUMBER);
		Label->SetRelativeLocation(FVector(0.f, 0.f, Box->GetUnscaledBoxExtent().Z + 5.f / ScaleZ));
	}
}

void ASyncTrigger::BeginPlay()
{
	Super::BeginPlay();
	Box->OnComponentBeginOverlap.AddDynamic(this, &ASyncTrigger::HandleBeginOverlap);
	if (UCameraSyncSubsystem* Sub = GetWorld()->GetSubsystem<UCameraSyncSubsystem>())
	{
		Sub->RegisterTrigger(this);
	}
	CheckDetectability();
}

void ASyncTrigger::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (UWorld* World = GetWorld())
	{
		if (UCameraSyncSubsystem* Sub = World->GetSubsystem<UCameraSyncSubsystem>())
		{
			Sub->UnregisterTrigger(this);
		}
	}
	Super::EndPlay(EndPlayReason);
}

void ASyncTrigger::HandleBeginOverlap(UPrimitiveComponent* OverlappedComp, AActor* OtherActor, UPrimitiveComponent* OtherComp,
	int32 OtherBodyIndex, bool bFromSweep, const FHitResult& SweepResult)
{
	if (!bTriggered && Accepts(OtherActor, OtherComp))
	{
		Fire(OtherActor);
	}
}

bool ASyncTrigger::Accepts(const AActor* OtherActor, const UPrimitiveComponent* OtherComp) const
{
	if (!OtherActor || OtherActor == this) { return false; }
	if (WatchedActor) { return OtherActor == WatchedActor; }
	return OtherComp && OtherComp->IsSimulatingPhysics();
}

void ASyncTrigger::Fire(AActor* ByActor)
{
	bTriggered = true;
	TriggeredTime = GetWorld()->GetTimeSeconds();
	TriggeredFrame = -1;

	int32 Frame = -1;
	const UCameraSyncSubsystem* Sub = GetWorld()->GetSubsystem<UCameraSyncSubsystem>();
	if (Sub && Sub->GetSequenceFrame(Frame))
	{
		TriggeredFrame = Frame;
		UE_LOG(LogCameraSync, Log, TEXT("SyncTrigger '%s' fired by %s at sequence frame %d (game time %.2f s)"),
			*TriggerName.ToString(), *GetNameSafe(ByActor), Frame, TriggeredTime);
	}
	else
	{
		UE_LOG(LogCameraSync, Log, TEXT("SyncTrigger '%s' fired by %s at game time %.2f s (no Level Sequence playing)"),
			*TriggerName.ToString(), *GetNameSafe(ByActor), TriggeredTime);
	}
	OnTriggered.Broadcast(TriggerName, ByActor);
}

void ASyncTrigger::ResetTrigger()
{
	bTriggered = false;
	TriggeredFrame = -1;
	TriggeredTime = 0.f;
}

FString ASyncTrigger::WhyCannotOverlap(const UPrimitiveComponent* Comp) const
{
	using namespace SyncTriggerPrivate;
	const ECollisionEnabled::Type Enabled = Comp->GetCollisionEnabled();
	if (Enabled == ECollisionEnabled::NoCollision)
	{
		return TEXT("Collision Enabled is No Collision");
	}
	if (!CollisionEnabledHasQuery(Enabled))
	{
		return TEXT("Collision Enabled has no Query (overlaps need Query, e.g. Query and Physics)");
	}
	if (!Comp->GetGenerateOverlapEvents())
	{
		return TEXT("Generate Overlap Events is off");
	}
	const ECollisionChannel BoxType = Box->GetCollisionObjectType();
	if (Comp->GetCollisionResponseToChannel(BoxType) == ECR_Ignore)
	{
		return FString::Printf(TEXT("its collision ignores the %s channel the trigger uses"), *ChannelName(BoxType));
	}
	const ECollisionChannel CompType = Comp->GetCollisionObjectType();
	if (Box->GetCollisionResponseToChannel(CompType) == ECR_Ignore)
	{
		return FString::Printf(TEXT("the trigger ignores its object type %s"), *ChannelName(CompType));
	}
	return FString();
}

void ASyncTrigger::CheckDetectability() const
{
	const FString Me = FString::Printf(TEXT("SyncTrigger '%s' (%s)"), *TriggerName.ToString(), *GetActorNameOrLabel());

	if (TriggerName.IsNone())
	{
		UE_LOG(LogCameraSync, Warning, TEXT("%s has no Trigger Name, so no wait point can use it."), *Me);
	}
	if (!Box->GetGenerateOverlapEvents() || !CollisionEnabledHasQuery(Box->GetCollisionEnabled()))
	{
		UE_LOG(LogCameraSync, Warning, TEXT("%s: the trigger box's own collision was changed and it cannot detect anything. Use the Trigger collision preset with Generate Overlap Events on."), *Me);
		return;
	}

	if (WatchedActor)
	{
		TInlineComponentArray<UPrimitiveComponent*> Prims(WatchedActor.Get());
		FString Reasons;
		for (const UPrimitiveComponent* Prim : Prims)
		{
			if (!Prim->IsRegistered()) { continue; }
			const FString Why = WhyCannotOverlap(Prim);
			if (Why.IsEmpty()) { return; } // At least one component can enter: fine.
			Reasons += FString::Printf(TEXT("\n    %s: %s"), *Prim->GetName(), *Why);
		}
		if (Reasons.IsEmpty())
		{
			UE_LOG(LogCameraSync, Warning, TEXT("%s: Watched Actor %s has no collision component, so it can never fire this trigger."),
				*Me, *WatchedActor->GetActorNameOrLabel());
		}
		else
		{
			UE_LOG(LogCameraSync, Warning, TEXT("%s: Watched Actor %s can never fire this trigger:%s"),
				*Me, *WatchedActor->GetActorNameOrLabel(), *Reasons);
		}
		return;
	}

	// No watched actor: check every object that simulates physics now.
	int32 Bad = 0;
	FString Examples;
	for (TActorIterator<AActor> It(GetWorld()); It; ++It)
	{
		if (*It == this) { continue; }
		TInlineComponentArray<UPrimitiveComponent*> Prims(*It);
		for (const UPrimitiveComponent* Prim : Prims)
		{
			if (!Prim->IsRegistered() || !Prim->IsSimulatingPhysics()) { continue; }
			const FString Why = WhyCannotOverlap(Prim);
			if (Why.IsEmpty()) { continue; }
			if (++Bad <= 5)
			{
				Examples += FString::Printf(TEXT("\n    %s.%s: %s"), *It->GetActorNameOrLabel(), *Prim->GetName(), *Why);
			}
		}
	}
	if (Bad > 0)
	{
		UE_LOG(LogCameraSync, Warning, TEXT("%s: %d physics object(s) in the level cannot fire this trigger%s:%s"),
			*Me, Bad, Bad > 5 ? TEXT(" (first 5 shown)") : TEXT(""), *Examples);
	}
}
