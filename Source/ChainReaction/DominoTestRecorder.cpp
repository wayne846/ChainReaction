#include "DominoTestRecorder.h"

#include "Components/PrimitiveComponent.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "HAL/IConsoleManager.h"

DEFINE_LOG_CATEGORY_STATIC(LogDominoTest, Log, All);

static TAutoConsoleVariable<float> CVarDominoRecord(
	TEXT("domino.Record"), 0.f,
	TEXT("Test: seconds of Play to record BP_DominoChain results for (0 = off)."));

bool UDominoTestRecorder::DoesSupportWorldType(const EWorldType::Type WorldType) const
{
	return WorldType == EWorldType::PIE || WorldType == EWorldType::Game;
}

TStatId UDominoTestRecorder::GetStatId() const
{
	RETURN_QUICK_DECLARE_CYCLE_STAT(UDominoTestRecorder, STATGROUP_Tickables);
}

void UDominoTestRecorder::Tick(float DeltaTime)
{
	const float Duration = CVarDominoRecord.GetValueOnGameThread();
	if (Duration <= 0.f || bDone) { return; }
	UWorld* World = GetWorld();
	if (!World || !World->HasBegunPlay()) { return; }

	const double Now = World->GetTimeSeconds();
	if (!bStarted)
	{
		bStarted = true;
		StartTime = Now;
		NextSummary = Now;
		Report(true);
	}
	if (Now >= NextSummary)
	{
		NextSummary += 0.5;
		Report(false);
	}
	if (Now - StartTime >= Duration)
	{
		bDone = true;
		Report(true);
	}
}

void UDominoTestRecorder::Report(bool bFull)
{
	UWorld* World = GetWorld();
	const double T = World->GetTimeSeconds() - StartTime;
	for (TActorIterator<AActor> It(World); It; ++It)
	{
		AActor* Chain = *It;
		if (!Chain->GetClass()->GetName().Contains(TEXT("DominoChain"))) { continue; }

		// Read the Blueprint's ordered "Dominoes" array.
		FArrayProperty* ArrayProp = FindFProperty<FArrayProperty>(Chain->GetClass(), TEXT("Dominoes"));
		FObjectPropertyBase* Inner = ArrayProp ? CastField<FObjectPropertyBase>(ArrayProp->Inner) : nullptr;
		if (!Inner) { continue; }
		FScriptArrayHelper Helper(ArrayProp, ArrayProp->ContainerPtrToValuePtr<void>(Chain));

		int32 Fallen = 0;
		FString Lines;
		for (int32 i = 0; i < Helper.Num(); ++i)
		{
			UPrimitiveComponent* Prim = Cast<UPrimitiveComponent>(Inner->GetObjectPropertyValue(Helper.GetRawPtr(i)));
			if (!Prim) { continue; }
			const FTransform Xf = Prim->GetComponentTransform();
			const float UpZ = Xf.GetRotation().GetUpVector().Z;
			if (UpZ < 0.7f) { ++Fallen; }
			if (bFull)
			{
				const FVector L = Xf.GetLocation();
				const FRotator R = Xf.Rotator();
				Lines += FString::Printf(TEXT("\n  DOM %d loc %.1f %.1f %.1f rot %.1f %.1f %.1f up %.2f"), i, L.X, L.Y, L.Z, R.Pitch, R.Yaw, R.Roll, UpZ);
			}
		}
		UE_LOG(LogDominoTest, Log, TEXT("DominoRec t=%.2f %s: %d dominoes, %d fallen%s"), T, *Chain->GetName(), Helper.Num(), Fallen, *Lines);
	}
}
