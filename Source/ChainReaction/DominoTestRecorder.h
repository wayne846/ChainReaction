// Test only: logs where every domino of every BP_DominoChain ends up, to compare runs.
// Set the console variable domino.Record to the number of seconds to record, then press Play.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "DominoTestRecorder.generated.h"

UCLASS()
class CHAINREACTION_API UDominoTestRecorder : public UTickableWorldSubsystem
{
	GENERATED_BODY()

public:
	virtual bool DoesSupportWorldType(const EWorldType::Type WorldType) const override;
	virtual void Tick(float DeltaTime) override;
	virtual TStatId GetStatId() const override;

private:
	void Report(bool bFull);

	bool bStarted = false;
	bool bDone = false;
	double StartTime = 0.0;
	double NextSummary = 0.0;
};
