#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleInterface.h"

class URobofleetBase;

class FRobofleetUnrealClientModule : public IModuleInterface
{
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;

public:

	static FName GetModuleName();
	static bool IsLoaded();
	static FRobofleetUnrealClientModule* Get();

	UPROPERTY()
	URobofleetBase* RobofleetClient;

	void StartRobofleetSession();
};