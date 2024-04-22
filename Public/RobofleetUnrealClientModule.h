#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleInterface.h"

class URobofleetBase;

class ROBOFLEETUNREALCLIENT_API FRobofleetUnrealClientModule : public IModuleInterface
{
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;

public:

	static FName GetModuleName();
	static bool IsLoaded();
	static FRobofleetUnrealClientModule* Get();
	static bool IsSessionRunning();

	UPROPERTY()
	URobofleetBase* RobofleetClient;

	void StartRobofleetSession(FString HostUrl, const UObject* WorldContextObject);
};