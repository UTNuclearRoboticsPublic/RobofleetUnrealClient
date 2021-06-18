#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleInterface.h"

class FRobofleetUnrealClientModule : public IModuleInterface
{
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;
};