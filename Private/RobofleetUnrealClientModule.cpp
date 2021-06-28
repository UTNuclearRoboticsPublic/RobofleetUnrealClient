#include "RobofleetUnrealClientModule.h"
#include "RobofleetClientBase.h"

IMPLEMENT_MODULE(FRobofleetUnrealClientModule, RobofleetUnrealClient);

DEFINE_LOG_CATEGORY(LogRobofleet);

void FRobofleetUnrealClientModule::StartupModule()
{
	// Put your module initialization code here
	UE_LOG(LogTemp, Warning, TEXT("Robofleet Unreal Client Module is Loaded"))
}

void FRobofleetUnrealClientModule::ShutdownModule()
{
	// Put your module termination code here
	UE_LOG(LogTemp, Warning, TEXT("Robofleet Unreal Client Module is Shutdown"))
}

FName FRobofleetUnrealClientModule::GetModuleName()
{
	static FName ModuleName = FName(TEXT("RobofleetUnrealClient"));
	return ModuleName;
}

bool FRobofleetUnrealClientModule::IsLoaded()
{
	return FModuleManager::Get().IsModuleLoaded(GetModuleName());
}

FRobofleetUnrealClientModule* FRobofleetUnrealClientModule::Get()
{
	return static_cast<FRobofleetUnrealClientModule*>(FModuleManager::Get().GetModule(FName(TEXT("RobofleetUnrealClient"))));
}

void FRobofleetUnrealClientModule::StartRobofleetSession()
{
	if (!RobofleetClient->IsValidLowLevel())
	{
		RobofleetClient = NewObject<URobofleetBase>();
	}
}
