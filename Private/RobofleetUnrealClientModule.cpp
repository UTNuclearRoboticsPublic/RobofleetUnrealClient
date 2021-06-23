#include "RobofleetUnrealClientModule.h"

IMPLEMENT_MODULE(FRobofleetUnrealClientModule, RobofleetUnrealClient);

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