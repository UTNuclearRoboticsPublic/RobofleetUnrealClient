#include "RobofleetUnrealClientModule.h"
#include "RobofleetClientBase.h"
#include "WebSocketsModule.h"


IMPLEMENT_MODULE(FRobofleetUnrealClientModule, RobofleetUnrealClient);

DEFINE_LOG_CATEGORY(LogRobofleet);

void FRobofleetUnrealClientModule::StartupModule()
{
	FModuleManager::LoadModuleChecked<FWebSocketsModule>(TEXT("WebSockets"));
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
	return static_cast<FRobofleetUnrealClientModule*>(FModuleManager::Get().GetModule(GetModuleName()));
}

bool FRobofleetUnrealClientModule::IsSessionRunning()
{
	if (IsValid(Get()->RobofleetClient))
	{
		if (Get()->RobofleetClient->IsConnected())
		{
			return true;
		}
	}
	return false;
}

void FRobofleetUnrealClientModule::StartRobofleetSession(FString HostUrl, const UObject* WorldContextObject)
{
	//static bool bIsInitialized = false;
	//if (!bIsInitialized)
	{
		//bIsInitialized = true;
		RobofleetClient = NewObject<URobofleetBase>(GEngine->GetWorldFromContextObject(WorldContextObject));
		
		// TODO: FInd a better way to avoid GC...
		RobofleetClient->AddToRoot();
		RobofleetClient->Initialize(HostUrl, WorldContextObject);
	}
}
