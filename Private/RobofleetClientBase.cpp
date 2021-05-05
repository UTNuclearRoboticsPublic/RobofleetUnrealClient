#include "RobofleetClientBase.h"

#include "robofleet_client_lib/MessageSchedulerLib.hpp"

#include "WebsocketClient.h"

URobofleetBase::URobofleetBase()
{
	MaxQueueBeforeWaiting = 1;
	Verbosity = 2;
	HostUrl = "ws://localhost:8080";
}

void URobofleetBase::deneme()
{	
	UWebsocketClient* hobarey = NewObject<UWebsocketClient>();

	hobarey->Initialize(TEXT("ws://172.25.22.169:8080"));
	
	std::vector<char> payload = { 'b','y','e','q','t' };

	hobarey->Ping(payload);

	auto hamcio = std::bind(&URobofleetBase::CallbackTest, this, std::placeholders::_1);

	MessageSchedulerLib<const void*> helpor(10, hamcio);
	UE_LOG(LogTemp, Warning, TEXT("Module loaded"));
}

void URobofleetBase::CallbackTest(const void* Data) 
{
	UE_LOG(LogTemp, Warning, TEXT("Callback Testing"));
}