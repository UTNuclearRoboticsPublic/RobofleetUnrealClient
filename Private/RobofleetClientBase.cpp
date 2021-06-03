#include "RobofleetClientBase.h"


URobofleetBase::URobofleetBase()
{
	MaxQueueBeforeWaiting = 1;
	Verbosity = 2;
	HostUrl = "ws://localhost:8080";
}

void URobofleetBase::deneme()
{	
	UE_LOG(LogTemp, Warning, TEXT("Module Starting"));

	hobarey = NewObject<UWebsocketClient>();
	//auto hamcio = std::bind(&URobofleetBase::WebsocketDataCB, this, std::placeholders::_1);
	//std::function< void(const void*) > callback = std::bind(&URobofleetBase::WebsocketDataCB, this);

	hobarey->Initialize(TEXT("ws://192.168.1.19:8080"), TEXT("ws"));
	hobarey->OnReceivedCB = std::bind(&URobofleetBase::WebsocketDataCB, this, std::placeholders::_1);
	hobarey->IsCallbackRegistered(true);

	//std::vector<char> payload = { 'b','y','e','q','t' };
	//hobarey->Ping(payload);


	//MessageSchedulerLib<const void*> helpor(10, hamcio);
	UE_LOG(LogTemp, Warning, TEXT("Module loaded"));
	// STATUS messages
	RobofleetSubscription msg;
	msg.topic_regex = "/*/status";
	msg.action = 1;
	std::string topic = "amrl_msgs/RobofleetSubscription";
	std::string subs = "/subscriptions";
	encode_ros_msg<RobofleetSubscription>(
		msg, topic, subs, subs);

}


void URobofleetBase::WebsocketDataCB(const void* Data)
{
	UE_LOG(LogTemp, Warning, TEXT("Callback Testing"));
}