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

	hobarey->Initialize(TEXT("ws://robofleet.csres.utexas.edu:8080"));
	
	std::vector<char> payload = { 'b','y','e','q','t' };

	//hobarey->Ping(payload);

	auto hamcio = std::bind(&URobofleetBase::CallbackTest, this, std::placeholders::_1);

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


void URobofleetBase::CallbackTest(const void* Data) 
{
	UE_LOG(LogTemp, Warning, TEXT("Callback Testing"));
}