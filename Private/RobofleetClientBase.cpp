#include "RobofleetClientBase.h"

// The base topic names for topics we want from roboleet
const std::string StatusTopicName = "status";
const std::string LocalizationTopicName = "localization";

//TODO fill in names of actual robots, Get this list from the server instead of hardcoding
std::vector<std::string> RobotNames = { "maxbot0", "gvrbot_051", "gvrbot_053", "robot_commander_001", "robot_commander_002" };



URobofleetBase::URobofleetBase()
{
	MaxQueueBeforeWaiting = 1;
	Verbosity = 2;
	HostUrl = "ws://localhost:8080";

	for (int i = 0; i < RobotNames.size(); i++) {
		RobotMap[RobotNames[i]] = std::make_shared<RobotData>();
	}
}

void URobofleetBase::deneme()
{	
	UE_LOG(LogTemp, Warning, TEXT("Module Starting"));

	hobarey = NewObject<UWebsocketClient>();

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
	const fb::amrl_msgs::RobofleetStatus* StatusRoot = flatbuffers::GetRoot<fb::amrl_msgs::RobofleetStatus>(Data);
	const fb::amrl_msgs::Localization2DMsg* LocationRoot = flatbuffers::GetRoot<fb::amrl_msgs::Localization2DMsg>(Data);
	std::string MsgTopic = StatusRoot->__metadata()->topic()->c_str();

	int NamespaceIndex = MsgTopic.substr(1, MsgTopic.length()).find('/');
	std::string RobotNamespace = MsgTopic.substr(1, NamespaceIndex);

	// Check if the robot that this message belongs to is one that we care about. If not, don't update.
	if (std::find(RobotNames.begin(), RobotNames.end(), RobotNamespace) == RobotNames.end()) {
		return;
	}

	// Check what type of message data was received, and stuff the appropriate struct
	std::size_t isStatus = MsgTopic.find(StatusTopicName);
	std::size_t isLocalization = MsgTopic.find(LocalizationTopicName);
	if (isStatus != std::string::npos) {
		UE_LOG(LogTemp, Warning, TEXT("%s"), *FString(StatusRoot->status()->c_str()));
		RobotMap[RobotNamespace]->Status.status = StatusRoot->status()->c_str();
		RobotMap[RobotNamespace]->Status.location = StatusRoot->location()->c_str();
		RobotMap[RobotNamespace]->Status.is_ok = StatusRoot->is_ok();
		RobotMap[RobotNamespace]->Status.battery_level = StatusRoot->battery_level();

	}
	else if (isLocalization != std::string::npos) {
		std::string LocationMsgRobot = LocationRoot->__metadata()->topic()->c_str();
		RobotMap[RobotNamespace]->Location.frame = LocationRoot->header()->frame_id()->c_str();
		RobotMap[RobotNamespace]->Location.x = LocationRoot->pose()->x();
		RobotMap[RobotNamespace]->Location.y = LocationRoot->pose()->y();
		RobotMap[RobotNamespace]->Location.theta = LocationRoot->pose()->theta();
	}
	for (int i = 0; i < RobotNames.size(); i++) {
		UE_LOG(LogTemp, Warning, TEXT("%s"), *FString(RobotNames[i].c_str()));
		UE_LOG(LogTemp, Warning, TEXT("%s"), *FString(RobotMap[RobotNames[i]]->Status.status.c_str()));
		UE_LOG(LogTemp, Warning, TEXT("%s"), *FString(RobotMap[RobotNames[i]]->Status.location.c_str()));
		UE_LOG(LogTemp, Warning, TEXT("%f"), RobotMap[RobotNames[i]]->Status.battery_level);
		UE_LOG(LogTemp, Warning, TEXT("%f"), RobotMap[RobotNames[i]]->Location.x);
	}

}

std::shared_ptr<RobotData> URobofleetBase::GetRobotDataByName(std::string RobotName) {
	return RobotMap[RobotName];
}

std::map<std::string, std::shared_ptr<RobotData> > URobofleetBase::GetAllRobotData() {
	return RobotMap;
}

FString URobofleetBase::GetRobotStatus(const FString& RobotName)
{
	std::string RobotNamestd = std::string(TCHAR_TO_UTF8(*RobotName));
	return FString(UTF8_TO_TCHAR(RobotMap[RobotNamestd]->Status.status.c_str()));
}

bool URobofleetBase::IsRobotOk(const FString& RobotName)
{
	std::string RobotNamestd = std::string(TCHAR_TO_UTF8(*RobotName));
	return RobotMap[RobotNamestd]->Status.is_ok;
}

float URobofleetBase::GetRobotBatteryLevel(const FString& RobotName)
{
	std::string RobotNamestd = std::string(TCHAR_TO_UTF8(*RobotName));
	return RobotMap[RobotNamestd]->Status.battery_level;
}

FString URobofleetBase::GetRobotLocationString(const FString& RobotName)
{
	std::string RobotNamestd = std::string(TCHAR_TO_UTF8(*RobotName));
	return FString(UTF8_TO_TCHAR(RobotMap[RobotNamestd]->Status.location.c_str()));
}

FVector URobofleetBase::GetRobotPosition(const FString& RobotName)
{
	std::string RobotNamestd = std::string(TCHAR_TO_UTF8(*RobotName));
	return FVector(RobotMap[RobotNamestd]->Location.x, RobotMap[RobotNamestd]->Location.y, 0);
}