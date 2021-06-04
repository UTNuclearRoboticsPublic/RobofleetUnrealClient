#include "RobofleetClientBase.h"


const bool verbose = true;

std::map<std::string, FDateTime> RobotsSeenTime;
std::set<std::string> RobotsSeen = {};

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
	hobarey->Initialize(TEXT("ws://192.168.1.19:8080"), TEXT("ws"));
	hobarey->OnReceivedCB = std::bind(&URobofleetBase::WebsocketDataCB, this, std::placeholders::_1);
	hobarey->IsCallbackRegistered(true);

	//std::vector<char> payload = { 'b','y','e','q','t' };
	//hobarey->Ping(payload);


	//MessageSchedulerLib<const void*> helpor(10, hamcio);
	RegisterRobotStatusSubscription();
	RegisterRobotSubscription("localization", "*", "amrl_msgs/Localization2D");
}

/*
 * Used to track all robots currently on robofleet. 
 * Status messages are very low overhead, so subscribing to all is not a problem.
 */
void URobofleetBase::RegisterRobotStatusSubscription() {
	// STATUS messages
	RobofleetSubscription msg;
	msg.topic_regex = "/*/status";
	msg.action = 1;
	std::string topic = "amrl_msgs/RobofleetSubscription";
	std::string subs = "/subscriptions";
	encode_ros_msg<RobofleetSubscription>(
		msg, topic, subs, subs);
}

/*
 * Used to register a higher-overhead topic on-demand. 
 * Note that `MessageType` should be fully qualified with the message package name, i.e.
 * `amrl_msgs/Localization2D` 
 */
void URobofleetBase::RegisterRobotSubscription(std::string TopicName, std::string RobotName, std::string MessageType) {
	// STATUS messages
	RobofleetSubscription msg;
	msg.topic_regex = "/" + RobotName + "/" + TopicName;
	msg.action = 1;
	std::string topic = MessageType;
	std::string subs = "/subscriptions";
	encode_ros_msg<RobofleetSubscription>(
		msg, topic, subs, subs);
}

/*
 * Called on a timer, removes robots that haven't been seen in a while.
 */
void URobofleetBase::PruneInactiveRobots() {
	std::map<std::string, FDateTime> newMap;
	int CutoffTime = 10;
	for (std::map<std::string, FDateTime>::iterator it = RobotsSeenTime.begin(); it != RobotsSeenTime.end(); ++it) {
		if (FDateTime::Now() - it->second > CutoffTime) {
			RobotsSeen.erase(it->first);
		}
		else {
			newMap[it->first] = it->second;
		}
	}
	RobotsSeenTime = newMap;
}


void URobofleetBase::WebsocketDataCB(const void* Data)
{
	const fb::MsgWithMetadata* msg = flatbuffers::GetRoot<fb::MsgWithMetadata>(Data);
	std::string MsgTopic = msg->__metadata()->topic()->c_str();

	int NamespaceIndex = MsgTopic.substr(1, MsgTopic.length()).find('/');
	std::string RobotNamespace = MsgTopic.substr(1, NamespaceIndex);
	std::string TopicIsolated = MsgTopic.substr(NamespaceIndex+2, MsgTopic.length());


	RobotsSeenTime[RobotNamespace] = FDateTime::Now();
	// If we're seeing this robot for the first time, create new data holder
	if (RobotsSeen.find(RobotNamespace) == RobotsSeen.end()) {
		RobotMap[RobotNamespace] = std::make_shared<RobotData>();
	}
	RobotsSeen.insert(RobotNamespace);

	DecodeMsg(Data, TopicIsolated, RobotNamespace);

	if (verbose)
		PrintRobotsSeen();

}

void URobofleetBase::PrintRobotsSeen() {
	for (auto elem : RobotsSeen) {
		UE_LOG(LogTemp, Warning, TEXT("%s"), *FString(elem.c_str()));
		UE_LOG(LogTemp, Warning, TEXT("%s"), *FString(RobotMap[elem]->Status.status.c_str()));
		UE_LOG(LogTemp, Warning, TEXT("%s"), *FString(RobotMap[elem]->Status.location.c_str()));
		UE_LOG(LogTemp, Warning, TEXT("%f"), RobotMap[elem]->Status.battery_level);
		UE_LOG(LogTemp, Warning, TEXT("%f"), RobotMap[elem]->Location.x);
	}
}

template <typename T>
typename T URobofleetBase::DecodeMsg(const void* Data) {
	const auto* root = flatbuffers::GetRoot<typename flatbuffers_type_for<T>::type>(Data);
	const T msg = decode<T>(root);
	return msg;
}

/*
 * This usage is unfortunate, but without std::any_cast and std::any, I can't see a way 
 * around switching on topic name, since we explicitly care about the returned type 
 * (and aren't just sending it over the wire like in ROS)
 */
void URobofleetBase::DecodeMsg(const void* Data, std::string topic, std::string RobotNamespace) {
	if (topic == "status") {
		RobotStatus rs = DecodeMsg<RobotStatus>(Data);
		RobotMap[RobotNamespace]->Status = rs;
	}
	else if (topic == "localization") {
		RobotLocation rl = DecodeMsg<RobotLocation>(Data);
		RobotMap[RobotNamespace]->Location = rl;
	}
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