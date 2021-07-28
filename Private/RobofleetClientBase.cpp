#include "RobofleetClientBase.h"
#include "GameFramework/Actor.h"


URobofleetBase::URobofleetBase()
{
	MaxQueueBeforeWaiting = 1;
	Verbosity = 0;
}


URobofleetBase::~URobofleetBase()
{
	UE_LOG(LogRobofleet, Error, TEXT("RobofleetBaseClient Destroyed"));
}

void URobofleetBase::Disconnect() {
	SocketClient->Disconnect();
}

bool URobofleetBase::IsInitilized()
{
	return bIsInitilized;
}

bool URobofleetBase::IsConnected()
{
	if (IsValid(SocketClient))
	{
		return SocketClient->Socket->IsConnected();
	}
	return false;
}

void URobofleetBase::Initialize(FString HostUrl, const UObject* WorldContextObject)
{	
	UE_LOG(LogTemp, Warning, TEXT("RobofleetClient module is initializing"));

	SocketClient = NewObject<UWebsocketClient>(this);
	SocketClient->Initialize(HostUrl, TEXT("ws"), false);
	SocketClient->OnReceivedCB = std::bind(&URobofleetBase::WebsocketDataCB, this, std::placeholders::_1);
	SocketClient->IsCallbackRegistered(true);

	UE_LOG(LogRobofleet, Warning, TEXT("Setting refresh timers"));
	GEngine->GetWorldFromContextObject(WorldContextObject)->GetTimerManager().SetTimer(RefreshTimerHandle, this, &URobofleetBase::RefreshRobotList, 5, true);
	
	RegisterRobotStatusSubscription();
	RegisterRobotSubscription("localization", "*");
	bIsInitilized = true;
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
	EncodeRosMsg<RobofleetSubscription>(
		msg, topic, subs, subs);
}

/*
 * Used to register a higher-overhead topic on-demand. 
 * Note that `MessageType` should be fully qualified with the message package name, i.e.
 * `amrl_msgs/Localization2D` 
 */
void URobofleetBase::RegisterRobotSubscription(FString TopicName, FString RobotName) {
	RobofleetSubscription msg;
	msg.topic_regex = "/" + std::string(TCHAR_TO_UTF8(*RobotName)) + "/" + std::string(TCHAR_TO_UTF8(*TopicName));
	msg.action = 1;
	std::string topic = "amrl_msgs/RobofleetSubscription";
	std::string subs = "/subscriptions";
	EncodeRosMsg<RobofleetSubscription>(
		msg, topic, subs, subs);
}

void URobofleetBase::RemoveObjectFromRoot()
{
	RemoveFromRoot();
}

/*
 * Called on a timer, removes robots that haven't been seen in a while.
 */
void URobofleetBase::PruneInactiveRobots() {
	std::map<FString, FDateTime> newMap;
	int CutoffTime = 10;
	for (std::map<FString, FDateTime>::iterator it = RobotsSeenTime.begin(); it != RobotsSeenTime.end(); ++it) {
		if (FDateTime::Now().GetSecond() - it->second.GetSecond() > CutoffTime) {
			OnRobotPruned.Broadcast(it->first);
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
	FString RobotNamespace = FString(MsgTopic.substr(1, NamespaceIndex).c_str());
	FString TopicIsolated = FString(MsgTopic.substr(NamespaceIndex+2, MsgTopic.length()).c_str());


	RobotsSeenTime[RobotNamespace] = FDateTime::Now();
	// If we're seeing this robot for the first time, create new data holder
	if (RobotsSeen.find(RobotNamespace) == RobotsSeen.end()) {
		RobotMap[RobotNamespace] = MakeShared<RobotData>();
		OnNewRobotSeen.Broadcast(RobotNamespace);
	}
	RobotsSeen.insert(RobotNamespace);

	DecodeMsg(Data, TopicIsolated, RobotNamespace);

	if (Verbosity)
		PrintRobotsSeen();

}

void URobofleetBase::PrintRobotsSeen() {

	UE_LOG(LogRobofleet, Warning, TEXT("Printing Existing Robots"));
	for (auto elem : RobotsSeen) {
		UE_LOG(LogRobofleet, Warning, TEXT("---------------"));
		UE_LOG(LogRobofleet, Warning, TEXT("Robot Name: %s"), *FString(elem));
		UE_LOG(LogRobofleet, Warning, TEXT("Status: %s"), *FString(RobotMap[elem]->Status.status.c_str()));
		UE_LOG(LogRobofleet, Warning, TEXT("Location String: %s"), *FString(RobotMap[elem]->Status.location.c_str()));
		UE_LOG(LogRobofleet, Warning, TEXT("Battery Level: %f"), RobotMap[elem]->Status.battery_level);
		UE_LOG(LogRobofleet, Warning, TEXT("Location, X: %f, Y: %f, Z: %f"), RobotMap[elem]->Location.x, RobotMap[elem]->Location.y, RobotMap[elem]->Location.z);
	}
}

void URobofleetBase::RefreshRobotList()
{
	if (IsConnected())
	{
		UE_LOG(LogRobofleet, Log, TEXT("Refreshing robot list"));
		RegisterRobotStatusSubscription();
		RegisterRobotSubscription("localization", "*");
		//PruneInactiveRobots();
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
void URobofleetBase::DecodeMsg(const void* Data, FString topic, FString RobotNamespace) {
	if (topic == "status") {
		RobotStatus rs = DecodeMsg<RobotStatus>(Data);
		RobotMap[RobotNamespace]->Status = rs;
	}
	else if (topic == "localization") {
		RobotLocation rl = DecodeMsg<RobotLocation>(Data);
		//UE_LOG(LogTemp,Warning,TEXT("x: %f, y:%f"), rl.x, rl.y)
		RobotMap[RobotNamespace]->Location = rl;
	}
}

FString URobofleetBase::GetRobotStatus(const FString& RobotName)
{
	// Check if robot exists
	FString RobotNamestd = FString(TCHAR_TO_UTF8(*RobotName));
	if (RobotMap.count(RobotNamestd) == 0) return "Robot unavailable";
	return FString(UTF8_TO_TCHAR(RobotMap[RobotNamestd]->Status.status.c_str()));
}

bool URobofleetBase::IsRobotOk(const FString& RobotName)
{
	FString RobotNamestd = FString(TCHAR_TO_UTF8(*RobotName));
	if (RobotMap.count(RobotNamestd) == 0) return false;
	return RobotMap[RobotNamestd]->Status.is_ok;
}

float URobofleetBase::GetRobotBatteryLevel(const FString& RobotName)
{
	FString RobotNamestd = FString(TCHAR_TO_UTF8(*RobotName));
	if (RobotMap.count(RobotNamestd) == 0) return -1.0;
	return RobotMap[RobotNamestd]->Status.battery_level;
}

FString URobofleetBase::GetRobotLocationString(const FString& RobotName)
{
	FString RobotNamestd = FString(TCHAR_TO_UTF8(*RobotName));
	if (RobotMap.count(RobotNamestd) == 0) return "Robot unavailable";
	return FString(UTF8_TO_TCHAR(RobotMap[RobotNamestd]->Status.location.c_str()));
}

FVector URobofleetBase::GetRobotPosition(const FString& RobotName)
{
	FString RobotNamestd = FString(TCHAR_TO_UTF8(*RobotName));
	if (RobotMap.count(RobotNamestd) == 0) return FVector(-1,-1,-1 );
	return FVector(RobotMap[RobotNamestd]->Location.x, RobotMap[RobotNamestd]->Location.y, 0);
}

TArray<FString> URobofleetBase::GetAllRobotsAtSite(const FString& Location)
{
	TArray<FString> RobotsAtSite;
	for (auto RobotName : RobotsSeen)
	{
		if (GetRobotLocationString(RobotName) == Location)
		{
			RobotsAtSite.Add(RobotName);
		}
	}
	return RobotsAtSite;
}
