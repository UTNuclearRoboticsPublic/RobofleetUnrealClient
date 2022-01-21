#include "RobofleetClientBase.h"
#include "GameFramework/Actor.h"
#include <typeinfo>
#include <math.h>
#include <array>

URobofleetBase::URobofleetBase()
{
	MaxQueueBeforeWaiting = 1;
	Verbosity = 0;
	bIsWorldGeoOriginSet = false;
}


URobofleetBase::~URobofleetBase()
{
	UE_LOG(LogRobofleet, Error, TEXT("RobofleetBaseClient Destroyed"));
}

void URobofleetBase::Disconnect() {
	SocketClient->Disconnect();
}

void URobofleetBase::SetWorldGeoOrigin(NavSatFix OriginPose)
{
	WorldGeoOrigin = OriginPose;
	PoseMap[FWorldOrigin] = Pose();
	ConvertToCartesian(OriginPose, FWorldOrigin);
	bIsWorldGeoOriginSet = true;
}

bool URobofleetBase::IsInitilized()
{
	return bIsInitilized;
}

bool URobofleetBase::IsConnected()
{
	if (IsValid(SocketClient))
	{
		//UE_LOG(LogRobofleet, Warning, TEXT("Websocket is Valid"))
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
	//RegisterRobotSubscription("image_compressed/main", "*");
	UE_LOG(LogRobofleet, Log, TEXT("RobofleetBase initialized"));
	RegisterRobotSubscription("detected", "*");

	bIsInitilized = true;

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
	FString TopicIsolated = FString(MsgTopic.substr(NamespaceIndex + 2, MsgTopic.length()).c_str());


	RobotsSeenTime[RobotNamespace] = FDateTime::Now();
	// If we're seeing this robot for the first time, create new data holder
	if (RobotsSeen.find(RobotNamespace) == RobotsSeen.end()) {
		RobotMap[RobotNamespace] = MakeShared<RobotData>();

		if (TopicIsolated == "NavSatFix") {
			PoseMap[RobotNamespace] = Pose();
		}

		RobotsSeen.insert(RobotNamespace);
		DecodeMsg(Data, TopicIsolated, RobotNamespace);
		OnNewRobotSeen.Broadcast(RobotNamespace);
	}
	else
	{
		RobotsSeen.insert(RobotNamespace);
		DecodeMsg(Data, TopicIsolated, RobotNamespace);
	}
}

void URobofleetBase::PrintRobotsSeen() {

	UE_LOG(LogRobofleet, Warning, TEXT("\n\nPrinting Existing Robots"));
	for (auto elem : RobotsSeen) {
		UE_LOG(LogRobofleet, Warning, TEXT("---------------"));
		UE_LOG(LogRobofleet, Warning, TEXT("Robot Name: %s"), *FString(elem));
		UE_LOG(LogRobofleet, Warning, TEXT("Status: %s"), *FString(RobotMap[elem]->Status.status.c_str()));
		UE_LOG(LogRobofleet, Warning, TEXT("Location String: %s"), *FString(RobotMap[elem]->Status.location.c_str()));
		UE_LOG(LogRobofleet, Warning, TEXT("Battery Level: %f"), RobotMap[elem]->Status.battery_level);
		UE_LOG(LogRobofleet, Warning, TEXT("Location: X: %f, Y: %f, Z: %f"), RobotMap[elem]->Location.x, RobotMap[elem]->Location.y, RobotMap[elem]->Location.z);
		UE_LOG(LogRobofleet, Warning, TEXT("Geo Location: Lat: %f, Long: %f, Alt: %f"), NavSatFixMap[elem].latitude, NavSatFixMap[elem].longitude, NavSatFixMap[elem].altitude);
		UE_LOG(LogRobofleet, Warning, TEXT("Detection Details: Name: %s, X: %f, Y: %f, Z: %f"), *FString(DetectedItemMap[elem].name.c_str()), DetectedItemMap[elem].x, DetectedItemMap[elem].y, DetectedItemMap[elem].z);
	}
}

void URobofleetBase::RefreshRobotList()
{
	if (IsConnected())
	{
		UE_LOG(LogRobofleet, Log, TEXT("Refreshing robot list"));
		RegisterRobotStatusSubscription();
		RegisterRobotSubscription("localization", "*");
		//RegisterRobotSubscription("image_compressed/main", "*");
		RegisterRobotSubscription("detected", "*");
		RegisterRobotSubscription("NavSatFix", "*");
		//PruneInactiveRobots();
	}
}


template <typename T>
typename T URobofleetBase::DecodeMsg(const void* Data) 
{ // Decoding Messages from Robofleet

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
		if (!RobotMap[RobotNamespace]->Status.location.empty())
		{
			std::string OldLocation = RobotMap[RobotNamespace]->Status.location;
			if (OldLocation != rs.location)
			{
				OnRobotLocationChanged.Broadcast(RobotNamespace, UTF8_TO_TCHAR(OldLocation.c_str()), UTF8_TO_TCHAR(rs.location.c_str()));
			}
		}
		RobotMap[RobotNamespace]->Status = rs;
	}
	else if (topic == "localization") {
		RobotLocation rl = DecodeMsg<RobotLocation>(Data);
		//UE_LOG(LogTemp,Warning,TEXT("x: %f, y:%f"), rl.x, rl.y)
		RobotMap[RobotNamespace]->Location = rl;
	}

	else if (topic == "detected") {
		DetectedItemMap[RobotNamespace] = DecodeMsg<DetectedItem>(Data);
		OnDetectedItemReceived.Broadcast(RobotNamespace);
	}
	else if (topic == "image_raw/compressed") {
		//call function to convert msg to bitmap
		//return bitmap
		RobotImageMap[RobotNamespace] = DecodeMsg<CompressedImage>(Data);
		OnImageReceived.Broadcast(RobotNamespace);
	}
	else if (topic == "NavSatFix") {
		NavSatFixMap[RobotNamespace] = DecodeMsg<NavSatFix>(Data);
		
		if (bIsWorldGeoOriginSet)
		{
			// TODO: Frank do it
			// Convert navsatfix position to x,y,z w.r.t. the worldgeoorigin.
			// set location in RobotMap eg. RobotMap[RobotNamespace]->Location.x ...
			ConvertToCartesian(NavSatFixMap[RobotNamespace], RobotNamespace);

			/*
			Since no Orientation in NavSatFix there does not need to Be a Transformation Matrix calc between
			GeoLocation of Origin and Robot because we are referencing the same orgin
			*/
			RobotMap[RobotNamespace]->Location.x = PoseMap[RobotNamespace].point.x - PoseMap[FWorldOrigin].point.x;
			RobotMap[RobotNamespace]->Location.y = PoseMap[RobotNamespace].point.y - PoseMap[FWorldOrigin].point.y;
			RobotMap[RobotNamespace]->Location.z = PoseMap[RobotNamespace].point.z - PoseMap[FWorldOrigin].point.z;
		}
	}
}

void URobofleetBase::ConvertToCartesian(const NavSatFix &GeoPose, const FString RobotNamespace)
{
	// degrees to radians
	double lat_rad = GeoPose.latitude * (PI / 180);
	double lon_rad = GeoPose.longitude * (PI / 180);

	// Estimated earth radius 
	double earth_radius = 6378137.0; // [m]

	// Simple Conversion assuming earth is a sphere
	PoseMap[RobotNamespace].point.x = earth_radius * cos(lat_rad) * cos(lon_rad);  
	PoseMap[RobotNamespace].point.y = earth_radius * cos(lat_rad) * sin(lon_rad); 
	PoseMap[RobotNamespace].point.z = earth_radius * sin(lat_rad);

	/*
	// Project Lat and Lon to flattened Sphere using a more apporiate approximation of the earths non-spherical shape
	double lat_x = cos(lat_rad);
	double lat_y = sin(lat_rad);
	double lon_x = cos(lon_rad);
	double lon_y = sin(lon_rad);
	
	double factor = 1.0 / 298.257224;
	double C = 1.0 / sqrt(lat_x * (lat_x + (1 - factor)) * (1 - factor) * lat_y * lat_y);
	double S = (1.0 - factor) * (1.0 - factor) * C;
	double h = 0.0;

	PoseMap[RobotNamespace].point.x = (earth_radius * C + h) * lat_x * lon_x;
	PoseMap[RobotNamespace].point.y = (earth_radius * C + h) * lat_x * lon_y;
	PoseMap[RobotNamespace].point.z = (earth_radius * S + h) * lat_y;
	*/
}


template <typename T>
void URobofleetBase::EncodeRosMsg (const T& msg, const std::string& msg_type, std::string& from_topic, const std::string& to_topic) 
{ // Encoding Messages for Robofleet as ROS Messages

	flatbuffers::FlatBufferBuilder fbb;
	auto metadata = encode_metadata(fbb, msg_type, to_topic);
	auto root_offset = encode<T>(fbb, msg, metadata);
	fbb.Finish(flatbuffers::Offset<void>(root_offset));

	if (SocketClient->IsValidLowLevel())
	{
		SocketClient->Send(fbb.GetBufferPointer(), fbb.GetSize(), true);
		//UE_LOG(LogRobofleet, Warning, TEXT("Message sent"));
	}
	else
	{
		UE_LOG(LogRobofleet, Warning, TEXT("Message not sent since socket is destroyed"));
	}
}

/* TODO - CHECK IF NEEDED
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

/* TODO - CHECK IF NEEDED
 * Used to register a higher-overhead topic on-demand.
 * Note that `MessageType` should be fully qualified with the message package name, i.e.
 * `amrl_msgs/Localization2D`
 */
void URobofleetBase::RegisterRobotSubscription(FString TopicName, FString RobotName)
{ // Publish a General Subscription Message to Robofleet
	RobofleetSubscription msg;
	msg.topic_regex = "/" + std::string(TCHAR_TO_UTF8(*RobotName)) + "/" + std::string(TCHAR_TO_UTF8(*TopicName));
	msg.action = 1;
	std::string topic = "amrl_msgs/RobofleetSubscription";
	std::string subs = "/subscriptions";
	EncodeRosMsg<RobofleetSubscription>(
		msg, topic, subs, subs);
}

void URobofleetBase::PublishStatusMsg(FString RobotName, RobotStatus& StatusMsg)
{ // Publish a Status Message to Robofleet
	std::string topic = "amrl_msgs/RobofleetStatus";
	std::string from = "/status";
	std::string to = "/" + std::string(TCHAR_TO_UTF8(*RobotName)) + "/status";
	EncodeRosMsg<RobotStatus>(StatusMsg, topic, from, to);
}

void URobofleetBase::PublishLocationMsg(FString RobotName, RobotLocationStamped& LocationMsg)
{ // Publish a Location Message to Robofleet
	std::string topic = "amrl_msgs/Localization2DMsg";
	std::string from = "/localization";
	std::string to = "/" + std::string(TCHAR_TO_UTF8(*RobotName)) + "/localization";
	EncodeRosMsg<RobotLocationStamped>(LocationMsg, topic, from, to);
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
	if (RobotMap.count(RobotNamestd) == 0) return FVector( 0.0f, 0.0f, 0.0f );
	return FVector(RobotMap[RobotNamestd]->Location.x, RobotMap[RobotNamestd]->Location.y, 0);
}

TArray<uint8> URobofleetBase::GetRobotImage(const FString& RobotName)
{
	FString RobotNamestd = FString(TCHAR_TO_UTF8(*RobotName));
	TArray<uint8> imageData;
	imageData.Append(&DetectedItemMap[RobotNamestd].cmpr_image.data[0], DetectedItemMap[RobotNamestd].cmpr_image.data.size());
	// you may want an TArray<FColor>
	// FColor pixelColor = {0, &RobotImageMap[Name].data[i] : i+3}
	return imageData;
}

bool URobofleetBase::IsRobotImageCompressed(const FString& RobotName)
{
	FString RobotNamestd = FString(TCHAR_TO_UTF8(*RobotName));
	if (RobotImageMap[RobotNamestd].format.find("compressed") != std::string::npos)
	{
		return true;
	}
	else return false;

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

FString URobofleetBase::GetDetectedName(const FString& RobotName)
{
	FString RobotNamestd = FString(TCHAR_TO_UTF8(*RobotName));
	if (RobotMap.count(RobotNamestd) == 0) return "Robot unavailable";
	return FString(DetectedItemMap[RobotNamestd].name.c_str());
}

FString URobofleetBase::GetDetectedRepIDRef(const FString& RobotName)
{
	FString RobotNamestd = FString(TCHAR_TO_UTF8(*RobotName));
	if (RobotMap.count(RobotNamestd) == 0) return "Robot unavailable";
	return FString(DetectedItemMap[RobotNamestd].repID.c_str());
}

FString URobofleetBase::GetDetectedAnchorIDRef(const FString& RobotName)
{
	FString RobotNamestd = FString(TCHAR_TO_UTF8(*RobotName));
	if (RobotMap.count(RobotNamestd) == 0) return "Robot unavailable";
	return FString(DetectedItemMap[RobotNamestd].anchorID.c_str());
}

FVector URobofleetBase::GetDetectedPositionRef(const FString& RobotName)
{
	FString RobotNamestd = FString(TCHAR_TO_UTF8(*RobotName));
	if (RobotMap.count(RobotNamestd) == 0) return FVector(-1,-1,-1);
	return FVector(DetectedItemMap[RobotNamestd].x, DetectedItemMap[RobotNamestd].y, DetectedItemMap[RobotNamestd].z);
}

FVector URobofleetBase::GetDetectedPositionGlobal(const FString& RobotName)
{
	FString RobotNamestd = FString(TCHAR_TO_UTF8(*RobotName));
	if (RobotMap.count(RobotNamestd) == 0) return FVector(-1, -1, -1);
	return FVector(DetectedItemMap[RobotNamestd].lat, DetectedItemMap[RobotNamestd].lon, DetectedItemMap[RobotNamestd].elv);
}
