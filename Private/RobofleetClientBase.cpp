#include "RobofleetClientBase.h"
#include "GameFramework/Actor.h"
#include <typeinfo>
#include <math.h>
#include <array>
#include "Misc/Char.h"

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
	RegisterRobotSubscription("image_raw/compressed", "*");
	UE_LOG(LogRobofleet, Log, TEXT("RobofleetBase initialized"));
	RegisterRobotSubscription("detected", "*");

	RegisterRobotSubscription("global_path", "*");
	RegisterRobotSubscription("trail_path", "*");
	RegisterRobotSubscription("twist_path", "*");

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
	int CutoffTime = 5;
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
	
	// *********************************************************************************
	// TODO: FIX THIS TO PARSE TOPICS WITH TWO OR MORE "/" SUCH AS /image_raw/compressed

	int NamespaceIndex = MsgTopic.substr(1, MsgTopic.length()).find('/');
	FString RobotNamespace = FString(MsgTopic.substr(1, NamespaceIndex).c_str());
	FString TopicIsolated = FString(MsgTopic.substr(NamespaceIndex + 2, MsgTopic.length()).c_str());
	//UE_LOG(LogRobofleet, Warning, TEXT("RobotNamespace: %s"), *RobotNamespace);
	//UE_LOG(LogRobofleet, Warning, TEXT("TopicIsolated: %s"), *TopicIsolated);
	
	// *********************************************************************************


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
		RegisterRobotSubscription("image_raw/compressed", "*");
		RegisterRobotSubscription("detected", "*");

		//RegisterRobotSubscription("NavSatFix", "*");
		PruneInactiveRobots();

		//PruneInactiveRobots();
		RegisterRobotSubscription("global_path", "*");
		RegisterRobotSubscription("trail_path", "*");
		RegisterRobotSubscription("twist_path", "*");

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
	//UE_LOG(LogTemp, Warning, TEXT("In Decode Message"));
	
	if (topic == "status") {
		RobotStatus rs = DecodeMsg<RobotStatus>(Data);
		if (!RobotMap[RobotNamespace]->Status.location.empty())
		{
			std::string OldLocation = RobotMap[RobotNamespace]->Status.location; // Need to revisit. Should be frame_id from localization message.
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
		//UE_LOG(LogTemp, Warning, TEXT("Found a compressed image"));
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
			RobotMap[RobotNamespace]->Location.x = PoseMap[RobotNamespace].position.x - PoseMap[FWorldOrigin].position.x;
			RobotMap[RobotNamespace]->Location.y = PoseMap[RobotNamespace].position.y - PoseMap[FWorldOrigin].position.y;
			RobotMap[RobotNamespace]->Location.z = PoseMap[RobotNamespace].position.z - PoseMap[FWorldOrigin].position.z;
		}
	}	
	
	else if (topic == "global_path") {
		RobotPath[RobotNamespace] = DecodeMsg<Path>(Data);
		FPath path = GetFPath(RobotNamespace);
		std::map<FString, FLinearColor>::iterator it;
		it = ColorGlobalPath.find(RobotNamespace);
		if (it == ColorGlobalPath.end())					//color_map does not exist
		{
			AssingBaseColor(RobotNamespace);
		}
		FString Tag = RobotNamespace;
		Tag = Tag.Append(topic);
		OnPathReceived.Broadcast(Tag, path, ColorGlobalPath[RobotNamespace]);
	}

	else if (topic == "trail_path") {
		RobotPath[RobotNamespace] = DecodeMsg<Path>(Data);
		FPath path = GetFPath(RobotNamespace);
		std::map<FString, FLinearColor>::iterator it;
		it = ColorTrailPath.find(RobotNamespace);
		if (it == ColorTrailPath.end())					//color_map does not exist
		{
			AssingBaseColor(RobotNamespace);
		}
		FString Tag = RobotNamespace;
		Tag = Tag.Append(topic);
		OnPathReceived.Broadcast(Tag, path, ColorTrailPath[RobotNamespace]);
	}

	else if (topic == "twist_path") {
		RobotPath[RobotNamespace] = DecodeMsg<Path>(Data);
		FPath path = GetFPath(RobotNamespace);
		std::map<FString, FLinearColor>::iterator it;
		it = ColorTwistPath.find(RobotNamespace);
		if (it == ColorTwistPath.end())					//color_map does not exist
		{
			AssingBaseColor(RobotNamespace);
		}
		FString Tag = RobotNamespace;
		Tag = Tag.Append(topic);
		OnPathReceived.Broadcast(Tag, path, ColorTwistPath[RobotNamespace]);
	}
}

void URobofleetBase::AssingBaseColor(const FString& RobotNamespace)
{
	int num = 0;
	FVector4 Color_HSVA;	
	for (int i = 0; i < RobotNamespace.Len(); i++)
	{
		num = num + (int)RobotNamespace.GetCharArray()[i];
	}
	Color_HSVA.X = ((num * 2) % 360);		
	Color_HSVA.Y = 1.0;
	Color_HSVA.Z = 0.2;
	Color_HSVA.W = 1.0;
	FLinearColor hsva_gp(Color_HSVA);
	ColorGlobalPath[RobotNamespace] = hsva_gp.HSVToLinearRGB();

	Color_HSVA.X = ((num * 2) % 360);		
	Color_HSVA.Y = 1.0;
	Color_HSVA.Z = 1.0;
	Color_HSVA.W = 1.0;
	FLinearColor hsva_tp(Color_HSVA);
	ColorTrailPath[RobotNamespace] = hsva_tp.HSVToLinearRGB();

	Color_HSVA.X = ((num * 2) % 360);		
	Color_HSVA.Y = 0.65;
	Color_HSVA.Z = 0.8;
	Color_HSVA.W = 0.5;
	FLinearColor hsva_twp(Color_HSVA);
	ColorTwistPath[RobotNamespace] = hsva_twp.HSVToLinearRGB();
}

void URobofleetBase::ConvertToCartesian(const NavSatFix &GeoPose, const FString RobotNamespace)
{
	// degrees to radians
	double lat_rad = GeoPose.latitude * (PI / 180);
	double lon_rad = GeoPose.longitude * (PI / 180);

	// Estimated earth radius 
	double earth_radius = 6378137.0; // [m]

	// Simple Conversion assuming earth is a sphere
	PoseMap[RobotNamespace].position.x = earth_radius * cos(lat_rad) * cos(lon_rad);  
	PoseMap[RobotNamespace].position.y = earth_radius * cos(lat_rad) * sin(lon_rad); 
	PoseMap[RobotNamespace].position.z = earth_radius * sin(lat_rad);

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

void URobofleetBase::PublishHololensOdom(const FString& RobotName, const PoseStamped& PoseStampedMsg)
{
	// Publish a mo Message to Robofleet
	std::string topic = "geometry_msgs/PoseStamped";
	std::string from = "/HololensOdom";
	std::string to = "/" + std::string(TCHAR_TO_UTF8(*RobotName)) + "/HololensOdom";
	EncodeRosMsg<PoseStamped>(PoseStampedMsg, topic, from, to);
}

void URobofleetBase::PublishMoveBaseSimpleGoal(const FString& RobotName, const PoseStamped& PoseStampedMsg)
{
	// Publish a mo Message to Robofleet
	std::string topic = "geometry_msgs/PoseStamped";
	std::string from = "/PoseStamped";
	std::string to = "/" + std::string(TCHAR_TO_UTF8(*RobotName)) + "/PoseStamped";
	EncodeRosMsg<PoseStamped>(PoseStampedMsg, topic, from, to);
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
	return 	TArray<uint8>(&RobotImageMap[RobotNamestd].data[0], RobotImageMap[RobotNamestd].data.size());
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
	return FString(DetectedItemMap[RobotNamestd].repID.c_str()); // Currently used to pass URL
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

void URobofleetBase::PublishStartUMRFMsg(StartUMRF& StartUMRFMsg)
{ // Publish a UMRF Message
	std::string topic = "temoto_action_engine/BroadcastStartUMRGgrapgh";
	std::string from = "/BroadcastStartUMRFgraph";
	std::string to = "/BroadcastStartUMRFgraph";
	EncodeRosMsg<StartUMRF>(StartUMRFMsg, topic, from, to);
	UE_LOG(LogTemp, Warning, TEXT("Publishing UMRF - Broadcast"));
}

Path URobofleetBase::GetPath(const FString& RobotName)
{
	FString RobotNamestd = FString(TCHAR_TO_UTF8(*RobotName));
	return RobotPath[RobotNamestd];
}

FPath URobofleetBase::GetFPath(const FString& RobotName)
{
	// convert from Path to FPath
	FString RobotNamestd = FString(TCHAR_TO_UTF8(*RobotName));
	TArray<FPoseStamped> poses;
	
	Path p = RobotPath[RobotNamestd];
	FPath Fp;		
	Fp.header.frame_id = (p.header.frame_id.c_str());
	Fp.header.seq = p.header.seq;
	Fp.header.stamp._nsec = p.header.stamp._nsec;
	Fp.header.stamp._sec = p.header.stamp._sec;
				
	for (std::vector<PoseStamped>::iterator it = p.poses.begin(); it != p.poses.end(); ++it)
	{
		FPoseStamped fpose_;
		fpose_.header.frame_id = (it->header.frame_id.c_str());
		fpose_.header.seq = it->header.seq;
		fpose_.header.stamp._nsec = it->header.stamp._nsec;
		fpose_.header.stamp._sec = it->header.stamp._sec;

		FVector location;
		location.X = it->pose.position.x * 100;
		location.Y = it->pose.position.y * -100;
		location.Z = it->pose.position.z * 100;
		fpose_.Transform.SetLocation(location);

		FQuat rotation;
		rotation.X = it->pose.orientation.x;
		rotation.Y = it->pose.orientation.y;
		rotation.Z = it->pose.orientation.z;
		rotation.W = it->pose.orientation.w;
		fpose_.Transform.SetRotation(rotation);

		Fp.poses.Add(fpose_);
	}
	return Fp;
}

