#include "RobofleetClientBase.h"
#include "GameFramework/Actor.h"
#include <typeinfo>
#include <math.h>
#include <array>
#include "Misc/Char.h"
#include "IImageWrapperModule.h"
#include "IImageWrapper.h"

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
	GEngine->GetWorldFromContextObjectChecked(WorldContextObject)->GetTimerManager().SetTimer(RefreshTimerHandle, this, &URobofleetBase::RefreshRobotList, 5, true);
	//GEngine->GetWorldFromContextObject(WorldContextObject)->GetTimerManager().SetTimer(RefreshTimerHandle, this, &URobofleetBase::RefreshRobotList, 5, true);
	
	RegisterRobotStatusSubscription();
	RegisterRobotSubscription("localization", "*");
	RegisterRobotSubscription("image_raw/compressed", "*");
	
	RegisterRobotSubscription("detected", "*");
	RegisterRobotSubscription("ScrewParameters", "*");

	RegisterRobotSubscription("global_path", "*");
	RegisterRobotSubscription("trail_path", "*");
	RegisterRobotSubscription("twist_path", "*");

	RegisterRobotSubscription("agent_status", "*");
	RegisterRobotSubscription("tf", "*");

	RegisterRobotSubscription("non_leg_clusters", "*");
	RegisterRobotSubscription("detected_leg_clusters", "*");
	RegisterRobotSubscription("people_detected", "*");
	RegisterRobotSubscription("people_tracked", "*");

	UE_LOG(LogRobofleet, Log, TEXT("RobofleetBase initialized"));

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

void URobofleetBase::ResetAllAgentsSeen() {
	RobotsSeen.erase(RobotsSeen.begin(),RobotsSeen.end());
}

void URobofleetBase::updateTFFrames()
{
	int CutoffTime = 10;

	for (auto& it : FrameInfoMap)
	{
		FrameInfoMap[it.first]->age = FDateTime::Now().ToUnixTimestamp() - FrameInfoMap[it.first]->TransformStamped.header.stamp._sec;
		if (FrameInfoMap[it.first]->age > CutoffTime)
		{
			//TODO  clean the tf_tree multimap
			UE_LOG(LogRobofleet, Warning, TEXT("Erase from the TF %s"), *it.first);
			//FrameInfoMap.erase(it.first);
		}
	}
}

void URobofleetBase::WebsocketDataCB(const void* Data)
{	
	const fb::MsgWithMetadata* msg = flatbuffers::GetRoot<fb::MsgWithMetadata>(Data);
	// Grab Full RobotNamespace/Topic
	std::string MsgTopic = msg->__metadata()->topic()->c_str();

	// Parce Topic into Namespace & Topic Name
	int NamespaceIndex = MsgTopic.substr(1, MsgTopic.length()).find('/');
	FString RobotNamespace = FString(MsgTopic.substr(1, NamespaceIndex).c_str());
	FString TopicIsolated = FString(MsgTopic.substr(NamespaceIndex + 2, MsgTopic.length()).c_str());

	// Print Out
	
	//UE_LOG(LogRobofleet, Warning, TEXT("RobotNamespace in: %s"), *RobotNamespace);
	//UE_LOG(LogRobofleet, Warning, TEXT("TopicIsolated in: %s"), *TopicIsolated);
	
	// *********************************************************************************

	// If the message received is a tf message, we must handle differently	
	if (TopicIsolated == "tf" || TopicIsolated == "tf_static")
	{
		DecodeTFMsg(Data); // Update RobotNamespace
	}	
	else
	{
		RobotsSeenTime[RobotNamespace] = FDateTime::Now();
		
		//RobotsSeen.insert(RobotNamespace); // Not sure if this is needed

		DecodeMsg(Data, TopicIsolated, RobotNamespace);
		
	}
}

void URobofleetBase::PrintRobotsSeen() {

	UE_LOG(LogRobofleet, Warning, TEXT("\n\nPrinting Existing Robots"));
	UE_LOG(LogRobofleet, Warning, TEXT("Size of RobotsSeen Set: %s"), *FString(std::to_string(RobotsSeen.size()).c_str()));
	for (auto elem : RobotsSeen) {
		UE_LOG(LogRobofleet, Warning, TEXT("---------------"));
		UE_LOG(LogRobofleet, Warning, TEXT("Robot Name: %s"), *FString(elem));
		UE_LOG(LogRobofleet, Warning, TEXT("Agent Status: %s"), *FString(AgentStatusMap[elem].callsign.c_str()));
		UE_LOG(LogRobofleet, Warning, TEXT("Location: X: %f, Y: %f, Z: %f"), TransformStampedMap[elem].transform.translation.x, 
																			 TransformStampedMap[elem].transform.translation.y, 
																			 TransformStampedMap[elem].transform.translation.z);

		
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
		RegisterRobotSubscription("detection", "*");
		RegisterRobotSubscription("ScrewParameters", "*");
		RegisterRobotSubscription("agent_status", "*");
		RegisterRobotSubscription("tf", "*");

		//RegisterRobotSubscription("NavSatFix", "*");		

		RegisterRobotSubscription("global_path", "*");
		RegisterRobotSubscription("trail_path", "*");
		RegisterRobotSubscription("twist_path", "*");

		//PruneInactiveRobots();
		// updateTFFrames();
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
	
	//if (topic == "status") {
	//	RobotStatus rs = DecodeMsg<RobotStatus>(Data);
	//	if (!RobotMap[RobotNamespace]->Status.location.empty())
	//	{
	//		std::string OldLocation = RobotMap[RobotNamespace]->Status.location; // Need to revisit. Should be frame_id from localization message.

	//		if (OldLocation != rs.location)
	//		{
	//			OnRobotLocationChanged.Broadcast(RobotNamespace, UTF8_TO_TCHAR(OldLocation.c_str()), UTF8_TO_TCHAR(rs.location.c_str()));
	//		}
	//	}
	//	RobotMap[RobotNamespace]->Status = rs;
	//}

	if (topic == "agent_status") {
		AgentStatus agent_status = DecodeMsg<AgentStatus>(Data);
		FString AgentNameSpace = FString(agent_status.uid.c_str());
		if (!AgentStatusMap[AgentNameSpace].agent_type.empty())
		{
			OnAgentStatusUpdate.Broadcast(AgentNameSpace);
		}
		AgentStatusMap[AgentNameSpace] = agent_status;
	}

	else if (topic == "tf") {
		UE_LOG(LogRobofleet, Warning, TEXT("In TF Decode Message, Something is wrong should not be here"));
	}

	else if (topic == "localization") {
		RobotLocation rl = DecodeMsg<RobotLocation>(Data);
		//UE_LOG(LogTemp,Warning,TEXT("x: %f, y:%f"), rl.x, rl.y)
		RobotMap[RobotNamespace]->Location = rl;
	}

	//Detected Item AugRe_msgs
	else if (topic == "detection")
	{
		// We do not care about the robot that sent detected items. Detected items are identified and saved by their UID.
		DetectedItem_augre decoded_detected_item = DecodeMsg<DetectedItem_augre>(Data);
		FString DetectedItemUid = FString(decoded_detected_item.uid.c_str());
		if (!DetectedItemAugreMap[DetectedItemUid].uid.empty())
		{
			OnDetectedItemReceived.Broadcast(DetectedItemUid);
		}
		DetectedItemAugreMap[DetectedItemUid] = decoded_detected_item;
	}

	//ScrewParam Item AugRe_msgs
	else if (topic == "ScrewParameters") {
		ScrewParametersMap[RobotNamespace] = DecodeMsg<PoseStamped>(Data);
		OnScrewParametersReceived.Broadcast(RobotNamespace);
	}

	else if (topic == "image_raw/compressed") {
		//call function to convert msg to bitmap
		//return bitmap
		UE_LOG(LogTemp, Warning, TEXT("Found a compressed image"));
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

	//Leg Tracker
	else if (topic == "non_leg_clusters") {
	LegTrackingMap[RobotNamespace]->DetectedLegClusters = DecodeMsg<DetectionArray>(Data);
	}

	else if (topic == "detected_leg_clusters") {
	LegTrackingMap[RobotNamespace]->NonLegClusters = DecodeMsg<DetectionArray>(Data);
	}

	else if (topic == "people_detected") {
	LegTrackingMap[RobotNamespace]->PeopleDetected = DecodeMsg<PersonArray>(Data);
	}

	else if (topic == "people_tracked") {
	LegTrackingMap[RobotNamespace]->PeopleTracker = DecodeMsg<PersonArray>(Data);
	}
}


/////// TF TREE ////////////

struct Node
{
	std::string name = {};
	std::string parent = {};
	std::vector<Node> children = {};
};

std::unordered_multimap<std::string, std::string> tf_tree;

Node makeTree(const std::unordered_multimap<std::string, std::string>& map, const std::string& nodeName)
{
	Node node{ nodeName };
	auto rng = map.equal_range(nodeName);
	for (auto it = rng.first; it != rng.second; ++it)
	{
		node.children.push_back(makeTree(map, it->second));
	}
	return node;
}

void printTree(const Node& n, unsigned lvl = 0)
{
	int level = lvl * 2;
	UE_LOG(LogTemp, Warning, TEXT("%*c %s\n"), lvl * 2, '>', *FString(n.name.c_str()));
	for (auto child : n.children)
	{
		printTree(child, lvl + 1);
	}
}

TArray<FString> URobofleetBase::GetChildrenFrameId(const FString& NodeName)
{
	TArray<FString> children;
	auto rng = tf_tree.equal_range(std::string(TCHAR_TO_UTF8(*NodeName)));
	for (auto it = rng.first; it != rng.second; ++it)
	{
		children.Add(FString(it->second.c_str()));
	}
	return children;
}

std::string GetTFRoot()
{
	std::vector<std::string> parent_frames = {};
	std::vector<std::string> children_frames = {};
	std::vector<std::string> root = {};

	for (auto tf_tree_it = tf_tree.begin(); tf_tree_it != tf_tree.end(); tf_tree_it++)
	{
		parent_frames.push_back(tf_tree_it->first);
		children_frames.push_back(tf_tree_it->second);
	}
	int n = sizeof(parent_frames) / sizeof(parent_frames[0]);
	std::sort(parent_frames.begin(), parent_frames.end());
	parent_frames.erase(std::unique(parent_frames.begin(), parent_frames.end()), parent_frames.end());
	std::sort(children_frames.begin(), children_frames.end());
	children_frames.erase(std::unique(children_frames.begin(), children_frames.end()), children_frames.end());

	std::set_difference(parent_frames.begin(), parent_frames.end(), children_frames.begin(), children_frames.end(), std::inserter(root, root.end()));
	UE_LOG(LogRobofleet, Warning, TEXT("Root TF Tree Size: %d"), root.size());

	for (int i = 0; i < root.size(); i++)
	{
		UE_LOG(LogRobofleet, Warning, TEXT("%s"), *FString(root[i].c_str()));
	}
	return root[0];
}

////////////////////////////

// Grab namespace from the TF Message
void URobofleetBase::DecodeTFMsg(const void* Data) {
	
	TFMessage tf_msg = DecodeMsg<TFMessage>(Data);

	for (auto tf_msg_iter = tf_msg.transforms.begin();
		tf_msg_iter != tf_msg.transforms.end();
		tf_msg_iter++)
	{
		// init
		TransformStamped rs = *tf_msg_iter;
		FString TFRobotNamespace;
		bool error = false;

		// save frames
		std::string full_frame_id = rs.header.frame_id.c_str();
		std::string child_frame_id = rs.child_frame_id.c_str();
		
		if (child_frame_id == full_frame_id)
		{
			UE_LOG(LogRobofleet, Warning, TEXT("Ignoring transform with frame_id and child_frame_id: %s because they are the same"), *FString(child_frame_id.c_str()));
			error = true;
		}

		if (child_frame_id == "/")	// Empty frame_id will be map to "/"
		{
			UE_LOG(LogRobofleet, Warning, TEXT("Ignoring transform with frame_id %s because child_frame_id not set"), *FString(full_frame_id.c_str()));
			error = true;
		}

		if (full_frame_id == "/")	//Empty parent id will be map to "/"
		{
			UE_LOG(LogRobofleet, Warning, TEXT("Ignoring transform with child_frame_id %s because frame_id not set"), *FString(child_frame_id.c_str()));
			error = true;
		}

		if (std::isnan(rs.transform.translation.x) || std::isnan(rs.transform.translation.y) || std::isnan(rs.transform.translation.z) ||
			std::isnan(rs.transform.rotation.x) || std::isnan(rs.transform.rotation.y) || std::isnan(rs.transform.rotation.z) || std::isnan(rs.transform.rotation.w))
		{
			UE_LOG(LogRobofleet, Warning, TEXT("Ignoring transform for child_frame_id %s from frame_id %s because a nan value in the transform "), *FString(child_frame_id.c_str()), *FString(full_frame_id.c_str()));
			error = true;
		}
		// TODO --- Do something when error (break maybe)
		if (error)
		{
			break;
		}
		
		//tf_tree.find(full_frame_id)
		auto it = tf_tree.begin();
		while (it != tf_tree.end()) {
			if (it->second == child_frame_id)
				break;
			it++;
		}

		if (it == tf_tree.end())
		{
			tf_tree.insert({ {full_frame_id, child_frame_id} });
		}
		//if node its already in the tree, update parent
		else
		{
			tf_tree.erase(it);
			tf_tree.insert({ {full_frame_id, child_frame_id} });
		}

		if (!FrameInfoMap[FString(child_frame_id.c_str())].IsValid())
		{
			FrameInfoMap[FString(child_frame_id.c_str())] = MakeShared<FrameInfo>();
		}
		FrameInfoMap[FString(child_frame_id.c_str())]->TransformStamped = rs;
		//update header timestamp with local time to avoid erros
		FrameInfoMap[FString(child_frame_id.c_str())]->TransformStamped.header.stamp._sec = FDateTime::Now().ToUnixTimestamp();

		// debug
		// UE_LOG(LogRobofleet, Warning, TEXT("full_frame_id: %s"), *FString(full_frame_id.c_str()));
		// UE_LOG(LogRobofleet, Warning, TEXT("Child_frame_id: %s"), *FString(child_frame_id.c_str()));
		// UE_LOG(LogRobofleet, Warning, TEXT("Transform : x %f y %f z %f "), rs.transform.translation.x, rs.transform.translation.y, rs.transform.translation.z);
		// UE_LOG(LogRobofleet, Warning, TEXT("TF Tree Size: %d"), tf_tree.size());
		// UE_LOG(LogRobofleet, Warning, TEXT("FrameInfoMap: %d"), FrameInfoMap.size());

		// If TransformStamped message is an ANCHOR transform 
		if (full_frame_id.find("anchor") != std::string::npos)
		{
			// grab asa id
			std::string asa = full_frame_id.substr(full_frame_id.find("_") + 1).c_str();
			std::replace(asa.begin(), asa.end(), '_', '-');
			rs.header.frame_id = asa;
			FString asa_id = FString(asa.c_str());

			if (AnchorGTSAM.find(asa_id) == AnchorGTSAM.end())
			{
				AnchorGTSAM.insert(asa_id);
				UE_LOG(LogTemp, Warning, TEXT("Broadcast OnNewAnchorSeen"));
				UE_LOG(LogRobofleet, Warning, TEXT("Broadcast OnNewAnchorSeen Anchor: %s"), *asa_id);
				OnNewAnchorSeen.Broadcast(asa_id);
			}
		}

		// TODO we need to refactor this logic... for now "base_link" is the string to define the tf corresponds to a robot/agent"
		if (child_frame_id.find("base_link") != std::string::npos)
		{
			// grab just "base_link" frames (without extra characters) 
			std::string base_link_str = child_frame_id.substr(child_frame_id.find("/")+1).c_str();
			UE_LOG(LogTemp, Warning, TEXT("agent_namespace  %s size: %d"), *FString(base_link_str.c_str()), base_link_str.length());

			if (base_link_str.length() == 9)
			{
				//Get the agent's name 
				TFRobotNamespace = FString(child_frame_id.substr(0, child_frame_id.find("/")).c_str());

				// New Robot Seen
				if (RobotsSeen.find(TFRobotNamespace) == RobotsSeen.end())
				{
					RobotsSeen.insert(TFRobotNamespace);
					UE_LOG(LogTemp, Warning, TEXT("Broadcast OnNewRobotSeen  %s"), *TFRobotNamespace);
					OnNewRobotSeen.Broadcast(TFRobotNamespace);
				}

				// Debug
				/*UE_LOG(LogRobofleet, Warning, TEXT("Location from rs: X: %f, Y: %f, Z: %f"), rs.transform.translation.x,
				rs.transform.translation.y,
				rs.transform.translation.z);*/
			}
		}
	}

	// UE_LOG(LogTemp, Warning, TEXT("====== TF Tree ==============="));
	//Node tree = makeTree(tf_tree, GetTFRoot());
	//printTree(tree);
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


/*
* HoloLens Publish Methods
*/


// TODO : Remove amrl_msgs

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

void URobofleetBase::PublishAgentStatusMsg(const FString& RobotName,const AgentStatus& StatusMsg)
{ // Publish a Status Message to Robofleet
	std::string topic = "augre_msgs/AgentStatus";
	std::string from = "/hololens/agent_status";
	std::string to = "/hololens/agent_status";
	EncodeRosMsg<AgentStatus>(StatusMsg, topic, from, to);
}

void URobofleetBase::PublishTransformWithCovarianceStampedMsg(const FString& TopicName, const TransformWithCovarianceStamped& TFwithCovStamped)
{
	//Topic can be odometry or measurements. Comes from the BP_Robofleet_Pub
	std::string topic = "augre_msgs/TransformWithCovarianceStamped";
	std::string from = "/gtsam/" + std::string(TCHAR_TO_UTF8(*TopicName));
	std::string to = "/gtsam/" + std::string(TCHAR_TO_UTF8(*TopicName));
	EncodeRosMsg<TransformWithCovarianceStamped>(TFwithCovStamped, topic, from, to);
}

void URobofleetBase::PublishAzureSpatialAnchorMsg(const FString& RobotName, const AzureSpatialAnchor& RosAzureSpatialAnchor) {
	
	// Publish a mo Message to Robofleet
	std::string topic = "asa_db_portal/AzureSpatialAnchor";
	std::string from = "/gtsam/add_anchor";
	//std::string to = "/" + std::string(TCHAR_TO_UTF8(*RobotName)) + "/add_anchor";
	std::string to = "/gtsam/add_anchor";
	EncodeRosMsg<AzureSpatialAnchor>(RosAzureSpatialAnchor, topic, from, to);
}

// Odom study
void URobofleetBase::PublishHololensOdom(const FString& RobotName, const PoseStamped& PoseStampedMsg)
{
	// Publish a mo Message to Robofleet
	std::string topic = "geometry_msgs/PoseStamped";
	std::string from = "/HololensOdom";
	//std::string to = "/" + std::string(TCHAR_TO_UTF8(*RobotName)) + "/HololensOdom";
	std::string to = "/odometry";
	EncodeRosMsg<PoseStamped>(PoseStampedMsg, topic, from, to);
}

void URobofleetBase::PublishMoveBaseSimpleGoal(const FString& RobotName, const PoseStamped& PoseStampedMsg)
{
	// Publish a mo Message to Robofleet
	std::string topic = "geometry_msgs/PoseStamped";
	std::string from = "/" + std::string(TCHAR_TO_UTF8(*RobotName)) + "/navigation/move_base/goal";
	std::string to = "/" + std::string(TCHAR_TO_UTF8(*RobotName)) + "/navigation/move_base/goal";
	EncodeRosMsg<PoseStamped>(PoseStampedMsg, topic, from, to);
}

void URobofleetBase::PublishHandPose(const FString& RobotName, const PoseStamped& PoseStampedMsg)
{
	// Publish a mo Message to Robofleet
	std::string topic = "geometry_msgs/PoseStamped";
	std::string from = "/HandPose";
	std::string to = "/" + std::string(TCHAR_TO_UTF8(*RobotName)) + "/HandPose";
	EncodeRosMsg<PoseStamped>(PoseStampedMsg, topic, from, to);
}

void URobofleetBase::PublishPath(const FString& RobotName, const Path& PathMsg)
{
	// Publish a path message to Robofleet
	std::string topic = "nav_msgs/Path";
	std::string from = "/" + std::string(TCHAR_TO_UTF8(*RobotName)) + "/navigation/path";
	std::string to = "/" + std::string(TCHAR_TO_UTF8(*RobotName)) + "/navigation/path";
	EncodeRosMsg<Path>(PathMsg, topic, from, to);
}

void URobofleetBase::PublishTwistMsg(const FString& RobotName, const FString& TopicName, const Twist& TwistMsg)
{
	std::string topic = "geometry_msgs/Twist";
	std::string from = "/" + std::string(TCHAR_TO_UTF8(*RobotName)) + "/" + std::string(TCHAR_TO_UTF8(*TopicName));
	std::string to = "/" + std::string(TCHAR_TO_UTF8(*RobotName)) + "/" + std::string(TCHAR_TO_UTF8(*TopicName));
	UE_LOG(LogTemp, Warning, TEXT("[PublishTwistMsg : ... %s"), *RobotName);
	EncodeRosMsg<Twist>(TwistMsg, topic, from, to);
}

void URobofleetBase::PublishTwistStampedMsg(const FString& RobotName, const FString& TopicName, const TwistStamped& TwistStampedMsg)
{
	std::string topic = "geometry_msgs/TwistStamped";
	std::string from = "/" + std::string(TCHAR_TO_UTF8(*RobotName)) + "/" + std::string(TCHAR_TO_UTF8(*TopicName));
	std::string to = "/" + std::string(TCHAR_TO_UTF8(*RobotName)) + "/" + std::string(TCHAR_TO_UTF8(*TopicName));
	UE_LOG(LogTemp, Warning, TEXT("[PublishTwistStampedMsg : ... %s"), *RobotName);
	EncodeRosMsg<TwistStamped>(TwistStampedMsg, topic, from, to);
}

void URobofleetBase::PublishTFMessage(const TFMessage& TFMessageMsg)
{
	std::string topic = "TF2_msgs/TFMessage";
	std::string from = "/augre/tf";
	std::string to = "/augre/tf";
	UE_LOG(LogTemp, Warning, TEXT("[PublishTFMessageMsg : ..."));
	EncodeRosMsg<TFMessage>(TFMessageMsg, topic, from, to);
}

/*
* Agent Status Messages
*/

FString URobofleetBase::GetUidFromAgentStatus(const FString& RobotName)
{
	// Check if robot exists
	FString RobotNamestd = FString(TCHAR_TO_UTF8(*RobotName));
	if (AgentStatusMap[RobotNamestd].uid.empty())
	{
		UE_LOG(LogTemp, Warning, TEXT("[NOT SPAWNING %s] %s is publishing a TF message, but no agent status message. Ensure tf.child_frame_id namespace matches agent_status.name field in agent status message."), *RobotNamestd, *RobotNamestd);
		return "";
	}
	return FString(AgentStatusMap[RobotNamestd].uid.c_str());
}

FString URobofleetBase::GetAgentDisplayName(const FString& RobotName)
{
	// Check if robot exists
	FString RobotNamestd = FString(TCHAR_TO_UTF8(*RobotName));
	if (AgentStatusMap.count(RobotNamestd) == 0) return "Robot unavailable";
	return FString(AgentStatusMap[RobotNamestd].callsign.c_str());
}

FString URobofleetBase::GetAgentType(const FString& RobotName)
{
	// Check if robot exists
	FString RobotNamestd = FString(TCHAR_TO_UTF8(*RobotName));
	if (AgentStatusMap.count(RobotNamestd) == 0) return "Robot unavailable";
	return FString(AgentStatusMap[RobotNamestd].agent_type.c_str());
}

float URobofleetBase::GetRobotBatteryLevel(const FString& RobotName)
{
	FString RobotNamestd = FString(TCHAR_TO_UTF8(*RobotName));
	if (AgentStatusMap.count(RobotNamestd) == 0) return -1.0;
	return AgentStatusMap[RobotNamestd].battery;
}

FString URobofleetBase::GetOwner(const FString& RobotName)
{
	// Check if robot exists
	FString RobotNamestd = FString(TCHAR_TO_UTF8(*RobotName));
	if (AgentStatusMap.count(RobotNamestd) == 0) return "Robot unavailable";
	return FString(AgentStatusMap[RobotNamestd].commander.c_str());
}

FString URobofleetBase::GetControlStatus(const FString& RobotName)
{
	// Check if robot exists
	FString RobotNamestd = FString(TCHAR_TO_UTF8(*RobotName));
	if (AgentStatusMap.count(RobotNamestd) == 0) return "Robot unavailable";
	return FString(AgentStatusMap[RobotNamestd].control_status.c_str());
}

/*
* AMRL Message Methods - NOT USED
* Left for compatability and completeness
*/
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

//FString URobofleetBase::GetRobotLocationString(const FString& RobotName)
//{
//	FString RobotNamestd = FString(TCHAR_TO_UTF8(*RobotName));
//	if (RobotMap.count(RobotNamestd) == 0) return "Robot unavailable";
//	return FString(UTF8_TO_TCHAR(RobotMap[RobotNamestd]->Status.location.c_str()));
//}

/*
* Transform Message Methods
*/

FString URobofleetBase::GetRobotLocationString(const FString& RobotName)
{
	FString RobotNamestd = FString(TCHAR_TO_UTF8(*RobotName));
	if (FrameInfoMap.count(RobotNamestd) == 0) return "Frame unavailable";
	std::string origin_anchor = FrameInfoMap[RobotNamestd]->TransformStamped.header.frame_id.c_str();
	return FString(origin_anchor.c_str());
}

// TODO - Remove
FVector URobofleetBase::GetRobotPosition(const FString& RobotName)
{
	FString RobotNamestd = FString(TCHAR_TO_UTF8(*RobotName));

	if (TransformStampedMap.count(RobotNamestd) == 0) return FVector(0.0f, 0.0f, 0.0f);
	return FVector(	TransformStampedMap[RobotNamestd].transform.translation.x, 
					TransformStampedMap[RobotNamestd].transform.translation.y,
					TransformStampedMap[RobotNamestd].transform.translation.z);
}

// Relative transform
FTransform URobofleetBase::GetFrameTransform(const FString& NodeName)
{
	FString NodeNamestd = FString(TCHAR_TO_UTF8(*NodeName));
	if (FrameInfoMap.count(NodeNamestd) == 0) return FTransform();

	return FTransform(FQuat(FrameInfoMap[NodeNamestd]->TransformStamped.transform.rotation.x,
							FrameInfoMap[NodeNamestd]->TransformStamped.transform.rotation.y,
							FrameInfoMap[NodeNamestd]->TransformStamped.transform.rotation.z,
							FrameInfoMap[NodeNamestd]->TransformStamped.transform.rotation.w),
							FVector(FrameInfoMap[NodeNamestd]->TransformStamped.transform.translation.x,
									FrameInfoMap[NodeNamestd]->TransformStamped.transform.translation.y,
									FrameInfoMap[NodeNamestd]->TransformStamped.transform.translation.z),
		FVector(1.0, 1.0, 1.0));
}

void URobofleetBase::SetFrameWorldTransform(const FString& NodeName, const FTransform& ActorWorldTransform)
{
	if (!FrameInfoMap[NodeName].IsValid())
	{
		FrameInfoMap[NodeName] = MakeShared<FrameInfo>();
		FrameInfoMap[NodeName]->TransformStamped = TransformStamped();
	}
	FrameInfoMap[NodeName]->WorldTransform = ActorWorldTransform;
}

FTransform URobofleetBase::GetFrameWorldTransform(const FString& NodeName)
{
	FString NodeNamestd = FString(TCHAR_TO_UTF8(*NodeName));
	if (FrameInfoMap.count(NodeNamestd) == 0) return FTransform();
	return FrameInfoMap[NodeName]->WorldTransform;
}

int URobofleetBase::GetAgeTransform(const FString& NodeName)
{
	FString NodeNamestd = FString(TCHAR_TO_UTF8(*NodeName));
	if (FrameInfoMap.count(NodeNamestd) == 0)
	{
		return -1;
	}
	return FrameInfoMap[NodeName]->age;
}


/*
* Compressed Image Message Methods
*/

TArray<uint8> URobofleetBase::GetRobotImage(const FString& RobotName)
{
	FString RobotNamestd = FString(TCHAR_TO_UTF8(*RobotName));
	if (RobotImageMap.count(RobotNamestd) == 0)
	{
		return TArray<uint8>();
	}
	else
	{
		return 	TArray<uint8>(&RobotImageMap[RobotNamestd].data[0], RobotImageMap[RobotNamestd].data.size());
	}
	
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

/*
* Global Message Methods
*/

TArray<FString> URobofleetBase::GetAllRobotsAtSite(const FString& Location)
{
	TArray<FString> RobotsAtSite;
	for (auto RobotName : RobotsSeen)
	{
		if (GetRobotLocationString(RobotName) == Location)
		{
			UE_LOG(LogTemp, Warning, TEXT("[GetAllRobotsAtSite Location: ... %s"), *Location);
			UE_LOG(LogTemp, Warning, TEXT("[GetAllRobotsAtSite Robot: ... %s"), *RobotName);
			RobotsAtSite.Add(RobotName);
		}
	}
	return RobotsAtSite;
}

TArray<FString> URobofleetBase::GetAllAgents()
{
	TArray<FString> ListOfAgents;
	for (auto AgentName : RobotsSeen)
	{
		ListOfAgents.Add(AgentName);
	}
	return ListOfAgents;
}

TArray<FString> URobofleetBase::GetAllFrames()
{
	TArray<FString> ListOfFrames;

	for (auto& it : FrameInfoMap)
	{
		ListOfFrames.Add(it.first);
	}
	return ListOfFrames;
}

/*
* Detected Item Message Methods
*/

FString URobofleetBase::GetDetectedName(const FString& RobotName)
{
	FString RobotNamestd = FString(TCHAR_TO_UTF8(*RobotName));
	if (AgentStatusMap.count(RobotNamestd) == 0) return "Robot unavailable";
	return FString(DetectedItemAugreMap[RobotNamestd].uid.c_str());
	//return FString(DetectedItemMap[RobotNamestd].name.c_str());
}

//TODO: REMOVE.... rep_id FIELD DOESNT EXIST ANYMORE
//FString URobofleetBase::GetDetectedRepIDRef(const FString& RobotName)
//{
//	FString RobotNamestd = FString(TCHAR_TO_UTF8(*RobotName));
//	if (AgentStatusMap.count(RobotNamestd) == 0) return "Robot unavailable";
//	return FString(DetectedItemAugreMap[RobotNamestd].asa_id.c_str());
//	//return FString(DetectedItemMap[RobotNamestd].repID.c_str()); // Currently used to pass URL
//}

//FString URobofleetBase::GetDetectedAnchorIDRef(const FString& RobotName)
//{
//	FString RobotNamestd = FString(TCHAR_TO_UTF8(*RobotName));
//	if (AgentStatusMap.count(RobotNamestd) == 0) return "Robot unavailable";
//
//	std::string asa_id = DetectedItemAugreMap[RobotNamestd].asa_id.c_str();
//	std::replace(asa_id.begin(), asa_id.end(), '_', '-');
//
//	return FString(asa_id.c_str());
//	//return FString(DetectedItemMap[RobotNamestd].anchorID.c_str());
//}

FVector URobofleetBase::GetDetectedPositionRef(const FString& RobotName)
{
	FString RobotNamestd = FString(TCHAR_TO_UTF8(*RobotName));
	if (DetectedItemAugreMap.count(RobotNamestd) == 0) return FVector(-1,-1,-1);
	return FVector(	DetectedItemAugreMap[RobotNamestd].pose.pose.position.x,
					DetectedItemAugreMap[RobotNamestd].pose.pose.position.y,
					DetectedItemAugreMap[RobotNamestd].pose.pose.position.z);
	//return FVector(DetectedItemMap[RobotNamestd].x, DetectedItemMap[RobotNamestd].y, DetectedItemMap[RobotNamestd].z);
}

//FVector URobofleetBase::GetDetectedPositionGlobal(const FString& RobotName)
//{
//	FString RobotNamestd = FString(TCHAR_TO_UTF8(*RobotName));
//	if (RobotMap.count(RobotNamestd) == 0) return FVector(-1, -1, -1);
//	return FVector(	DetectedItemAugreMap[RobotNamestd].geopose.pose.position.latitude,
//					DetectedItemAugreMap[RobotNamestd].geopose.pose.position.longitude,
//					DetectedItemAugreMap[RobotNamestd].geopose.pose.position.altitude);
//	//return FVector(DetectedItemMap[RobotNamestd].lat, DetectedItemMap[RobotNamestd].lon, DetectedItemMap[RobotNamestd].elv);
//}

TArray<uint8> URobofleetBase::GetDetectedImage(const FString& RobotName)
{
	FString RobotNamestd = FString(TCHAR_TO_UTF8(*RobotName));
	if (DetectedItemAugreMap.count(RobotNamestd) == 0) {
		return TArray<uint8>();
	}
	else {
		UE_LOG(LogRobofleet, Warning, TEXT("size %d") , DetectedItemAugreMap[RobotNamestd].cmpr_image.data.size());
		return 	TArray<uint8>(&DetectedItemAugreMap[RobotNamestd].cmpr_image.data[0], DetectedItemAugreMap[RobotNamestd].cmpr_image.data.size());
	}	
}

FVector URobofleetBase::GetDetectedImageSize(const FString& ObjectName)
{
	FString object_name = FString(TCHAR_TO_UTF8(*ObjectName));
	if (DetectedItemAugreMap.count(object_name) == 0 || !(DetectedItemAugreMap[ObjectName].cmpr_image.data.size() >= 0.0)) {
		return FVector();
	}
	else {
		TArray<uint8> image = TArray<uint8>(&DetectedItemAugreMap[ObjectName].cmpr_image.data[0], DetectedItemAugreMap[ObjectName].cmpr_image.data.size());
		IImageWrapperModule& imageWrapperModule = FModuleManager::LoadModuleChecked<IImageWrapperModule>(FName("ImageWrapper"));
		TSharedPtr<IImageWrapper> imageWrapper = imageWrapperModule.CreateImageWrapper(EImageFormat::JPEG);
		imageWrapper->SetCompressed(image.GetData(), image.Num());
		float height = imageWrapper->GetHeight();
		float width = imageWrapper->GetWidth();
		FVector size;
		size.X = height;
		size.Y = width;
		return 	size;
	}
}


//FString URobofleetBase::GetDetectedItemAsaId(const FString& DetectedItemUid)
//{
//	FString DetectedItemUidStd = FString(TCHAR_TO_UTF8(*DetectedItemUid));
//	if (DetectedItemAugreMap.count(DetectedItemUidStd) == 0) return "No Items Found!";
//
//	// Fix ASA ID if needed
//	std::string asa_id = DetectedItemAugreMap[DetectedItemUidStd].asa_id.c_str();
//	std::replace(asa_id.begin(), asa_id.end(), '_', '-');
//
//	return  FString(asa_id.c_str());
//}

FVector URobofleetBase::GetDetectedItemPosition(const FString& DetectedItemUid)
{
	FString DetectedItemUidStd = FString(TCHAR_TO_UTF8(*DetectedItemUid));
	if (DetectedItemAugreMap.count(DetectedItemUidStd) == 0) return FVector(-1, -1, -1);
	return FVector(DetectedItemAugreMap[DetectedItemUidStd].pose.pose.position.x,
		           DetectedItemAugreMap[DetectedItemUidStd].pose.pose.position.y,
		           DetectedItemAugreMap[DetectedItemUidStd].pose.pose.position.z);
	//return FVector(DetectedItemMap[RobotNamestd].x, DetectedItemMap[RobotNamestd].y, DetectedItemMap[RobotNamestd].z);
}

/*
* AR Screw Axis Message Methods
*/

// TODO NEED TO UPDATE WITH NEW MESSAGE TYPE
// /////////////////////////////////////////
// \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

FVector URobofleetBase::GetScrewAxisPoint(const FString& RobotName)
{
	FString RobotNamestd = FString(TCHAR_TO_UTF8(*RobotName));
	//if (RobotMap.count(RobotNamestd) == 0) return FVector(0, 0, 0);
	return FVector(ScrewParametersMap[RobotNamestd].pose.position.x,
		ScrewParametersMap[RobotNamestd].pose.position.y,
		ScrewParametersMap[RobotNamestd].pose.position.z);
}

FVector URobofleetBase::GetScrewAxis(const FString& RobotName)
{
	FString RobotNamestd = FString(TCHAR_TO_UTF8(*RobotName));
	//if (RobotMap.count(RobotNamestd) == 0) return FVector(0, 0, 0);
	return FVector(ScrewParametersMap[RobotNamestd].pose.orientation.x,
		ScrewParametersMap[RobotNamestd].pose.orientation.y,
		ScrewParametersMap[RobotNamestd].pose.orientation.z);
}

float URobofleetBase::GetScrewAxisPitch(const FString& RobotName)
{
	FString RobotNamestd = FString(TCHAR_TO_UTF8(*RobotName));
	//if (RobotMap.count(RobotNamestd) == 0) return float{ 0 };
	return ScrewParametersMap[RobotNamestd].pose.orientation.w;
}
// /////////////////////////////////////////
// \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

/*
* TEMOTO Message Methods
*/
void URobofleetBase::PublishStartUMRFMsg(StartUMRF& StartUMRFMsg)
{ // Publish a UMRF Message
	std::string topic = "temoto_action_engine/BroadcastStartUMRGgrapgh";
	std::string from = "/BroadcastStartUMRFgraph";
	std::string to = "/BroadcastStartUMRFgraph";
	EncodeRosMsg<StartUMRF>(StartUMRFMsg, topic, from, to);
	UE_LOG(LogTemp, Warning, TEXT("Publishing UMRF - Broadcast"));
}

void URobofleetBase::PublishStopUMRFMsg(StopUMRF& StopUMRFMsg)
{ // Publish a UMRF Message
	std::string topic = "temoto_action_engine/BroadcastStopUMRGgrapgh";
	std::string from = "/BroadcastStopUMRFgraph";
	std::string to = "/BroadcastStopUMRFgraph";
	EncodeRosMsg<StopUMRF>(StopUMRFMsg, topic, from, to);
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

	std::string frame_id = p.header.frame_id.substr(p.header.frame_id.find("_") + 1).c_str();
	std::replace(frame_id.begin(), frame_id.end(), '_', '-');
	Fp.header.frame_id = frame_id.c_str();

	//Fp.header.frame_id = (p.header.frame_id.c_str());
	Fp.header.seq = p.header.seq;
	Fp.header.stamp._nsec = p.header.stamp._nsec;
	Fp.header.stamp._sec = p.header.stamp._sec;
				
	for (std::vector<PoseStamped>::iterator it = p.poses.begin(); it != p.poses.end(); ++it)
	{
		FPoseStamped fpose_;

		std::string frame_id_pose = it->header.frame_id.substr(it->header.frame_id.find("_") + 1).c_str();
		std::replace(frame_id_pose.begin(), frame_id_pose.end(), '_', '-');
		Fp.header.frame_id = frame_id_pose.c_str();

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


/*
* Leg Tracker Detections
*/
void URobofleetBase::GetNonLegClusters(const FString& RobotName, DetectionArray& NonLegClusterArray)
{
	FString RobotNamestd = FString(TCHAR_TO_UTF8(*RobotName));
	if (LegTrackingMap.count(RobotNamestd) == 0) {
		UE_LOG(LogRobofleet, Warning, TEXT("[ERROR] In URobofleetBase::GetNonLegClusters(): No key exists in LegTrackingMap"));
	} else {
		NonLegClusterArray = LegTrackingMap[RobotName]->NonLegClusters;
	}
}

void URobofleetBase::GetDetectedLegClusters(const FString& RobotName, DetectionArray& DetectedLegClusterArray)
{
	FString RobotNamestd = FString(TCHAR_TO_UTF8(*RobotName));
	if (LegTrackingMap.count(RobotNamestd) == 0){
		UE_LOG(LogRobofleet, Warning, TEXT("[ERROR] In URobofleetBase::GetDetectedLegClusters(): No key exists in LegTrackingMap"));
	} else {
		DetectedLegClusterArray = LegTrackingMap[RobotName]->DetectedLegClusters;
	}
}

void URobofleetBase::GetPeopleDetected(const FString& RobotName, PersonArray& PeopleDetectedArray)
{
	FString RobotNamestd = FString(TCHAR_TO_UTF8(*RobotName));
	if (LegTrackingMap.count(RobotNamestd) == 0){
		UE_LOG(LogRobofleet, Warning, TEXT("[ERROR] In URobofleetBase::GetPeopleDetected(): No key exists in LegTrackingMap"));
	} else {
		PeopleDetectedArray = LegTrackingMap[RobotName]->PeopleDetected;
	}
}

void URobofleetBase::GetPeopleTracked(const FString& RobotName, PersonArray& PeopleTrackedArray)
{
	FString RobotNamestd = FString(TCHAR_TO_UTF8(*RobotName));
	if (LegTrackingMap.count(RobotNamestd) == 0){
		UE_LOG(LogRobofleet, Warning, TEXT("[ERROR] In URobofleetBase::GetPeopleTracked(): No key exists in LegTrackingMap"));
	} else {
		PeopleTrackedArray = LegTrackingMap[RobotName]->PeopleTracker;
	}
}

