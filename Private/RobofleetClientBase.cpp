#include "RobofleetClientBase.h"
#include "GameFramework/Actor.h"
#include <typeinfo>
#include <math.h>
#include <array>
#include "Misc/Char.h"
#include "IImageWrapperModule.h"
#include "IImageWrapper.h"
#include <regex>

/////// TF TREE ////////////

struct Node
{
	std::string name = {};
	std::string parent = {};
	std::vector<Node> children = {};
};

std::unordered_multimap<std::string, std::string> tf_tree;

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
	RegisterRobotSubscription("image/compressed", "*");
	RegisterRobotSubscription("pointcloud", "*");

	RegisterRobotSubscription("detection", "*");
	RegisterRobotSubscription("ScrewParameters", "*");

	RegisterRobotSubscription("global_path", "*");
	RegisterRobotSubscription("trail_path", "*");
	RegisterRobotSubscription("twist_path", "*");
	RegisterRobotSubscription("pv_request", "*");

	RegisterRobotSubscription("heat_map/count_rate_grid", "*");
	RegisterRobotSubscription("heat_map/count_time_grid", "*");
	RegisterRobotSubscription("heat_map/survey_grid", "*");

	RegisterRobotSubscription("agent_status", "*");
	RegisterRobotSubscription("tf", "*");

	RegisterRobotSubscription("non_leg_clusters", "*");
	RegisterRobotSubscription("detected_leg_clusters", "*");
	RegisterRobotSubscription("people_detected", "*");
	RegisterRobotSubscription("people_tracked", "*");

	RegisterRobotSubscription("goal_pose", "*");
	RegisterRobotSubscription("surface_repair/virtual_fixtures", "*");
	RegisterRobotSubscription("surface_repair/surface_points", "*");
	RegisterRobotSubscription("surface_repair/study_modality", "*");

	RegisterRobotSubscription("visualization_marker_array", "*");

	RegisterRobotSubscription("broadcast_start_umrf_graph", "*");
	RegisterRobotSubscription("umrf_status", "*");
	RegisterRobotSubscription("state", "*");

	// natural input gestures
	RegisterRobotSubscription("gesture/classification/best_result", "*");
	RegisterRobotSubscription("speech/classification/best_result", "*");
	RegisterRobotSubscription("fused/classification/result", "*");
	

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
	int CutoffTime = 30;
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
	RobotsSeen.erase(RobotsSeen.begin(), RobotsSeen.end());
	tf_tree.clear();
	FrameInfoMap.erase(FrameInfoMap.begin(), FrameInfoMap.end());
	OnResetAllAgentsSeen.Broadcast();
}

void URobofleetBase::updateTFFrames()
{
	int CutoffTime = 10;

	auto itr = FrameInfoMap.begin();
	while (itr != FrameInfoMap.end())
	{
		FrameInfoMap[itr->first]->age = FDateTime::Now().ToUnixTimestamp() - FrameInfoMap[itr->first]->TransformStamped.header.stamp._sec;
		if (FrameInfoMap[itr->first]->age > CutoffTime)
		{
			auto iter_tree = tf_tree.begin();
			while (iter_tree != tf_tree.end()) {
				if (iter_tree->second == std::string(TCHAR_TO_UTF8(*itr->first)))
					break;
				iter_tree++;
			}
			if (iter_tree != tf_tree.end())
			{
				tf_tree.erase(iter_tree);
			}

			// Get name of agent --- Asuming: name/base_link 
			if (isAnchorFrame(itr->first))
			{
				//UE_LOG(LogRobofleet, Warning, TEXT("It is an anchor Frame: %s"), *itr->first);
				++itr;
			}
			else
			{
				std::string agent_name = std::string(TCHAR_TO_UTF8(*itr->first));
				FString agent = FString(agent_name.substr(0, agent_name.find("/")).c_str());

				//Remove the representation
				OnRobotPruned.Broadcast(agent);
				RobotsSeen.erase(agent);

				FrameInfoMap.erase(itr++);
			}
		}
		else {
			++itr;
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
	if (TopicIsolated == "tf")
	{
		DecodeTFMsg(Data); // Update RobotNamespace
	}
	else if (TopicIsolated == "tf_static")
	{
		DecodeTFMsg(Data, true); // Update RobotNamespace
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
		UE_LOG(LogRobofleet, Warning, TEXT("Agent UID: %s"), *FString(elem));
		UE_LOG(LogRobofleet, Warning, TEXT("Agent Status Callsign: %s"), *FString(AgentStatusMap[elem].callsign.c_str()));
		//UE_LOG(LogRobofleet, Warning, TEXT("Location: X: %f, Y: %f, Z: %f"), TransformStampedMap[elem].transform.translation.x, 
		//																	 TransformStampedMap[elem].transform.translation.y, 
		//																	 TransformStampedMap[elem].transform.translation.z);


	}
}

void URobofleetBase::RefreshRobotList()
{
	if (IsConnected())
	{
		//UE_LOG(LogRobofleet, Log, TEXT("Refreshing robot list"));
		RegisterRobotStatusSubscription();
		RegisterRobotSubscription("localization", "*");
		RegisterRobotSubscription("agent_status", "*");
		RegisterRobotSubscription("tf", "*");
		RegisterRobotSubscription("detection", "*");
		RegisterRobotSubscription("image/compressed", "*");
		RegisterRobotSubscription("pointcloud", "*");

		RegisterRobotSubscription("heat_map/count_rate_grid", "*");
		RegisterRobotSubscription("heat_map/count_time_grid", "*");
		RegisterRobotSubscription("heat_map/survey_grid", "*");

		RegisterRobotSubscription("ScrewParameters", "*");

		//RegisterRobotSubscription("NavSatFix", "*");		

		RegisterRobotSubscription("global_path", "*");
		RegisterRobotSubscription("trail_path", "*");
		RegisterRobotSubscription("twist_path", "*");
		RegisterRobotSubscription("pv_request", "*");

		RegisterRobotSubscription("goal_pose", "*");
		RegisterRobotSubscription("surface_repair/virtual_fixtures", "*");
		RegisterRobotSubscription("surface_repair/surface_points", "*");
		RegisterRobotSubscription("surface_repair/study_modality", "*");

		RegisterRobotSubscription("visualization_marker_array", "*");

		RegisterRobotSubscription("broadcast_start_umrf_graph", "*");
		RegisterRobotSubscription("umrf_status", "*");
		RegisterRobotSubscription("state", "*");

		// get gesture and speech classification results
		RegisterRobotSubscription("gesture/classification/best_result", "*");
		RegisterRobotSubscription("speech/classification/best_result", "*");
		RegisterRobotSubscription("fused/classification/result", "*");

		//PruneInactiveRobots();
		updateTFFrames();
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
	//UE_LOG(LogTemp, Warning, TEXT("topic: %s"), *topic);

	if (topic == "agent_status") {
		AgentStatus agent_status = DecodeMsg<AgentStatus>(Data);
		FString AgentNameSpace = FString(agent_status.uid.c_str());
		//if (!AgentStatusMap[AgentNameSpace].agent_type.empty())
		//if (AgentStatusMap.find(AgentNameSpace) == AgentStatusMap.end())
		//{
		//	OnAgentStatusUpdate.Broadcast(AgentNameSpace);
		//}
		AgentStatusMap[AgentNameSpace] = agent_status;
	}

	else if (topic == "tf") {
		UE_LOG(LogRobofleet, Warning, TEXT("In TF Decode Message, Something is wrong should not be here"));
	}

	// TODO: to delete
	else if (topic == "localization") {
		RobotLocation rl = DecodeMsg<RobotLocation>(Data);
		//UE_LOG(LogTemp,Warning,TEXT("x: %f, y:%f"), rl.x, rl.y)
		RobotMap[RobotNamespace]->Location = rl;
	}

	else if (topic.StartsWith("image/compressed/")) {
		FString StreamName = topic;
		StreamName.RemoveFromStart("image/compressed/");

		if (!RobotImageMap[RobotNamespace][StreamName].IsValid()) {
			RobotImageMap[RobotNamespace][StreamName] = MakeShared<CompressedImage>();
		}
		*RobotImageMap[RobotNamespace][StreamName] = DecodeMsg<CompressedImage>(Data);
		OnImageReceived.Broadcast(RobotNamespace, StreamName);
	}

	//Detected Item AugRe_msgs
	else if (topic == "detection")
	{
		// We do not care about the robot that sent detected items. Detected items are identified and saved by their UID.
		DetectedItem_augre decoded_detected_item = DecodeMsg<DetectedItem_augre>(Data);
		FString DetectedItemUid = FString(decoded_detected_item.uid.c_str());
		FString frame_id = decoded_detected_item.pose.header.frame_id.c_str();

		if (decoded_detected_item.pose.header.stamp._sec == 0)
		{
			decoded_detected_item.pose.header.stamp._nsec = FDateTime::Now().GetMillisecond() * 1000000;
			decoded_detected_item.pose.header.stamp._sec = FDateTime::Now().ToUnixTimestamp();
		}
	    UE_LOG(LogTemp, Warning, TEXT("===Detected Item %s ==="), *DetectedItemUid);

		DetectedItemAugreMap[DetectedItemUid] = decoded_detected_item;
		OnDetectedItemReceived.Broadcast(DetectedItemUid, frame_id);

		/*if (!DetectedItemAugreMap[DetectedItemUid].uid.empty())
		{
			OnDetectedItemReceived.Broadcast(DetectedItemUid, frame_id);
		}
		DetectedItemAugreMap[DetectedItemUid] = decoded_detected_item;*/
	}

	//ScrewParam Item AugRe_msgs
	else if (topic == "ScrewParameters") {
		ScrewParametersMap[RobotNamespace] = DecodeMsg<PoseStamped>(Data);
		OnScrewParametersReceived.Broadcast(RobotNamespace);
	}

	else if (topic == "pointcloud") {
		if (!PointCloudMap[RobotNamespace].IsValid()) {
			PointCloudMap[RobotNamespace] = MakeShared<PointCloud2>();
		}
		*PointCloudMap[RobotNamespace] = DecodeMsg<PointCloud2>(Data);
		OnPointCloudMessageReceived.Broadcast(RobotNamespace);
	}

	else if (topic == "surface_repair/virtual_fixtures") {
		if (!PointCloudMap[RobotNamespace].IsValid()) {
			PointCloudMap[RobotNamespace] = MakeShared<PointCloud2>();
		}
		*PointCloudMap[RobotNamespace] = DecodeMsg<PointCloud2>(Data);
		OnPointCloudMessageReceived.Broadcast(RobotNamespace);
	}

	else if (topic == "surface_repair/surface_points") {
		if (!PointCloudMap[RobotNamespace].IsValid()) {
			PointCloudMap[RobotNamespace] = MakeShared<PointCloud2>();
		}
		*PointCloudMap[RobotNamespace] = DecodeMsg<PointCloud2>(Data);
		OnPointCloudMessageReceived.Broadcast(RobotNamespace);
	}

	else if (topic == "surface_repair/study_modality") {

		StudyModalityMap[RobotNamespace] = DecodeMsg<Header>(Data);
		OnStudyModalityReceived.Broadcast(RobotNamespace);
	}


	
	// TODO: Do we need this? If not... Delete
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
		//RobotMap is depricated
	}

	else if (topic == "global_path") {
		GlobalPath[RobotNamespace] = DecodeMsg<Path>(Data);
		FPath path = GetFPath(RobotNamespace, "global_path");
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
		TrailPath[RobotNamespace] = DecodeMsg<Path>(Data);
		FPath path = GetFPath(RobotNamespace, "trail_path");
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
		TwistPath[RobotNamespace] = DecodeMsg<Path>(Data);
		FPath path = GetFPath(RobotNamespace, "twist_path");
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

	// Heat Map
	else if (topic == "heat_map/count_rate_grid") {
		//UE_LOG(LogTemp, Display, TEXT("Found a heat_map"));
		OccupancyGridMap[RobotNamespace] = DecodeMsg<OccupancyGrid>(Data);
		OnOccupancyGridReceived.Broadcast(RobotNamespace);

	}

	//Leg Tracker
	else if (topic == "non_leg_clusters") {
		if (!LegTrackingMap[RobotNamespace].IsValid()) {
			LegTrackingMap[RobotNamespace] = MakeShared<LegTrackingData>();
		}
		LegTrackingMap[RobotNamespace]->NonLegClusters = DecodeMsg<DetectionArray>(Data);
		// TODO: Create Delegate
	}

	else if (topic == "detected_leg_clusters") {
		if (!LegTrackingMap[RobotNamespace].IsValid()) {
			LegTrackingMap[RobotNamespace] = MakeShared<LegTrackingData>();
		}
		LegTrackingMap[RobotNamespace]->DetectedLegClusters = DecodeMsg<DetectionArray>(Data);
		OnDetectedLegClusterReceived.Broadcast(RobotNamespace);
	}

	else if (topic == "people_detected") {
		if (!LegTrackingMap[RobotNamespace].IsValid()) {
			LegTrackingMap[RobotNamespace] = MakeShared<LegTrackingData>();
		}
		LegTrackingMap[RobotNamespace]->PeopleDetected = DecodeMsg<PersonArray>(Data);
		// TODO: Create Delegate
	}

	else if (topic == "people_tracked") {
		if (!LegTrackingMap[RobotNamespace].IsValid()) {
			LegTrackingMap[RobotNamespace] = MakeShared<LegTrackingData>();
		}
		LegTrackingMap[RobotNamespace]->PeopleTracker = DecodeMsg<PersonArray>(Data);
		// TODO: Create Delegate
	}

	else if (topic == "goal_pose") {
		GoalPoseMap[RobotNamespace] = DecodeMsg<PoseStamped>(Data);
		OnGoalPoseReceived.Broadcast(RobotNamespace);
	}

	else if (topic == "pv_request") {
		OnPVCamRequest.Broadcast(RobotNamespace, DecodeMsg<Bool>(Data).data);
	}

	else if (topic == "visualization_marker_array") {
		if (!MarkerArrayMap[RobotNamespace].IsValid()) {
			MarkerArrayMap[RobotNamespace] = MakeShared<MarkerArray>();
		}

		*MarkerArrayMap[RobotNamespace] = DecodeMsg<MarkerArray>(Data);
		OnMarkerArrayMessageReceived.Broadcast(RobotNamespace);
	}

	// TeMoto Topics
	else if (topic == "broadcast_start_umrf_graph") {
		StartUMRF decoded_umrf_g = DecodeMsg<StartUMRF>(Data);
		FString umrf_graph_json = FString(decoded_umrf_g.umrf_graph_json.c_str());
		UMRF_Graph[umrf_graph_json] = decoded_umrf_g;
		OnUMRFGraphReceived.Broadcast(umrf_graph_json);
	}

	else if (topic == "umrf_status") {
		String umrf_status = DecodeMsg<String>(Data);
		std::string action_status = umrf_status.data;
		int action = action_status.substr(1, action_status.length()).find(';');
		FString action_name = FString(action_status.substr(0, action + 1).c_str());
		FString status = FString(action_status.substr(action + 2, action_status.length()).c_str());
		OnUMRFGraphStatus.Broadcast(action_name, status);
	}

	else if (topic == "state") {
		UE_LOG(LogTemp, Warning, TEXT("=== GPT Parser State ==="));
		String state = DecodeMsg<String>(Data);
		OnGPTParserState.Broadcast(FString(state.data.c_str()));
	}

	else if (topic == "gesture/classification/best_result") {
		UE_LOG(LogTemp, Warning, TEXT("=== Gesture Results Available ==="));
		String result = DecodeMsg<String>(Data);
		GestureCmdMap[RobotNamespace] = result;
		UE_LOG(LogTemp, Warning, TEXT("Gesture: %s"), *FString(result.data.c_str()));
		OnGestureCmd.Broadcast(RobotNamespace);
	}

	else if (topic == "speech/classification/best_result") {
		UE_LOG(LogTemp, Warning, TEXT("=== Speech Results Available ==="));
		String result = DecodeMsg<String>(Data);
		SpeechCmdMap[RobotNamespace] = result;
		UE_LOG(LogTemp, Warning, TEXT("Speech: %s"), *FString(result.data.c_str()));
		OnSpeechCmd.Broadcast(RobotNamespace);
	}
	
	else if (topic == "fused/classification/result") {
		UE_LOG(LogTemp, Warning, TEXT("=== Combined Results Available ==="));
		String result = DecodeMsg<String>(Data);
		FusedCmdMap[RobotNamespace] = result;
		UE_LOG(LogTemp, Warning, TEXT("Fused: %s"), *FString(result.data.c_str()));
		OnFusedCmd.Broadcast(RobotNamespace);
		}
}


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

FTransform URobofleetBase::LookupTransform(const FString& target_frame, const FString& source_frame)
{
	FTransform temp1 = GetFrameWorldTransform(source_frame);
	FTransform temp2 = GetFrameWorldTransform(target_frame);
	temp1.Inverse();
	FTransform lookuptransform = FTransform();
	lookuptransform.SetTranslation(temp1.Rotator().GetInverse().RotateVector(temp2.GetTranslation() - temp1.GetTranslation()));
	lookuptransform.SetRotation(FQuat(temp1.Rotator().GetInverse()) * FQuat(temp2.Rotator()));
	return lookuptransform;
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
void URobofleetBase::DecodeTFMsg(const void* Data, bool is_static) {

	TFMessage tf_msg = DecodeMsg<TFMessage>(Data);

	for (auto tf_msg_iter = tf_msg.transforms.begin();
		tf_msg_iter != tf_msg.transforms.end();
		tf_msg_iter++)
	{
		// init
		TransformStamped rs = *tf_msg_iter;
		FString TFRobotNamespace;
		bool error = false;

		std::replace(rs.header.frame_id.begin(), rs.header.frame_id.end(), '-', '_');
		std::replace(rs.child_frame_id.begin(), rs.child_frame_id.end(), '-', '_');

		// save frames
		std::string full_frame_id = rs.header.frame_id.c_str();
		std::string child_frame_id = rs.child_frame_id.c_str();

		//UE_LOG(LogRobofleet, Warning, TEXT("full_frame_id: %s"), *FString(full_frame_id.c_str()));
		//UE_LOG(LogRobofleet, Warning, TEXT("Child_frame_id: %s"), *FString(child_frame_id.c_str()));


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
		//if node is already in the tree, update parent
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

		// If tf_static
		if (is_static)
		{
			FrameInfoMap[FString(child_frame_id.c_str())]->TransformStamped.header.stamp._sec = -1;
			//UE_LOG(LogRobofleet, Warning, TEXT("FrameInfoMap: %s %d"), *FString(child_frame_id.c_str()), FrameInfoMap[FString(child_frame_id.c_str())]->TransformStamped.header.stamp._sec);
		}

		// debug
		//UE_LOG(LogRobofleet, Warning, TEXT("full_frame_id: %s"), *FString(full_frame_id.c_str()));
		//UE_LOG(LogRobofleet, Warning, TEXT("Child_frame_id: %s"), *FString(child_frame_id.c_str()));
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
				if (isValidUuid(asa_id))
				{
					OnNewAnchorSeen.Broadcast(asa_id);
				}
				else
				{
					UE_LOG(LogTemp, Warning, TEXT("ASA id invalid"));
				}
			}
		}

		// TODO we need to refactor this logic... for now "base_link" is the string to define the tf corresponds to a robot/agent"
		if (child_frame_id.find("base_link") != std::string::npos)
		{
			// grab just "base_link" frames (without extra characters) 
			std::string base_link_str = child_frame_id.substr(child_frame_id.find("/") + 1).c_str();

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

	/*UE_LOG(LogTemp, Warning, TEXT("====== TF Tree ==============="));
	Node tree = makeTree(tf_tree, GetTFRoot());
	printTree(tree);*/
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

void URobofleetBase::ConvertToCartesian(const NavSatFix& GeoPose, const FString RobotNamespace)
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
void URobofleetBase::EncodeRosMsg(const T& msg, const std::string& msg_type, std::string& from_topic, const std::string& to_topic)
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




void URobofleetBase::PublishHeaderMsg(const std::string& TopicName, const std::string& Namespace, const Header& Msg)
{
	std::string topic = "std_msgs/Header";
	std::string from = "/" + TopicName;
	std::string to = "/" + TopicName;

	if (!Namespace.empty()) {
		from = "/" + Namespace + from;
		to = "/" + Namespace + to;
	}

	/*UE_LOG(LogTemp, Warning, TEXT("Publish Header Message to: %s"), *FString(UTF8_TO_TCHAR(to.c_str())));*/

	EncodeRosMsg<Header>(Msg, topic, from, to);
}

void URobofleetBase::PublishHeaderArrayMsg(const std::string& TopicName, const std::string& Namespace, const HeaderArrayStamped& Msg)
{
	std::string topic = "augre_msgs/HeaderArrayStamped";
	std::string from = "/" + TopicName;
	std::string to = "/" + TopicName;

	if (!Namespace.empty()) {
		from = "/" + Namespace + from;
		to = "/" + Namespace + to;
	}

	EncodeRosMsg<HeaderArrayStamped>(Msg, topic, from, to);
}

void URobofleetBase::PublishImuMsg(const std::string& TopicName, const std::string& Namespace, const Imu& Msg)
{
	std::string topic = "sensor_msgs/Imu";
	std::string from = "/" + TopicName;
	std::string to = "/" + TopicName;

	if (!Namespace.empty()) {
		from = "/" + Namespace + from;
		to = "/" + Namespace + to;
	}

	EncodeRosMsg<Imu>(Msg, topic, from, to);
}

// TODO : Remove amrl_msgs
void URobofleetBase::PublishLocationMsg(FString RobotName, RobotLocationStamped& LocationMsg)
{ // Publish a Location Message to Robofleet
	std::string topic = "amrl_msgs/Localization2DMsg";
	std::string from = "/localization";
	std::string to = "/" + std::string(TCHAR_TO_UTF8(*RobotName)) + "/localization";
	EncodeRosMsg<RobotLocationStamped>(LocationMsg, topic, from, to);
}

void URobofleetBase::PublishAgentStatusMsg(const FString& RobotName, const AgentStatus& StatusMsg)
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

void URobofleetBase::PublishBoundingObject3DArrayMsg(const std::string& TopicName, const std::string& Namespace, const BoundingObject3DArray& Msg)
{
	std::string topic = "augre_msgs/BoundingObject3DArray";
	std::string from = "/" + TopicName;
	std::string to = "/" + TopicName;

	if (!Namespace.empty()) {
		from = "/" + Namespace + from;
		to = "/" + Namespace + to;
	}	
	EncodeRosMsg<BoundingObject3DArray>(Msg, topic, from, to);
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

// Odometry 
void URobofleetBase::PublishOdomMsg(const FString& AgentName, const Odometry& OdometryMsg)
{
	std::string topic = "nav_msgs/Odometry";
	std::string from = "/" + std::string(TCHAR_TO_UTF8(*AgentName)) + "/odometry";
	std::string to = "/" + std::string(TCHAR_TO_UTF8(*AgentName)) + "/odometry";
	EncodeRosMsg<Odometry>(OdometryMsg, topic, from, to);
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

void URobofleetBase::PublishNavigationPath(const FString& RobotName, const Path& PathMsg)
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
	//UE_LOG(LogTemp, Warning, TEXT("[PublishTwistMsg : ... %s"), *RobotName);
	EncodeRosMsg<Twist>(TwistMsg, topic, from, to);
}

void URobofleetBase::PublishTwistStampedMsg(const FString& RobotName, const FString& TopicName, const TwistStamped& TwistStampedMsg)
{
	std::string topic = "geometry_msgs/TwistStamped";
	std::string from = "/" + std::string(TCHAR_TO_UTF8(*RobotName)) + "/" + std::string(TCHAR_TO_UTF8(*TopicName));
	std::string to = "/" + std::string(TCHAR_TO_UTF8(*RobotName)) + "/" + std::string(TCHAR_TO_UTF8(*TopicName));
	//UE_LOG(LogTemp, Warning, TEXT("[PublishTwistStampedMsg : ... %s"), *RobotName);
	EncodeRosMsg<TwistStamped>(TwistStampedMsg, topic, from, to);
}

void URobofleetBase::PublishTFMessage(const TFMessage& TFMessageMsg)
{
	std::string topic = "TF2_msgs/TFMessage";
	std::string from = "/augre/tf";
	std::string to = "/augre/tf";
	// UE_LOG(LogTemp, Warning, TEXT("[PublishTFMessageMsg : ..."));
	EncodeRosMsg<TFMessage>(TFMessageMsg, topic, from, to);
}

// to replace the above function
void URobofleetBase::PublishTFMsg(const FString& TopicName, const FString& Namespace, const TFMessage& TFMessageMsg)
{
	std::string topic = "TF2_msgs/TFMessage";
	std::string from = "/" + std::string(TCHAR_TO_UTF8(*TopicName));
	std::string to = "/" + std::string(TCHAR_TO_UTF8(*TopicName));
	if (!Namespace.IsEmpty()) {
		from = "/" + std::string(TCHAR_TO_UTF8(*Namespace)) + from;
		to = "/" + std::string(TCHAR_TO_UTF8(*Namespace)) + to;
	}

	EncodeRosMsg<TFMessage>(TFMessageMsg, topic, from, to);
}

void URobofleetBase::PublishPoseStamped(const FString& RobotUid, const FString& TopicName, const PoseStamped& PoseStampedMsg)
{
	std::string topic = "geometry_msgs/PoseStamped";
	std::string from = "/" + std::string(TCHAR_TO_UTF8(*RobotUid)) + "/" + std::string(TCHAR_TO_UTF8(*TopicName)) + "/pose";
	std::string to = "/" + std::string(TCHAR_TO_UTF8(*RobotUid)) + "/" + std::string(TCHAR_TO_UTF8(*TopicName)) + "/pose";

	EncodeRosMsg<PoseStamped>(PoseStampedMsg, topic, from, to);
}

//void URobofleetBase::PublishPolygonStamped(const std::string& TopicName, const std::string& Namespace, const PolygonStamped& Msg)
//{
//	PolygonStamped poly_msg = Msg;
//	std::string topic = "geometry_msgs/PolygonStamped";
//	std::string from = "/" + TopicName;
//	std::string to = "/" + TopicName;
//
//	if (!Namespace.empty()) {
//		from = "/" + Namespace + from;
//		to = "/" + Namespace + to;
//	}
//
//	EncodeRosMsg<PolygonStamped>(poly_msg, topic, from, to);
//}




void URobofleetBase::PublishCancel(const FString& RobotUid, const FString& TopicName)
{
	const Empty cancel_msg;
	std::string topic = "std_msgs/Empty";
	std::string from = "/" + std::string(TCHAR_TO_UTF8(*RobotUid)) + std::string(TCHAR_TO_UTF8(*TopicName)) + "/cancel";
	std::string to = "/" + std::string(TCHAR_TO_UTF8(*RobotUid)) + std::string(TCHAR_TO_UTF8(*TopicName)) + "/cancel";
	// UE_LOG(LogTemp, Warning, TEXT("[PublishTFMessageMsg : ..."));
	EncodeRosMsg<Empty>(cancel_msg, topic, from, to);
}

void URobofleetBase::PublishEmptyMsg(const FString& TopicName, const FString& Namespace)
{
	const Empty cancel_msg;
	std::string topic = "std_msgs/Empty";
	std::string from = "/" + std::string(TCHAR_TO_UTF8(*TopicName));
	std::string to = "/" + std::string(TCHAR_TO_UTF8(*TopicName));

	if (!Namespace.IsEmpty()) {
		from = "/" + std::string(TCHAR_TO_UTF8(*Namespace)) + from;
		to = "/" + std::string(TCHAR_TO_UTF8(*Namespace)) + to;
	}

	EncodeRosMsg<Empty>(cancel_msg, topic, from, to);
}

void URobofleetBase::PublishUInt8MultiArrayMsg(const std::string& TopicName, const std::string& Namespace, const UInt8MultiArray& Msg)
{
	std::string topic = "std_msgs/UInt8MultiArray";
	std::string from = "/" + TopicName;
	std::string to = "/" + TopicName;

	if (!Namespace.empty()) {
		from = "/" + Namespace + from;
		to = "/" + Namespace + to;
	}

	EncodeRosMsg<UInt8MultiArray>(Msg, topic, from, to);
}

void URobofleetBase::PublishStringMsg(const std::string& TopicName, const std::string& Namespace, const std::string& StringMessage)
{
	String msg;
	/*msg.data = std::string(TCHAR_TO_UTF8(*StringMessage));*/
	msg.data = StringMessage;
	std::string topic = "std_msgs/String";
	std::string from = "/" + TopicName;
	std::string to = "/" + TopicName;

	if (!Namespace.empty()) {
		from = "/" + Namespace + from;
		to = "/" + Namespace + to;
	}

	EncodeRosMsg<String>(msg, topic, from, to);
}

void URobofleetBase::PublishUInt8Msg(const std::string& TopicName, const std::string& Namespace, const uint8_t& UInt8Message)
{
	UInt8 msg;
	msg.data = UInt8Message;
	std::string topic = "std_msgs/UInt8";
	std::string from = "/" + TopicName;
	std::string to = "/" + TopicName;

	if (!Namespace.empty()) {
		from = "/" + Namespace + from;
		to = "/" + Namespace + to;
	}

	EncodeRosMsg<UInt8>(msg, topic, from, to);
}

void URobofleetBase::PublishAudioData(const std::string& TopicName, const std::string& Namespace, const AudioData& Msg)
{
	std::string topic = "audio_common_msgs/AudioData";
	std::string from = "/" + TopicName;
	std::string to = "/" + TopicName;

	if (!Namespace.empty()) {
		from = "/" + Namespace + from;
		to = "/" + Namespace + to;
	}

	EncodeRosMsg<AudioData>(Msg, topic, from, to);
}

void URobofleetBase::PublishAudioDataStamped(const std::string& TopicName, const std::string& Namespace, const AudioDataStamped& Msg)
{
	std::string topic = "audio_common_msgs/AudioDataStamped";
	std::string from = "/" + TopicName;
	std::string to = "/" + TopicName;

	if (!Namespace.empty()) {
		from = "/" + Namespace + from;
		to = "/" + Namespace + to;
	}

	EncodeRosMsg<AudioDataStamped>(Msg, topic, from, to);
	UE_LOG(LogTemp, Warning, TEXT("Published Audio Data Stamped"));
}

void URobofleetBase::PublishAudioInfo(const std::string& TopicName, const std::string& Namespace, const AudioInfo& Msg)
{
	std::string topic = "audio_common_msgs/AudioInfo";
	std::string from = "/" + TopicName;
	std::string to = "/" + TopicName;

	if (!Namespace.empty()) {
		from = "/" + Namespace + from;
		to = "/" + Namespace + to;
	}

	EncodeRosMsg<AudioInfo>(Msg, topic, from, to);
	UE_LOG(LogTemp, Warning, TEXT("Published Audio Info"));
}

void URobofleetBase::PublishBoolMsg(const FString& TopicName, const FString& Namespace, const bool& cmd)
{
	Bool request;
	request.data = cmd;
	std::string topic = "std_msgs/Bool";
	std::string from = "/" + std::string(TCHAR_TO_UTF8(*Namespace)) + "/" + std::string(TCHAR_TO_UTF8(*TopicName));
	std::string to = "/" + std::string(TCHAR_TO_UTF8(*Namespace)) + "/" + std::string(TCHAR_TO_UTF8(*TopicName));  // */pv_request
	EncodeRosMsg<Bool>(request, topic, from, to);
}

void URobofleetBase::PublishHapticsResearchMsg(const FString& RobotName, const PoseStamped& PoseStampedMsg)
{
	// Publish a mo Message to Robofleet
	std::string topic = "geometry_msgs/PoseStamped";
	std::string from = "/haptics/recorded_data";
	std::string to = "/haptics/recorded_data";
	EncodeRosMsg<PoseStamped>(PoseStampedMsg, topic, from, to);
}

/// <summary>
/// String command with augmented information for parser node (GPT or T5)
/// </summary>
/// <param name="cmd: Natural Language instruction"></param>
void URobofleetBase::PublishStringCommand(const FString& cmd)
{
	String test;
	test.data = std::string(TCHAR_TO_UTF8(*cmd));
	std::string topic = "std_msgs/String";
	std::string from = "/umrf_parser/cmd";
	std::string to = "/umrf_parser/cmd";
	UE_LOG(LogTemp, Warning, TEXT("[Publish String Cmd : ...  %s"), *cmd);
	EncodeRosMsg<String>(test, topic, from, to);
}

void URobofleetBase::PublishDetection(const DetectedItem_augre& Detection)
{
	std::string topic = "augre_msgs/DetectedItem";
	std::string from = "/hololens/detection";
	std::string to = "/hololens/detection";
	UE_LOG(LogTemp, Warning, TEXT("[Publish Detection : ..."));
	EncodeRosMsg<DetectedItem_augre>(Detection, topic, from, to);
}

void URobofleetBase::PublishCompressedImageMsg(const FString& TopicName, const FString& Namespace, const CompressedImage& Msg)
{
	std::string topic = "sensor_msgs/CompressedImage";
	std::string from = "/" + std::string(TCHAR_TO_UTF8(*TopicName));
	std::string to = "/" + std::string(TCHAR_TO_UTF8(*TopicName));
	if (!Namespace.IsEmpty()) {
		from = "/" + std::string(TCHAR_TO_UTF8(*Namespace)) + from;
		to = "/" + std::string(TCHAR_TO_UTF8(*Namespace)) + to;
	}

	EncodeRosMsg<CompressedImage>(Msg, topic, from, to);
}

void  URobofleetBase::PublishImageMsg(const FString& TopicName, const FString& Namespace, const Image& Msg)
{
	std::string topic = "sensor_msgs/Image";
	std::string from = "/" + std::string(TCHAR_TO_UTF8(*TopicName));
	std::string to = "/" + std::string(TCHAR_TO_UTF8(*TopicName));
	if (!Namespace.IsEmpty()) {
		from = "/" + std::string(TCHAR_TO_UTF8(*Namespace)) + from;
		to = "/" + std::string(TCHAR_TO_UTF8(*Namespace)) + to;
	}
	EncodeRosMsg<Image>(Msg, topic, from, to);
}


void  URobofleetBase::PublishPointCloudMsg(const FString& TopicName, const FString& Namespace, const PointCloud2& Msg)
{
	std::string topic = "sensor_msgs/PointCloud2";
	std::string from = "/" + std::string(TCHAR_TO_UTF8(*TopicName));
	std::string to = "/" + std::string(TCHAR_TO_UTF8(*TopicName));
	if (!Namespace.IsEmpty()) {
		from = "/" + std::string(TCHAR_TO_UTF8(*Namespace)) + from;
		to = "/" + std::string(TCHAR_TO_UTF8(*Namespace)) + to;
	}
	EncodeRosMsg<PointCloud2>(Msg, topic, from, to);
}

/*
* Agent Status Messages
*/

bool URobofleetBase::IsAgentPublishingStatusMsg(const FString& TfNamespace)
{
	FString TfNamespacestd = FString(TCHAR_TO_UTF8(*TfNamespace));
	if (AgentStatusMap.find(TfNamespacestd) != AgentStatusMap.end()) { return true; }
	return false;
}

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

FString URobofleetBase::GetAgentCallsign(const FString& RobotName)
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
	return FVector(TransformStampedMap[RobotNamestd].transform.translation.x,
		TransformStampedMap[RobotNamestd].transform.translation.y,
		TransformStampedMap[RobotNamestd].transform.translation.z);
}

void URobofleetBase::GetGoalPose(const FString& RobotName, PoseStamped& PoseStamped, bool& MsgReceived)
{
	MsgReceived = false;
	// Why are we doing this in every function? Kept to stay consitent, but believe it should be a std::string.
	FString RobotNamestd = FString(TCHAR_TO_UTF8(*RobotName));
	if (GoalPoseMap.count(RobotNamestd) == 0) return;
	MsgReceived = true;
	PoseStamped = GoalPoseMap[RobotNamestd];
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

// used for ar-star study
void URobofleetBase::GetStudyModality(const std::string& RobotName, Header& Msg)
{
	FString RobotNamestd = FString(UTF8_TO_TCHAR(RobotName.c_str()));
	UE_LOG(LogRobofleet, Warning, TEXT("RobotName: %s"), *RobotNamestd);
	auto iter = StudyModalityMap.find(RobotNamestd);

	if (iter == StudyModalityMap.end()) {
		UE_LOG(LogRobofleet, Warning, TEXT("[ERROR] URobofleetBase::GetStudyModality(): Header message does not exist."));
	}
	else {
		UE_LOG(LogRobofleet, Warning, TEXT("Getting the Study Modality"));
		Msg = StudyModalityMap[RobotNamestd];
	}
}


/*
* Compressed Image Message Methods
*/

TSet<FString> URobofleetBase::GetImageStreams(const FString& RobotName)
{
	TSet<FString> image_streams;

	if (RobotImageMap.count(RobotName) == 0) return image_streams;

	for (const auto img_stream : RobotImageMap[RobotName]) {
		image_streams.Emplace(img_stream.first);
	}

	return image_streams;
}

void URobofleetBase::GetRobotImage(const FString& RobotName, const FString& StreamName, TArray<uint8>& Image)
{
	//FString RobotNamestd = FString(TCHAR_TO_UTF8(*RobotName));
	//FString StreamNamestd = FString(TCHAR_TO_UTF8(*StreamName));
	if (RobotImageMap.count(RobotName) == 0) return;
	if (RobotImageMap[RobotName].count(StreamName) == 0) return;
	Image = TArray<uint8>(&RobotImageMap[RobotName][StreamName]->data[0], RobotImageMap[RobotName][StreamName]->data.size());
}

bool URobofleetBase::IsRobotImageCompressed(const FString& RobotName, const FString& StreamName)
{
	//FString RobotNamestd = FString(TCHAR_TO_UTF8(*RobotName));
	//FString StreamNamestd = FString(TCHAR_TO_UTF8(*StreamName));
	if (RobotImageMap[RobotName][StreamName]->format.find("compressed") != std::string::npos)
	{
		return true;
	}
	else return false;
}

/*
* Occupancy Grid Message Methods
*/
FMapMetaData URobofleetBase::GetOccupancyGridInfo(const FString& RobotName)
{
	FString RobotNamestd = FString(TCHAR_TO_UTF8(*RobotName));

	if (OccupancyGridMap.count(RobotNamestd) == 0)
	{
		return FMapMetaData();
	}
	else
	{
		FMapMetaData mdata_;
		FTime t_;
		FTransform p_;
		t_._nsec = OccupancyGridMap[RobotNamestd].info.map_load_time._nsec;
		t_._sec = OccupancyGridMap[RobotNamestd].info.map_load_time._sec;
		FVector position = FVector(OccupancyGridMap[RobotNamestd].info.origin.position.x,
			OccupancyGridMap[RobotNamestd].info.origin.position.y,
			OccupancyGridMap[RobotNamestd].info.origin.position.z);
		FQuat orientation = FQuat(OccupancyGridMap[RobotNamestd].info.origin.orientation.x,
			OccupancyGridMap[RobotNamestd].info.origin.orientation.y,
			OccupancyGridMap[RobotNamestd].info.origin.orientation.z,
			OccupancyGridMap[RobotNamestd].info.origin.orientation.w);
		p_.SetTranslation(position);
		p_.SetRotation(orientation);

		mdata_.map_load_time = t_;
		mdata_.resolution = OccupancyGridMap[RobotNamestd].info.resolution;
		mdata_.width = static_cast<int32>(OccupancyGridMap[RobotNamestd].info.width);
		mdata_.height = static_cast<int32>(OccupancyGridMap[RobotNamestd].info.height);
		mdata_.origin = p_;

		return mdata_;
	}
}

TArray<uint8> URobofleetBase::GetOccupancyGridImage(const FString& RobotName)
{
	FString RobotNamestd = FString(TCHAR_TO_UTF8(*RobotName));
	const auto* map = &OccupancyGridMap[RobotNamestd];
	if (OccupancyGridMap.count(RobotNamestd) == 0)
	{
		return TArray<uint8>();
	}
	else
	{
		int32 gridSize = OccupancyGridMap[RobotNamestd].data.size();
		int width = OccupancyGridMap[RobotNamestd].info.width;
		int height = OccupancyGridMap[RobotNamestd].info.height;
		int numOfCells = gridSize / sizeof(int8);
		TArray<uint8> gridImage;
		// initialize image array with all values set to 100
		gridImage.Init(100, numOfCells * 4);
		int8_t* val = nullptr;
		uint8 r, g, b, a;

		// build image from top left (0) or bottom left (1)
		int rowOrder = 1;
		switch (rowOrder) {
		case 0:
			// translate input from top row down
			//UE_LOG(LogRobofleet, Display, TEXT("Translating grid from top down"));
			for (auto i = 0; i < numOfCells; i++) {
				val = &(OccupancyGridMap[RobotNamestd].data[i]);
				// cells having value -1 are set to grey with semi-transparency
				if (*val < 0) {
					// multiplying by 4 to converts the greyscale value to an rgba value
					gridImage[i * 4 + 3] = 0;
				}
				else {
					r = 255 - *val * (255 / 100);
					g = 0;
					b = *val * (255 / 100);
					a = 255;
					gridImage[i * 4] = r;
					gridImage[i * 4 + 1] = g;
					gridImage[i * 4 + 2] = b;
					gridImage[i * 4 + 3] = a;
				}
			}
			break;
		case 1:
			//translate image from bottom row up
			// UE_LOG(LogRobofleet, Display, TEXT("Translating grid from bottom up"));
			//for (int i = (height - 1); i >= 0; i--) {
			for (int i = 0; i < height; i++) {
				for (int j = 0; j < width; j++) {
					// the position of a pixel with 2d loc (x,y) in a 1d array is (x + (y * width))
					int k = j + (i * width);
					// get the position when height (y) is inverted
					int l = j + ((height-i-1) * width);
					val = &(OccupancyGridMap[RobotNamestd].data[k]);
					// the OG will have negative values for unoccupied space

					// k = x + y*width
					// 
					if (*val < 0) {
						// set alpha channel to 30 of 255
						gridImage[l * 4 + 3] = 30;
					}
					else {
						r = 255 - *val * (255 / 100);
						g = 0;
						b = *val * (255 / 100);
						a = 255;
						// multiplying by 4 to converts the greyscale value to an rgba value
						gridImage[l * 4] = r;
						gridImage[l * 4 + 1] = g;
						gridImage[l * 4 + 2] = b;
						gridImage[l * 4 + 3] = a;
					}
				}
			}
			break;
		}
		// UE_LOG(LogTemp, Display, TEXT("returning image of size %i"), gridImage.GetAllocatedSize());
		return gridImage;
	}
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

bool URobofleetBase::isFrameAvailable(const FString& FrameName)
{
	// TODO: THIS IS NOT IDEAL. A CONVENTION SHOULD BE PROVIDED TO ROBOT
	if (FrameInfoMap.count(FrameName) == 0) return false;
	return true;
}

bool URobofleetBase::isAnchorFrame(const FString& FrameName)
{
	return FrameName.Contains("anchor");
}

/*
* Detected Item Message Methods
*/

FString URobofleetBase::ConvertAsaToFrameId(const FString& asa, const FString& tf_prefix) {
	std::string frame_id = std::string(TCHAR_TO_UTF8(*asa));
	std::replace(frame_id.begin(), frame_id.end(), '-', '_');

	frame_id = std::string(TCHAR_TO_UTF8(*tf_prefix)) + frame_id;
	return FString(UTF8_TO_TCHAR(frame_id.c_str()));
}

FString URobofleetBase::ConvertFrameIdToAsa(const FString& frame_id, const FString& tf_prefix) {
	std::string frame = std::string(TCHAR_TO_UTF8(*frame_id));
	std::string prefix = std::string(TCHAR_TO_UTF8(*tf_prefix));
	std::string asa;

	if (frame.size() != prefix.size() + 36) {
		return "";
	}

	asa = frame.substr(prefix.size(), frame.size());
	std::replace(asa.begin(), asa.end(), '_', '-');

	if (!isValidUuid(asa)) {
		return "";
	}

	return FString(UTF8_TO_TCHAR(asa.c_str()));;
}

FString URobofleetBase::GetDetectedName(const FString& DetectedItemUid)
{
	if (DetectedItemAugreMap.count(DetectedItemUid) == 0) return "Item unavailable";
	return FString(DetectedItemAugreMap[DetectedItemUid].callsign.c_str());
}

FString URobofleetBase::GetDetectedType(const FString& DetectedItemUid)
{
	if (DetectedItemAugreMap.count(DetectedItemUid) == 0) return "Item unavailable";
	return FString(DetectedItemAugreMap[DetectedItemUid].type.c_str());
}

FString URobofleetBase::GetDetectedTypeLabel(const FString& DetectedItemUid)
{
	if (DetectedItemAugreMap.count(DetectedItemUid) == 0) return "Item unavailable";
	return FString(DetectedItemAugreMap[DetectedItemUid].type_label.c_str());
}

FString URobofleetBase::GetDetectedHow(const FString& DetectedItemUid)
{
	if (DetectedItemAugreMap.count(DetectedItemUid) == 0) return "Item unavailable";
	return FString(DetectedItemAugreMap[DetectedItemUid].how.c_str());
}

FString URobofleetBase::GetDetectedHowLabel(const FString& DetectedItemUid)
{
	if (DetectedItemAugreMap.count(DetectedItemUid) == 0) return "Item unavailable";
	return FString(DetectedItemAugreMap[DetectedItemUid].how_label.c_str());
}

FPoseStamped URobofleetBase::GetDetectedItemPose(const FString& DetectedItemUid)
{
	if (DetectedItemAugreMap.count(DetectedItemUid) == 0) return FPoseStamped();
	FPoseStamped item_pose;
	item_pose.header.frame_id = DetectedItemAugreMap[DetectedItemUid].pose.header.frame_id.c_str();
	item_pose.Transform.SetTranslation(FVector(DetectedItemAugreMap[DetectedItemUid].pose.pose.position.x,
		DetectedItemAugreMap[DetectedItemUid].pose.pose.position.y,
		DetectedItemAugreMap[DetectedItemUid].pose.pose.position.z));
	item_pose.Transform.SetRotation(FQuat(DetectedItemAugreMap[DetectedItemUid].pose.pose.orientation.x,
		DetectedItemAugreMap[DetectedItemUid].pose.pose.orientation.y,
		DetectedItemAugreMap[DetectedItemUid].pose.pose.orientation.z,
		DetectedItemAugreMap[DetectedItemUid].pose.pose.orientation.w));
	item_pose.Transform.SetScale3D(FVector(1, 1, 1));
	return item_pose;
}

FDateTime URobofleetBase::GetDetectedItemTimeStamped(const FString& DetectedItemUid)
{
	if (DetectedItemAugreMap.count(DetectedItemUid) == 0) return FDateTime();
	double GMT = -6;
	double nsec = DetectedItemAugreMap[DetectedItemUid].pose.header.stamp._nsec / 1000000000;
	FDateTime timeStamped = FDateTime::FromUnixTimestamp(DetectedItemAugreMap[DetectedItemUid].pose.header.stamp._sec);
	timeStamped += FTimespan::FromHours(GMT);
	// UE_LOG(LogTemp, Warning, TEXT("Converted FDateTime: %s"), *timeStamped.ToString());
	
	return timeStamped;
}

TArray<uint8> URobofleetBase::GetDetectedImage(const FString& DetectedItemUid)
{
	if (DetectedItemAugreMap.count(DetectedItemUid) == 0) {
		return TArray<uint8>();
	}
	else {
		UE_LOG(LogRobofleet, Warning, TEXT("size %d"), DetectedItemAugreMap[DetectedItemUid].cmpr_image.data.size());
		return 	TArray<uint8>(&DetectedItemAugreMap[DetectedItemUid].cmpr_image.data[0], DetectedItemAugreMap[DetectedItemUid].cmpr_image.data.size());
	}
}

FVector URobofleetBase::GetDetectedImageSize(const FString& DetectedItemUid)
{
	//FString object_name = FString(TCHAR_TO_UTF8(*ObjectName));
	if (DetectedItemAugreMap.count(DetectedItemUid) == 0 || !(DetectedItemAugreMap[DetectedItemUid].cmpr_image.data.size() >= 0.0)) {
		return FVector();
	}
	else {
		TArray<uint8> image = TArray<uint8>(&DetectedItemAugreMap[DetectedItemUid].cmpr_image.data[0], DetectedItemAugreMap[DetectedItemUid].cmpr_image.data.size());
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

void URobofleetBase::IsPointCloudReceived(const FString& RobotName, bool& MsgReceived)
{
	FString RobotNamestd = FString(TCHAR_TO_UTF8(*RobotName));

	if (PointCloudMap.find(RobotNamestd) == PointCloudMap.end()) {
		UE_LOG(LogRobofleet, Warning, TEXT("[ERROR] URobofleetBase::IsPointCloudReceived(): No PointCloudMap was received yet."));
		MsgReceived = false;
		return;
	}

	MsgReceived = true;
}

void URobofleetBase::GetPointCloudFrame(const FString& RobotName, FString& PointCloudFrame)
{
	FString RobotNamestd = FString(TCHAR_TO_UTF8(*RobotName));
	auto iter = PointCloudMap.find(RobotNamestd);

	if (iter == PointCloudMap.end()) {
		UE_LOG(LogRobofleet, Warning, TEXT("[ERROR] URobofleetBase::GetPointCloudFrame(): Point Cloud Frame exists."));
	}
	else {
		UE_LOG(LogRobofleet, Warning, TEXT("Getting the Point Cloud Frame"));
		PointCloudFrame = FString(UTF8_TO_TCHAR(iter->second->header.frame_id.c_str()));
	}
}

void URobofleetBase::GetPointCloud(const FString& RobotName, TSharedPtr<PointCloud2>& PointCloud, bool& MsgReceived)
{
	FString RobotNamestd = FString(TCHAR_TO_UTF8(*RobotName));
	auto iter = PointCloudMap.find(RobotNamestd);

	if (iter == PointCloudMap.end())
	{
		UE_LOG(LogRobofleet, Warning, TEXT("[ERROR] In URobofleetBase::GetPointCloud(): No key exists in PointCloudMap"));
		PointCloud.Reset();
		MsgReceived = false;
	}
	else
	{
		PointCloud = iter->second;
		MsgReceived = true;
	}

	//if (PointCloudMap.count(RobotNamestd) == 0) {
	//	UE_LOG(LogRobofleet, Warning, TEXT("[ERROR] In URobofleetBase::GetPointCloud(): No key exists in PointCloudMap"));
	//}
	//else {
	//	PointCloud = PointCloudMap[RobotName];
	//}
}

void URobofleetBase::GetMarkerArray(const FString& RobotName, TSharedPtr<MarkerArray>& Markers)
{
	FString RobotNamestd = FString(TCHAR_TO_UTF8(*RobotName));
	if (MarkerArrayMap.count(RobotNamestd) == 0) {
		UE_LOG(LogRobofleet, Warning, TEXT("[ERROR] In URobofleetBase::GetMarkerArray(): No key exists in MarkerArrayMap"));
	}
	else {
		Markers = MarkerArrayMap[RobotName];
		//UE_LOG(LogTemp, Warning, TEXT("Get array size of MarkerArrayMap: %d"), MarkerArrayMap[RobotName]->markers.size());
	}
}

FVector URobofleetBase::GetDetectedItemPosition(const FString& DetectedItemUid)
{
	FString DetectedItemUidStd = FString(TCHAR_TO_UTF8(*DetectedItemUid));
	if (DetectedItemAugreMap.count(DetectedItemUidStd) == 0) return FVector(-1, -1, -1);
	return FVector(DetectedItemAugreMap[DetectedItemUidStd].pose.pose.position.x,
		DetectedItemAugreMap[DetectedItemUidStd].pose.pose.position.y,
		DetectedItemAugreMap[DetectedItemUidStd].pose.pose.position.z);
}

FString URobofleetBase::GetDetectedItemImageURL(const FString& DetectedItemUid)
{
	if (DetectedItemAugreMap.count(DetectedItemUid) == 0) return "NoUrl";
	return FString(DetectedItemAugreMap[DetectedItemUid].url.c_str());
}

TArray<FString> URobofleetBase::GetAllDetectedItems()
{
	TArray<FString> ListOfDetectedItems;

	for (auto& it : DetectedItemAugreMap)
	{
		ListOfDetectedItems.Add(it.first);
	}
	return ListOfDetectedItems;
}

void URobofleetBase::RemoveDetectedItem(const FString& DetectedItemUid)
{
	FString DetectedItemUidStd = FString(TCHAR_TO_UTF8(*DetectedItemUid));
	DetectedItemAugreMap.erase(DetectedItemUidStd);
}

void URobofleetBase::AddDetectionToAugReMap(const DetectedItem_augre& Detection)
{
	FString DetectedItemUid = FString(Detection.uid.c_str());
	FString frame_id = Detection.pose.header.frame_id.c_str();
	DetectedItemAugreMap[DetectedItemUid] = Detection;
	
	OnDetectedItemReceived.Broadcast(DetectedItemUid, frame_id);
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
	return FVector(ScrewParametersMap[RobotNamestd].pose.position.x,
		ScrewParametersMap[RobotNamestd].pose.position.y,
		ScrewParametersMap[RobotNamestd].pose.position.z);
}

FVector URobofleetBase::GetScrewAxis(const FString& RobotName)
{
	FString RobotNamestd = FString(TCHAR_TO_UTF8(*RobotName));
	return FVector(ScrewParametersMap[RobotNamestd].pose.orientation.x,
		ScrewParametersMap[RobotNamestd].pose.orientation.y,
		ScrewParametersMap[RobotNamestd].pose.orientation.z);
}

float URobofleetBase::GetScrewAxisPitch(const FString& RobotName)
{
	FString RobotNamestd = FString(TCHAR_TO_UTF8(*RobotName));
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

FPath URobofleetBase::GetFPath(const FString& RobotName, const FString& Type)
{
	// convert from Path to FPath
	FString RobotNamestd = FString(TCHAR_TO_UTF8(*RobotName));
	TArray<FPoseStamped> poses;

	Path p;
	if (Type == "global_path")
	{
		p = GlobalPath[RobotNamestd];
	}
	else if (Type == "trail_path")
	{
		p = TrailPath[RobotNamestd];
	}
	else if (Type == "twist_path")
	{
		p = TwistPath[RobotNamestd];
	}
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
* Natural Input Commands
*/


// /////////////////////////////////////////
// \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

void URobofleetBase::GetGestureResult(const FString& RobotName, std::string& Result)
{
	//FString RobotNamestd = FString(TCHAR_TO_UTF8(RobotName.c_str()));
	UE_LOG(LogTemp, Warning, TEXT("Gesture Name: %s"), *RobotName);
	Result = GestureCmdMap[RobotName].data;
	UE_LOG(LogTemp, Warning, TEXT("Gesture Result: %s"), *FString(Result.c_str()));
}

void URobofleetBase::GetSpeechResult(const FString& RobotName, std::string& Result)
{
	//FString RobotNamestd = FString(TCHAR_TO_UTF8(RobotName.c_str()));
	Result = SpeechCmdMap[RobotName].data;
}

void URobofleetBase::GetFusedResult(const FString& RobotName, std::string& Result)
{
	//FString RobotNamestd = FString(TCHAR_TO_UTF8(RobotName.c_str()));
	Result = FusedCmdMap[RobotName].data;
}


/*
* Leg Tracker Detections
*/
void URobofleetBase::GetNonLegClusters(const FString& RobotName, DetectionArray& NonLegClusterArray)
{
	FString RobotNamestd = FString(TCHAR_TO_UTF8(*RobotName));
	if (LegTrackingMap.count(RobotNamestd) == 0) {
		UE_LOG(LogRobofleet, Warning, TEXT("[ERROR] In URobofleetBase::GetNonLegClusters(): No key exists in LegTrackingMap"));
	}
	else {
		NonLegClusterArray = LegTrackingMap[RobotName]->NonLegClusters;
	}
}

void URobofleetBase::GetDetectedLegClusters(const FString& RobotName, DetectionArray& DetectedLegClusterArray)
{
	FString RobotNamestd = FString(TCHAR_TO_UTF8(*RobotName));
	if (LegTrackingMap.count(RobotNamestd) == 0) {
		UE_LOG(LogRobofleet, Warning, TEXT("[ERROR] In URobofleetBase::GetDetectedLegClusters(): No key exists in LegTrackingMap"));
	}
	else {
		DetectedLegClusterArray = LegTrackingMap[RobotName]->DetectedLegClusters;
	}
}

void URobofleetBase::GetPeopleDetected(const FString& RobotName, PersonArray& PeopleDetectedArray)
{
	FString RobotNamestd = FString(TCHAR_TO_UTF8(*RobotName));
	if (LegTrackingMap.count(RobotNamestd) == 0) {
		UE_LOG(LogRobofleet, Warning, TEXT("[ERROR] In URobofleetBase::GetPeopleDetected(): No key exists in LegTrackingMap"));
	}
	else {
		PeopleDetectedArray = LegTrackingMap[RobotName]->PeopleDetected;
	}
}

void URobofleetBase::GetPeopleTracked(const FString& RobotName, PersonArray& PeopleTrackedArray)
{
	FString RobotNamestd = FString(TCHAR_TO_UTF8(*RobotName));
	if (LegTrackingMap.count(RobotNamestd) == 0) {
		UE_LOG(LogRobofleet, Warning, TEXT("[ERROR] In URobofleetBase::GetPeopleTracked(): No key exists in LegTrackingMap"));
	}
	else {
		PeopleTrackedArray = LegTrackingMap[RobotName]->PeopleTracker;
	}
}

bool URobofleetBase::isValidUuid(const FString& id) {
	// From: https://www.regextester.com/99148
	const std::regex uuid_regex(
		"[0-9a-fA-F]{8}\\-[0-9a-fA-F]{4}\\-[0-9a-fA-F]{4}\\-[0-9a-fA-F]{4}\\-[0-"
		"9a-fA-F]{12}");
	return std::regex_match(std::string(TCHAR_TO_UTF8(*id)), uuid_regex);
}

bool URobofleetBase::isValidUuid(const std::string& id) {
	// From: https://www.regextester.com/99148
	const std::regex uuid_regex(
		"[0-9a-fA-F]{8}\\-[0-9a-fA-F]{4}\\-[0-9a-fA-F]{4}\\-[0-9a-fA-F]{4}\\-[0-"
		"9a-fA-F]{12}");
	return std::regex_match(id, uuid_regex);
}
