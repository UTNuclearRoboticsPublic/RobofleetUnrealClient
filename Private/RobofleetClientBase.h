#pragma once

#include "CoreMinimal.h"
#include "UObject/Object.h"
#include "include/schema_generated.h"
#include "decode.hpp"
#include "encode.hpp"
#include "flatbuffers/flatbuffers.h"
#include "message_structs.h"
#include "MessageSchedulerLib.hpp"
#include "WebsocketClient.h"
#include "RobofleetBPMessageStructs.h"

#include <cstdint>
#include <map>
#include <string>

#include "RobofleetClientBase.generated.h"

struct RobotData {
	RobotLocation Location;
	RobotStatus Status;
	bool IsAlive;
};

struct FrameInfo {
	TransformStamped TransformStamped;	// Filled with the relative transforms from tfMessage
	FTransform WorldTransform;			// Transform of actor(tf) in worldFrame
	int64 age;							// time.now - last_update
	bool visible;						// Toggle on off each partivular frame... Not implemented yet
};

struct LegTrackingData {
	DetectionArray DetectedLegClusters;
	DetectionArray NonLegClusters;
	PersonArray PeopleDetected;
	PersonArray PeopleTracker;
};

//Define Log Category and Verbosity
DECLARE_LOG_CATEGORY_EXTERN(LogRobofleet, Log, All);

//OnNewRobotSeen event
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnNewRobotSeen, FString, RobotName);

//OnNewAnchorSeen event
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnNewAnchorSeen, FString, AsaId);

//OnRobotPruned event
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnRobotPruned, FString, RobotName);

//ResetAllAgentSeen event
DECLARE_DYNAMIC_MULTICAST_DELEGATE(FOnResetAllAgentsSeen);

//OnImageRecevied  event
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnImageReceived, FString, RobotName);

//OnOccupacyGridRecevied  event
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnOccupancyGridReceived, FString, RobotName);

//OnDetectedItemRecevied  event
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FOnDetectedItemReceived, FString, DetectedItemUid, FString, frame_id);

//OnScrewParametersReceived  event
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnScrewParametersReceived, FString, RobotName);

//OnRobotChangedLocation event
DECLARE_DYNAMIC_MULTICAST_DELEGATE_ThreeParams(FOnRobotLocationChanged, FString, RobotName, FString, OldSite, FString, NewSite);

//OnAgentStatusUpdate event
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnAgentStatusUpdate, FString, RobotName);

//OnPathRecevied  event
DECLARE_DYNAMIC_MULTICAST_DELEGATE_ThreeParams(FOnPathReceived, FString, Tag, FPath, RobotPath, FLinearColor, Color);

//OnAgentStatusUpdate event
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnDetectedLegClusterReceived, FString, RobotName);

//DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnPathReceived, FString, RobotName);

UCLASS(Blueprintable)
class ROBOFLEETUNREALCLIENT_API URobofleetBase : public UObject
{
	GENERATED_BODY()

public:
	// TODO: expose constructor to blueprints
	URobofleetBase();
	~URobofleetBase();

private:

	int MaxQueueBeforeWaiting;
	int Verbosity = 0;
	bool bIsInitilized = false;

	UPROPERTY()
	UWebsocketClient* SocketClient;

	FTimerHandle RefreshTimerHandle;
	
	std::map<FString, TSharedPtr<RobotData> > RobotMap;
	std::map<FString, TSharedPtr<LegTrackingData> > LegTrackingMap;
	std::map<FString, AgentStatus> AgentStatusMap;
	std::map<FString, TransformStamped> TransformStampedMap;
	std::map<FString, TSharedPtr<FrameInfo>> FrameInfoMap;
	std::map<FString, CompressedImage> RobotImageMap;
	std::map<FString, FDateTime> RobotsSeenTime;
	std::map<FString, FString> AnchorMap;
	std::map<FString, DetectedItem_augre> DetectedItemAugreMap;
	std::map<FString, PoseStamped> ScrewParametersMap;
	std::map<FString, NavSatFix> NavSatFixMap;
	std::map<FString, Pose> PoseMap;
	std::map<FString, Path> RobotPath;
	std::map<FString, FLinearColor> ColorGlobalPath;
	std::map<FString, FLinearColor> ColorTwistPath;
	std::map<FString, FLinearColor> ColorTrailPath;
	std::set<FString> RobotsSeen = {};
	std::set<FString> AnchorGTSAM = {};
	std::map<FString, OccupancyGrid> OccupancyGridMap;
	std::map<FString, DetectionArray> LegDetectionMap;

	NavSatFix WorldGeoOrigin;
	bool bIsWorldGeoOriginSet;

	template <typename T> 
	typename T DecodeMsg(const void* Data);

	void DecodeMsg(const void* Data, FString topic, FString RobotNamespace);

	void DecodeTFMsg(const void* Data);

	template <typename T> 
	void EncodeRosMsg(
		const T& msg,
		const std::string& msg_type,
		std::string& from_topic,
		const std::string& to_topic);

	void WebsocketDataCB(const void* Data);

	std::string WorldOrigin = "WorldOrigin";
	FString FWorldOrigin = FString(WorldOrigin.c_str());

public:
	
	bool IsInitilized();
	bool IsConnected();
	// TODO: Move the Blueprint exposure to the BP function library

	void Initialize(FString HostUrl, const UObject* WorldContextObject);

	void Disconnect();

	void SetWorldGeoOrigin(NavSatFix OriginPose);

	FString GetRobotStatus(const FString& RobotName);

	// ***********************************************************
	// augre_msgs/agent_status getters
	FString GetUidFromAgentStatus(const FString& RobotName);

	FString GetAgentCallsign(const FString& RobotName);

	FString GetAgentType(const FString& RobotName);

	float GetRobotBatteryLevel(const FString& RobotName);

	FString GetOwner(const FString& RobotName);

	FString GetControlStatus(const FString& RobotName);	

	// ***********************************************************
	// geometry_msgs/TransformStamped getters

	FString GetRobotLocationString(const FString& RobotName);

	FVector GetRobotPosition(const FString& RobotName);

	FTransform GetFrameTransform(const FString& NodeName);

	void SetFrameWorldTransform(const FString& NodeName, const FTransform& ActorWorldTransform);

	FTransform GetFrameWorldTransform(const FString& NodeName);

	int GetAgeTransform(const FString& NodeName);

	TArray<uint8> GetRobotImage(const FString& RobotName);     // image_raw/compressed

	bool IsRobotImageCompressed(const FString& RobotName);

	TArray<FString> GetAllRobotsAtSite(const FString& Location);

	TArray<FString> GetAllAgents();

	TArray<FString> GetAllFrames();

	bool isFrameAvailable(const FString& FrameName);

	bool IsAgentPublishingStatusMsg(const FString& AgentUid);

	TArray<FString> GetChildrenFrameId(const FString& NodeName);

	FTransform LookupTransform(const FString& target_frame, const FString& source_frame);

	// ***********************************************************
	// nav_msgs/OccupancyGrid getters
	FMapMetaData GetOccupancyGridInfo(const FString& RobotName);
	TArray<uint8> GetOccupancyGridImage(const FString& RobotName);

	// ***********************************************************
	// augre_msgs/DetectedItem getters

	FString GetDetectedName(const FString& RobotName);		//callsign

	FString GetDetectedType(const FString& RobotName);

	FString GetDetectedTypeLabel(const FString& RobotName);

	FString GetDetectedHow(const FString& RobotName);

	FString GetDetectedHowLabel(const FString& RobotName);

	FPoseStamped GetDetectedItemPose(const FString& DetectedItemUid);

	TArray<uint8> GetDetectedImage(const FString& RobotName);

	FVector GetDetectedImageSize(const FString& ObjectName);

	FVector GetDetectedItemPosition(const FString& DetectedItemUid);

	TArray<FString> GetAllDetectedItems();

	void RemoveDetectedItem(const FString& DetectedItemUid);

	// ***********************************************************

	FVector GetScrewAxisPoint(const FString& RobotName);

	FVector GetScrewAxis(const FString& RobotName);

	float GetScrewAxisPitch(const FString& RobotName);

	Path GetPath(const FString& RobotName);

	FPath GetFPath(const FString& RobotName);

	void AssingBaseColor(const FString& RobotName);

	bool IsRobotOk(const FString& RobotName);

	void PrintRobotsSeen();

	void GetNonLegClusters(const FString& RobotName, DetectionArray& NonLegClusterArray);
	
	void GetDetectedLegClusters(const FString& RobotName, DetectionArray& DetectedLegClusterArray);

	void GetPeopleDetected(const FString& RobotName, PersonArray& PeopleDetectedArray);

	void GetPeopleTracked(const FString& RobotName, PersonArray& PeopleTrackedArray);

	bool isValidUuid(const FString& id);

	UFUNCTION()
	void RefreshRobotList();

	void PruneInactiveRobots();

	void updateTFFrames();

	void ResetAllAgentsSeen();
	
	void RegisterRobotStatusSubscription();

	void RegisterRobotSubscription(FString TopicName, FString RobotName);

	void PublishStatusMsg(FString RobotName, RobotStatus& RobotStatus);

	void PublishLocationMsg(FString RobotName, RobotLocationStamped& LocationMsg);

	// augre_msgs/agent_status
	void PublishAgentStatusMsg(const FString& RobotName, const AgentStatus& AgentStatus);

	// augre_msgs/TransformWithCovarianceStamped
	void PublishTransformWithCovarianceStampedMsg(const FString& TopicName, const TransformWithCovarianceStamped& TFwithCovStamped);

	void PublishAzureSpatialAnchorMsg(const FString& RobotName, const AzureSpatialAnchor& RosAzureSpatialAnchor);

	void PublishMoveBaseSimpleGoal(const FString& RobotName, const PoseStamped& PoseStampedMsg);

	void PublishHandPose(const FString& RobotName, const PoseStamped& PoseStampedMsg);

	void PublishNavigationPath(const FString& RobotName, const Path& PathMsg);

	void PublishTwistMsg(const FString& RobotName, const FString& TopicName, const Twist& TwistMsg);

	void PublishTwistStampedMsg(const FString& RobotName, const FString& TopicName, const TwistStamped& TwistStampedMsg);

	void PublishTFMessage(const TFMessage& TFMessageMsg);

	void PublishTFMsg(const FString& TopicName, const FString& Namespace, const TFMessage& TFMessageMsg); // **to replace PublishTFMessage

	void PublishHololensOdom(const FString& RobotName, const PoseStamped& PoseStampedMsg);

	void PublishStartUMRFMsg(StartUMRF& StartUMRFMsg);
	
	void PublishStopUMRFMsg(StopUMRF& StopUMRFMsg);

	void PublishPoseStamped(const FString& RobotUid, const FString& TopicName, const PoseStamped& PoseStampedMsg);

	void PublishCancel(const FString& RobotUid, const FString& TopicName);

	void PublishEmptyMsg(const FString& TopicName, const FString& Namespace);

	void PublishHapticsResearchMsg(const FString& RobotName, const PoseStamped& PoseStampedMsg);

	void PublishStringCommand(const FString& cmd);

	UPROPERTY(BlueprintAssignable, Category = "Robofleet")
	FOnNewRobotSeen OnNewRobotSeen;

	UPROPERTY(BlueprintAssignable, Category = "Robofleet")
	FOnNewAnchorSeen OnNewAnchorSeen;

	UPROPERTY(BlueprintAssignable, Category = "Robofleet")
	FOnRobotPruned OnRobotPruned;

	UPROPERTY(BlueprintAssignable, Category = "Robofleet")
	FOnImageReceived OnImageReceived;

	UPROPERTY(BlueprintAssignable, Category = "Robofleet")
		FOnOccupancyGridReceived OnOccupancyGridReceived;

	UPROPERTY(BlueprintAssignable, Category = "Robofleet")
	FOnDetectedItemReceived OnDetectedItemReceived;

	UPROPERTY(BlueprintAssignable, Category = "Robofleet")
	FOnScrewParametersReceived OnScrewParametersReceived;

	UPROPERTY(BlueprintAssignable, Category = "Robofleet")
	FOnRobotLocationChanged OnRobotLocationChanged;

	UPROPERTY(BlueprintAssignable, Category = "Robofleet")
	FOnAgentStatusUpdate OnAgentStatusUpdate;

	UPROPERTY(BlueprintAssignable, Category = "Robofleet")
	FOnPathReceived OnPathReceived;

	UPROPERTY(BlueprintAssignable, Category = "Robofleet")
	FOnDetectedLegClusterReceived OnDetectedLegClusterReceived;

	UPROPERTY(BlueprintAssignable, Category = "Robofleet")
	FOnResetAllAgentsSeen OnResetAllAgentsSeen;


	//TODO: fix this terrible Idea for demo crunch. This is an extremely hacky way to avoid GC
	UFUNCTION(BlueprintCallable)
	void RemoveObjectFromRoot();

	void ConvertToCartesian(const NavSatFix& GeoPose, const FString RobotNamespace);
};