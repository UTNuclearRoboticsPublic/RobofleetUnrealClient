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

//OnImageRecevied  event
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnImageReceived, FString, RobotName);

//OnDetectedItemRecevied  event
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnDetectedItemReceived, FString, DetectedItemUid);

//OnScrewParametersReceived  event
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnScrewParametersReceived, FString, RobotName);

//OnRobotChangedLocation event
DECLARE_DYNAMIC_MULTICAST_DELEGATE_ThreeParams(FOnRobotLocationChanged, FString, RobotName, FString, OldSite, FString, NewSite);

//OnAgentStatusUpdate event
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnAgentStatusUpdate, FString, RobotName);

//OnPathRecevied  event
DECLARE_DYNAMIC_MULTICAST_DELEGATE_ThreeParams(FOnPathReceived, FString, Tag, FPath, RobotPath, FLinearColor, Color);
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
	std::map<FString, AgentStatus> AgentStatusMap;
	std::map<FString, TransformStamped> TransformStampedMap;
	std::map<FString, CompressedImage> RobotImageMap;
	std::map<FString, FDateTime> RobotsSeenTime;
	std::map<FString, DetectedItem> DetectedItemMap;		//TODO remove 
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
	std::map<FString, TSharedPtr<LegTrackingData>> LegTrackingMap;

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

	FString GetAgentDisplayName(const FString& RobotName);

	FString GetAgentType(const FString& RobotName);

	float GetRobotBatteryLevel(const FString& RobotName);

	FString GetOwner(const FString& RobotName);

	FString GetControlStatus(const FString& RobotName);	

	// ***********************************************************
	// geometry_msgs/TransformStamped getters

	FString GetRobotLocationString(const FString& RobotName);

	FVector GetRobotPosition(const FString& RobotName);

	TArray<uint8> GetRobotImage(const FString& RobotName);

	bool IsRobotImageCompressed(const FString& RobotName);

	TArray<FString> GetAllRobotsAtSite(const FString& Location);

	TArray<FString> GetAllAgents();

	FString GetDetectedName(const FString& RobotName);

	//FString GetDetectedRepIDRef(const FString& RobotName);

	//FString GetDetectedAnchorIDRef(const FString& RobotName);

	FVector GetDetectedPositionRef(const FString& RobotName);

	//FVector GetDetectedPositionGlobal(const FString& RobotName);

	TArray<uint8> GetDetectedImage(const FString& RobotName);

	FVector GetDetectedImageSize(const FString& ObjectName);

	//FString GetDetectedItemAsaId(const FString& DetectedItemUid);

	FVector GetDetectedItemPosition(const FString& DetectedItemUid);

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

	UFUNCTION()
	void RefreshRobotList();

	void PruneInactiveRobots();

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

	void PublishPath(const FString& RobotName, const Path& PathMsg);

	void PublishTwistMsg(const FString& RobotName, const FString& TopicName, const Twist& TwistMsg);

	void PublishTwistStampedMsg(const FString& RobotName, const FString& TopicName, const TwistStamped& TwistStampedMsg);

	void PublishHololensOdom(const FString& RobotName, const PoseStamped& PoseStampedMsg);

	void PublishStartUMRFMsg(StartUMRF& StartUMRFMsg);
	
	void PublishStopUMRFMsg(StopUMRF& StopUMRFMsg);

	UPROPERTY(BlueprintAssignable, Category = "Robofleet")
	FOnNewRobotSeen OnNewRobotSeen;

	UPROPERTY(BlueprintAssignable, Category = "Robofleet")
	FOnNewAnchorSeen OnNewAnchorSeen;

	UPROPERTY(BlueprintAssignable, Category = "Robofleet")
	FOnRobotPruned OnRobotPruned;

	UPROPERTY(BlueprintAssignable, Category = "Robofleet")
	FOnImageReceived OnImageReceived;

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

	//TODO: fix this terrible Idea for demo crunch. This is an extremely hacky way to avoid GC
	UFUNCTION(BlueprintCallable)
	void RemoveObjectFromRoot();

	void ConvertToCartesian(const NavSatFix& GeoPose, const FString RobotNamespace);
};