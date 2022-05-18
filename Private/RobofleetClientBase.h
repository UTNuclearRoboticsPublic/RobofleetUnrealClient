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

//Define Log Category and Verbosity
DECLARE_LOG_CATEGORY_EXTERN(LogRobofleet, Log, All);

//OnNewRobotSeen event
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnNewRobotSeen, FString, RobotName);

//OnRobotPruned event
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnRobotPruned, FString, RobotName);

//OnImageRecevied  event
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnImageReceived, FString, RobotName);

//OnDetectedItemRecevied  event
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnDetectedItemReceived, FString, RobotName);

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
	std::map<FString, TSharedPtr<TransformStamped>> TransformStampedMap;
	std::map<FString, CompressedImage> RobotImageMap;
	std::map<FString, FDateTime> RobotsSeenTime;
	std::map<FString, DetectedItem> DetectedItemMap;
	std::map<FString, DetectedItem_augre> DetectedItemAugreMap;
	std::map<FString, NavSatFix> NavSatFixMap;
	std::map<FString, Pose> PoseMap;
	std::map<FString, Path> RobotPath;
	std::map<FString, FLinearColor> ColorGlobalPath;
	std::map<FString, FLinearColor> ColorTwistPath;
	std::map<FString, FLinearColor> ColorTrailPath;
	std::set<FString> RobotsSeen = {};

	NavSatFix WorldGeoOrigin;
	bool bIsWorldGeoOriginSet;

	template <typename T> 
	typename T DecodeMsg(const void* Data);

	void DecodeMsg(const void* Data, FString topic, FString RobotNamespace);

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

	//AugRe_msgs
	FString GetName(const FString& RobotName);

	FString GetDisplayName(const FString& RobotName);

	FString GetAgentType(const FString& RobotName);

	float GetRobotBatteryLevel(const FString& RobotName);

	FString GetOwner(const FString& RobotName);

	FString GetControlStatus(const FString& RobotName);	

	FString GetRobotLocationString(const FString& RobotName);

	FVector GetRobotPosition(const FString& RobotName);

	TArray<uint8> GetRobotImage(const FString& RobotName);

	bool IsRobotImageCompressed(const FString& RobotName);

	TArray<FString> GetAllRobotsAtSite(const FString& Location);

	FString GetDetectedName(const FString& RobotName);

	FString GetDetectedRepIDRef(const FString& RobotName);

	FString GetDetectedAnchorIDRef(const FString& RobotName);

	FVector GetDetectedPositionRef(const FString& RobotName);

	FVector GetDetectedPositionGlobal(const FString& RobotName);

	Path GetPath(const FString& RobotName);

	FPath GetFPath(const FString& RobotName);

	void AssingBaseColor(const FString& RobotName);

	bool IsRobotOk(const FString& RobotName);

	void PrintRobotsSeen();
	
	UFUNCTION()
	void RefreshRobotList();

	void PruneInactiveRobots();
	
	void RegisterRobotStatusSubscription();

	void RegisterRobotSubscription(FString TopicName, FString RobotName);

	void PublishStatusMsg(FString RobotName, RobotStatus& RobotStatus);

	void PublishLocationMsg(FString RobotName, RobotLocationStamped& LocationMsg);

	void PublishAgentStatusMsg(const FString& RobotName, const AgentStatus& AgentStatus);

	void PublishTransformWithCovarianceStampedMsg(const FString& Robotname, const TransformWithCovarianceStamped& TFwithCovStamped);

	void PublishMoveBaseSimpleGoal(const FString& RobotName, const PoseStamped& PoseStampedMsg);

	void PublishPath(const FString& RobotName, const Path& PathMsg);

	void PublishHololensOdom(const FString& RobotName, const PoseStamped& PoseStampedMsg);

	void PublishStartUMRFMsg(StartUMRF& StartUMRFMsg);

	UPROPERTY(BlueprintAssignable, Category = "Robofleet")
	FOnNewRobotSeen OnNewRobotSeen;

	UPROPERTY(BlueprintAssignable, Category = "Robofleet")
	FOnRobotPruned OnRobotPruned;

	UPROPERTY(BlueprintAssignable, Category = "Robofleet")
	FOnImageReceived OnImageReceived;

	UPROPERTY(BlueprintAssignable, Category = "Robofleet")
	FOnDetectedItemReceived OnDetectedItemReceived;

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