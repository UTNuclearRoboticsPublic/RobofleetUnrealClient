// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "RobofleetBPMessageStructs.h"
#include "RobofleetBPFunctionLibrary.generated.h"

class URobofleetBase;

/**
 * 
 */
UCLASS()
class ROBOFLEETUNREALCLIENT_API	URobofleetBPFunctionLibrary : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()
	
	UFUNCTION(BlueprintCallable, Category = "Robofleet", meta = (WorldContext = "WorldContextObject"))
	static void StartRobofleetSession(FString HostUrl, const UObject* WorldContextObject);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static FString GetRobotStatus(const FString& RobotName);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static void ResetAllAgentsSeen();

	// Augre_msgs
	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static bool IsAgentPublishingStatusMsg(const FString& TfNamespace);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static FString GetUidFromAgentStatus(const FString& RobotName);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static FString GetAgentCallsign(const FString& RobotName);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static FString GetAgentType(const FString& RobotName);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static float GetRobotBatteryLevel(const FString& RobotName);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static FString GetOwner(const FString& RobotName);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static FString GetControlStatus(const FString& RobotName);	

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static FString GetRobotLocationString(const FString& RobotName);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static FVector GetRobotPosition(const FString& RobotName);
	
	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static TArray<FString> GetAllRobotsAtSite(const FString& Location);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static TArray<FString> GetAllAgents();

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static TArray<FString> GetAllFrames();

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static bool isFrameAvailable(const FString& FrameName);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static FTransform GetFrameTransform(const FString& NodeName);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static FTransform GetFrameWorldTransform(const FString& NodeName);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static TArray<FString> GetChildrenFrameId(const FString& NodeName);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static FTransform LookupTransform(const FString& target_frame, const FString& source_frame);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static bool IsRobotOk(const FString& RobotName);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static TArray<uint8> GetRobotImage(const FString& RobotName);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static bool IsRobotImageCompressed(const FString& RobotName);

	UFUNCTION(BlueprintCallable, Category = "Robofleet | HeatMap")
	static TArray<uint8> GetOccupancyGridImage(const FString& RobotName);

	UFUNCTION(BlueprintCallable, Category = "Robofleet | HeatMap")
	FMapMetaData GetOccupancyGridInfo(const FString& RobotName);

	//UFUNCTION(BlueprintCallable, Category = "HeatMap")
	//static TArray<uint8> ConvertGridtoRGBA(const TArray<int8>& OccupancyGridData);
	 
	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static void PrintRobotsSeen();

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static void RegisterRobotSubscription(FString TopicName, FString RobotName);

	// Detected Item
	UFUNCTION(BlueprintCallable, Category = "Robofleet | DetectedItem")
	static FString GetDetectedName(const FString& DetectedItemUid);

	UFUNCTION(BlueprintCallable, Category = "Robofleet | DetectedItem")
	static FString GetDetectedType(const FString& DetectedItemUid);

	UFUNCTION(BlueprintCallable, Category = "Robofleet | DetectedItem")
	static FString GetDetectedTypeLabel(const FString& DetectedItemUid);

	UFUNCTION(BlueprintCallable, Category = "Robofleet | DetectedItem")
	static FString GetDetectedHow(const FString& DetectedItemUid);

	UFUNCTION(BlueprintCallable, Category = "Robofleet | DetectedItem")
	static FString GetDetectedHowLabel(const FString& DetectedItemUid);

	UFUNCTION(BlueprintCallable, Category = "Robofleet | DetectedItem")
	FPoseStamped GetDetectedItemPose(const FString& DetectedItemUid);
	
	UFUNCTION(BlueprintCallable, Category = "Robofleet | DetectedItem")
	static TArray<uint8> GetDetectedImage(const FString& DetectedItemUid);

	UFUNCTION(BlueprintCallable, Category = "Robofleet | DetectedItem")
	static FVector GetDetectedImageSize(const FString& DetectedItemUid);

	UFUNCTION(BlueprintCallable, Category = "Robofleet | DetectedItem")
	static FVector GetDetectedItemPosition(const FString& DetectedItemUid);

	UFUNCTION(BlueprintCallable, Category = "Robofleet | DetectedItem")
	static TArray<FString> GetAllDetectedItems();

	UFUNCTION(BlueprintCallable, Category = "Robofleet|Screw Axis")
	static FVector GetScrewAxisPoint(const FString& RobotName);

	UFUNCTION(BlueprintCallable, Category = "Robofleet|Screw Axis")
	static FVector GetScrewAxis(const FString& RobotName);

	UFUNCTION(BlueprintCallable, Category = "Robofleet|Screw Axis")
	static float GetScrewAxisPitch(const FString& RobotName);

	UFUNCTION(BlueprintCallable, Category = "Robofleet|Leg Tracker")
	static void GetNonLegClusters(const FString& RobotName, FDetectionArray& NonLegClusterArray_);

	UFUNCTION(BlueprintCallable, Category = "Robofleet|Leg Tracker")
	static void GetDetectedLegClusters(const FString& RobotName, FDetectionArray& DetectedLegClusterArray_);

	UFUNCTION(BlueprintCallable, Category = "Robofleet|Leg Tracker")
	static void GetPeopleDetected(const FString& RobotName, FPersonArray& PeopleDetectedArray_);

	UFUNCTION(BlueprintCallable, Category = "Robofleet|Leg Tracker")
	static void GetPeopleTracked(const FString& RobotName, FPersonArray& PeopleTrackedArray_);


	// Publish Messages to Robofleet
	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static void PublishStatusMsg(const FString& RobotName, const FRobotStatus& StatusMsg);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static void PublishLocationMsg(const FString& RobotName, const FRobotLocationStamped& LocationMsg);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static void PublishAgentStatusMsg(const FString& RobotName, const FAgentStatus& StatusMsg);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static void PublishTransformWithCovarianceStampedMsg(const FString& TopicName, const FTransformWithCovarianceStamped& FTfWithCovarianceStampedmsg);
	
	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static void PublishAzureSpatialAnchorMsg(const FString& RobotName, const FAzureSpatialAnchor& FAzureSpatialAnchorMsg);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static void PublishHololensOdom(const FString& RobotName, const FPoseStamped& PoseStampedMsg);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static void PublishPoseStamped(const FString& RobotUid, const FString& TopicName, const FPoseStamped& PoseStampedMsg);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static void PublishCancel(const FString& RobotUid, const FString& TopicName);
	
	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static void PublishEmptyMsg(const FString& TopicName, const FString& Namespace);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static void PublishMoveBaseSimpleGoal(const FString& RobotName, const FPoseStamped& PoseStampedMsg);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static void PublishHandPose(const FString& RobotName, const FPoseStamped& PoseStampedMsg, FDateTime CurTimeStamp);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static void PublishNavigationPath(const FString& RobotName, const FPath& PathMsg);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static void PublishTwistMsg(const FString& RobotName, const FString& TopicName, const FTwist& TwistMsg);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static void PublishTwistStampedMsg(const FString& RobotName, const FString& TopicName, const FTwistStamped& TwistStampedMsg);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static void PublishTFMessageMsg(const FTFMessage& TFMessageMsg);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static void PublishTFMsg(const FString& TopicName, const FString& Namespace, const FTFMessage& TFMessageMsg);

	UFUNCTION(BlueprintCallable, Category = "TeMoto")
	static void PublishStartUMRFMsg(const FStartUMRF& StartUMRFMsg);

	UFUNCTION(BlueprintCallable, Category = "TeMoto")
	static void PublishStopUMRFMsg(const FStopUMRF& StopUMRFMsg);

	UFUNCTION(BlueprintCallable, Category = "Robofleet|Haptics Research")
	static void PublishHapticsResearchMsg(const FString& RobotName, const FPoseStamped& PoseStampedMsg, FDateTime CurTimeStamp);

	UFUNCTION(BlueprintCallable, Category = "TeMoto")
	static void PublishStringCommand(const FString& cmd);

	

	

	// Use only for delegates
	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static URobofleetBase* GetClientReference();

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static FPath GetRobotPath(const FString& RobotName);
};