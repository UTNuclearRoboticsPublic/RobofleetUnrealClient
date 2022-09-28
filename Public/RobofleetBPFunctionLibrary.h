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

	// Augre_msgs
	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static FString GetName(const FString& RobotName);	

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static FString GetAgentDisplayName(const FString& RobotName);

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
	static bool IsRobotOk(const FString& RobotName);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static TArray<uint8> GetRobotImage(const FString& RobotName);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static bool IsRobotImageCompressed(const FString& RobotName);
	 
	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static void PrintRobotsSeen();

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static void RegisterRobotSubscription(FString TopicName, FString RobotName);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static FString GetDetectedName(const FString& RobotName);

	//TODO: REMOVE REP ID
	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static FString GetDetectedRepIDRef(const FString& RobotName);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static FString GetDetectedAnchorIDRef(const FString& RobotName);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static FVector GetDetectedPositionRef(const FString& RobotName);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static FVector GetDetectedPositionGlobal(const FString& RobotName);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static TArray<uint8> GetDetectedImage(const FString& RobotName);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static FVector GetDetectedImageSize(const FString& ObjectName);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static FString GetDetectedItemAsaId(const FString& DetectedItemUid);
	
	UFUNCTION(BlueprintCallable, Category = "Robofleet")
		static FVector GetDetectedItemPosition(const FString& DetectedItemUid);

	UFUNCTION(BlueprintCallable, Category = "Robofleet|Screw Axis")
	static FVector GetScrewAxisPoint(const FString& RobotName);

	UFUNCTION(BlueprintCallable, Category = "Robofleet|Screw Axis")
	static FVector GetScrewAxis(const FString& RobotName);

	UFUNCTION(BlueprintCallable, Category = "Robofleet|Screw Axis")
	static float GetScrewAxisPitch(const FString& RobotName);

	UFUNCTION(BlueprintCallable, Category = "Robofleet|Leg Tracker")
	static void GetNonLegClusters(const FString& RobotName, FDetectionArray& NonLegClusterArray);

	UFUNCTION(BlueprintCallable, Category = "Robofleet|Leg Tracker")
	static void GetDetectedLegClusters(const FString& RobotName, FDetectionArray& DetectedLegClusterArray);

	UFUNCTION(BlueprintCallable, Category = "Robofleet|Leg Tracker")
	static void GetPeopleDetected(const FString& RobotName, FPersonArray& PeopleDetectedArray);

	UFUNCTION(BlueprintCallable, Category = "Robofleet|Leg Tracker")
	static void GetPeopleTracked(const FString& RobotName, FPersonArray& PeopleTrackedArray);


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
	static void PublishMoveBaseSimpleGoal(const FString& RobotName, const FPoseStamped& PoseStampedMsg);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static void PublishHandPose(const FString& RobotName, const FPoseStamped& PoseStampedMsg, FDateTime CurTimeStamp);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static void PublishNavigationPath(const FString& RobotName, const FPath& PathMsg);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static void PublishTwistMsg(const FString& RobotName, const FString& TopicName, const FTwist& TwistMsg);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static void PublishTwistStampedMsg(const FString& RobotName, const FString& TopicName, const FTwistStamped& TwistStampedMsg);

	static void PublishStartUMRFMsg(const FStartUMRF& StartUMRFMsg);

	static void PublishStopUMRFMsg(const FStopUMRF& StopUMRFMsg);

	
	

	// need to add detected image

	// Use only for delegates
	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static URobofleetBase* GetClientReference();

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static FPath GetRobotPath(const FString& RobotName);
};