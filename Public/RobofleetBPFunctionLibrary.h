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
	static void PublishNavigationPath(const FString& RobotName, const FPath& PathMsg);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static void PublishTwistMsg(const FString& RobotName, const FString& TopicName, const FTwist& TwistMsg);

	static void PublishStartUMRFMsg(const FStartUMRF& StartUMRFMsg);

	// need to add detected image

	// Use only for delegates
	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static URobofleetBase* GetClientReference();

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static FPath GetRobotPath(const FString& RobotName);
};