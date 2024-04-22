// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Templates/SharedPointer.h"

#include "Kismet/BlueprintFunctionLibrary.h"
#include "Engine/Texture2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "IImageWrapper.h"
#include "IImageWrapperModule.h"

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
	static void GetGoalPose(const FString& RobotName, FPoseStamped& PoseStamped, bool& MsgReceived);

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
	static void GetRobotImage(const FString& RobotName, const FString& StreamName, TArray<uint8>& Image);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static TSet<FString> GetImageStreams(const FString& RobotName);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static bool IsRobotImageCompressed(const FString& RobotName, const FString& StreamName);

	UFUNCTION(BlueprintCallable, Category = "Robofleet | HeatMap")
	static TArray<uint8> GetOccupancyGridImage(const FString& RobotName);

	UFUNCTION(BlueprintCallable, Category = "Robofleet | HeatMap")
	static FMapMetaData GetOccupancyGridInfo(const FString& RobotName);

	UFUNCTION(BlueprintCallable, Category = "Robofleet | HeatMap")
	static FTransform CreateGridToAnchorTransform(const FMapMetaData& OccupancyGridInfo, 
		const FTransform& AnchorTransform);

	//UFUNCTION(BlueprintCallable, Category = "HeatMap")
	//static TArray<uint8> ConvertGridtoRGBA(const TArray<int8>& OccupancyGridData);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static void PrintRobotsSeen();

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static void RegisterRobotSubscription(FString TopicName, FString RobotName);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static FString ConvertAsaToFrameId(const FString& asa, const FString& tf_prefix);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static FString ConvertFrameIdToAsa(const FString& frame_id, const FString& tf_prefix);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static FPose ConvertPoseToRightHandMeters(const FPose& pose_in);

	UFUNCTION(BlueprintCallable, Category = "Robofleet | HeatMap")
	static FTransform ConvertTransformToLeftHand(const FTransform ROSPose);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static FVector ConvertPositionToRightHandMeters(const FVector& pos_in);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static FQuat ConvertQuaternionToRightHand(const FQuat& rot_in);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static FVector ConvertDegToRad(const FVector& omega_vec);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static FVector ConvertAngleToRightHand(const FVector& omega_vec);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static FRotator ConvertEulerToRightHand(const FRotator& rot_in);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static FQuat ConvertEulerToRightHandQuaternion(const FRotator& rot_in);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static FRotator ConvertQuaternionToRightHandEuler(const FQuat& rot_in);

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
	static FPoseStamped GetDetectedItemPose(const FString& DetectedItemUid);

	UFUNCTION(BlueprintCallable, Category = "Robofleet | DetectedItem")
	static FDateTime GetDetectedItemTimeStamped(const FString& DetectedItemUid);

	UFUNCTION(BlueprintCallable, Category = "Robofleet | DetectedItem")
	static TArray<uint8> GetDetectedImage(const FString& DetectedItemUid);

	UFUNCTION(BlueprintCallable, Category = "Robofleet | DetectedItem")
	static FVector GetDetectedImageSize(const FString& DetectedItemUid);

	UFUNCTION(BlueprintCallable, Category = "Robofleet | DetectedItem")
	static FVector GetDetectedItemPosition(const FString& DetectedItemUid);

	UFUNCTION(BlueprintCallable, Category = "Robofleet | DetectedItem")
	static FString GetDetectedItemImageURL(const FString& DetectedItemUid);

	UFUNCTION(BlueprintCallable, Category = "Robofleet | DetectedItem")
	static TArray<FString> GetAllDetectedItems();

	UFUNCTION(BlueprintCallable, Category = "Robofleet | DetectedItem")
	static void RemoveDetectedItem(const FString& DetectedItemUid);

	UFUNCTION(BlueprintCallable, Category = "Robofleet|Screw Axis")
	static FVector GetScrewAxisPoint(const FString& RobotName);

	UFUNCTION(BlueprintCallable, Category = "Robofleet|Screw Axis")
	static FVector GetScrewAxis(const FString& RobotName);

	UFUNCTION(BlueprintCallable, Category = "Robofleet|Screw Axis")
	static float GetScrewAxisPitch(const FString& RobotName);

	UFUNCTION(BlueprintCallable, Category = "Robofleet|Natural Gestures")
	static void GetGestureResult(const FString& RobotName, FString& Result);

	UFUNCTION(BlueprintCallable, Category = "Robofleet|Natural Gestures")
	static void GetSpeechResult(const FString& RobotName, FString& Result);

	UFUNCTION(BlueprintCallable, Category = "Robofleet|Natural Gestures")
	static void GetFusedResult(const FString& RobotName, FString& Result);

	UFUNCTION(BlueprintCallable, Category = "Robofleet|Leg Tracker")
	static void GetNonLegClusters(const FString& RobotName, FDetectionArray& NonLegClusterArray_);

	UFUNCTION(BlueprintCallable, Category = "Robofleet|Leg Tracker")
	static void GetDetectedLegClusters(const FString& RobotName, FDetectionArray& DetectedLegClusterArray_);

	UFUNCTION(BlueprintCallable, Category = "Robofleet|Leg Tracker")
	static void GetPeopleDetected(const FString& RobotName, FPersonArray& PeopleDetectedArray_);

	UFUNCTION(BlueprintCallable, Category = "Robofleet|Leg Tracker")
	static void GetPeopleTracked(const FString& RobotName, FPersonArray& PeopleTrackedArray_);

	UFUNCTION(BlueprintCallable, Category = "Robofleet|Sensor Msgs")
	static void IsPointCloudReceived(const FString& RobotName, bool& MsgReceived);

	UFUNCTION(BlueprintCallable, Category = "Robofleet|Sensor Msgs")
	static void GetPointCloudFrame(const FString& RobotName, FString& PointCloudFrame);

	UFUNCTION(BlueprintCallable, Category = "Robofleet|Sensor Msgs")
	static void GetPointCloud(const FString& RobotName, FPointCloud2& Msg, bool& MsgReceived);

	UFUNCTION(BlueprintCallable, Category = "Robofleet|Visualization Msgs")
	static void GetMarkerArray(const FString& RobotName, FMarkerArray& Msg);

	// Publish Messages to Robofleet
	// TODO: To remove
	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static void PublishLocationMsg(const FString& RobotName, const FRobotLocationStamped& LocationMsg);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static void PublishAgentStatusMsg(const FString& RobotName, const FAgentStatus& StatusMsg);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static void PublishTransformWithCovarianceStampedMsg(const FString& TopicName, const FTransformWithCovarianceStamped& FTfWithCovarianceStampedmsg);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static void PublishBoundingObject3DArrayMsg(const FString& TopicName, const FString& Namespace, const FBoundingObject3DArray& Msg);

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
	static void PublishStringMsg(const FString& TopicName, const FString& Namespace, const FString& StringMessage);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static void PublishUInt8Msg(const FString& TopicName, const FString& Namespace, const int& UInt8Message);

	UFUNCTION(BlueprintCallable, Category = "Robofleet|Audio Common Msgs")
	static void PublishAudioData(const FString& TopicName, const FString& Namespace, const FAudioData& Msg);

	UFUNCTION(BlueprintCallable, Category = "Robofleet|Audio Common Msgs")
	static void PublishAudioDataStamped(const FString& TopicName, const FString& Namespace, const FAudioDataStamped& Msg);

	UFUNCTION(BlueprintCallable, Category = "Robofleet|Audio Common Msgs")
	static void PublishAudioInfo(const FString& TopicName, const FString& Namespace, const FAudioInfo& Msg);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static void PublishBoolMsg(const FString& TopicName, const FString& Namespace, const bool& cmd);

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

	UFUNCTION(BlueprintCallable, Category = "Robofleet|Haptics Research")
	static void PublishHapticsResearchMsg(const FString& RobotName, const FPoseStamped& PoseStampedMsg, FDateTime CurTimeStamp);

	UFUNCTION(BlueprintCallable, Category = "Robofleet | Hololens Sensors")
	static void PublishPointCloudMsg(const FString& TopicName, const FString& Namespace, const FPointCloud2& Msg);

	UFUNCTION(BlueprintCallable, Category = "TeMoto")
	static void PublishStringCommand(const FString& cmd);

	UFUNCTION(BlueprintCallable, Category = "Robofleet | DetectedItem")
	static void PublishDetection(const FDetectedItem& detection);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static void PublishImuMsg(const FString& TopicName, const FString& Namespace, const FImu& Msg);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static void PublishOdomMsg(const FString& AgentName, const FOdometry& Msg);


	//UFUNCTION(BlueprintCallable, Category = "Robofleet | DetectedItem")
	//static void AddDetection(const FDetectedItem& detection);

	// Use only for delegates
	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static URobofleetBase* GetClientReference();

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static FPath GetRobotPath(const FString& RobotName, const FString& Type);

	UFUNCTION(BlueprintCallable, Category = "Robofleet | Hololens Sensors")
	static void PublishVLCImageMsg(const FString& TopicName, const FString& Namespace, const FImageROS& ImageMsg);

	UFUNCTION(BlueprintCallable, Category = "Robofleet | Hololens Sensors")
	static void PublishVLCCompressedImageMsg(const FString& TopicName, const FString& Namespace, const FCompressedImage& CompressedImageMsg, const int& ImageHeight, const int& ImageWidth, const int& BitDepth);

	UFUNCTION(BlueprintCallable, Category = "Robofleet | Hololens Sensors")
	static void PublishPVImageMsg(const FString& TopicName, const FString& Namespace, const FImageROS& ImageMsg, UTextureRenderTarget2D* RenderTarget);

	UFUNCTION(BlueprintCallable, Category = "Robofleet | Hololens Sensors")
	static void PublishPVCompressedImageMsg(const FString& TopicName, const FString& Namespace, const FCompressedImage& ImageMsg, UTextureRenderTarget2D* RenderTarget);

	// Utility Functions
	UFUNCTION(BlueprintCallable, Category = "Robofleet|Image")
	static void ReadPixelData(UTextureRenderTarget2D* RenderTarget, TArray<FColor>& OutPixels);

	UFUNCTION(BlueprintCallable, Category = "Robofleet|Transform")
	static void GetRelativeTransform(const FTransform& target_frame, const FTransform& source_frame, FTransform& transform);

	UFUNCTION(BlueprintCallable, Category = "Robofleet|Transform")
	static void ConvertTransformToRobotConvention(const FTransform& unreal_transform, FTransform& robot_transform);

	UFUNCTION(BlueprintCallable, Category = "Robofleet|Image")
	static void GetCompressedByteArrayFromTextureRenderTarget(UTextureRenderTarget2D* RenderTarget, TArray<uint8>& OutCompressedImageData);

	static void ToCompressedJPEGImage(const void* InRawImageData, const uint32& InHeight, const uint32& InWidth, const uint32& InBitDepth, const ERGBFormat RawFormat, TArray<uint8>& OutCompressedImageData);

	static void GetByteArrayFromTextureRenderTarget(UTextureRenderTarget2D* RenderTarget, TArray<uint8>& OutCompressedImageData, uint32& OutHeight, uint32& OutWidth, uint32& OutBitDepth);

	static void GetRenderTargetFormat(UTextureRenderTarget2D* RenderTarget, FString& Type);

};