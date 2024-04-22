// Fill out your copyright notice in the Description page of Project Settings.


#include "RobofleetBPFunctionLibrary.h"
#include "Modules/ModuleManager.h"
#include "RobofleetUnrealClientModule.h"
#include "RobofleetClientBase.h"

# define M_PI 3.14159265358979323846  /* pi */

// New thing
//

void URobofleetBPFunctionLibrary::StartRobofleetSession(FString HostUrl, const UObject* WorldContextObject)
{
	FRobofleetUnrealClientModule::Get()->StartRobofleetSession(HostUrl, WorldContextObject);
}

void URobofleetBPFunctionLibrary::ResetAllAgentsSeen()
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		FRobofleetUnrealClientModule::Get()->RobofleetClient->ResetAllAgentsSeen();
	}

}

//augre_msgs

bool URobofleetBPFunctionLibrary::IsAgentPublishingStatusMsg(const FString& TfNamespace)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		return FRobofleetUnrealClientModule::Get()->RobofleetClient->IsAgentPublishingStatusMsg(TfNamespace);
	}
	return false;
}

FString URobofleetBPFunctionLibrary::GetUidFromAgentStatus(const FString& RobotName)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		return FRobofleetUnrealClientModule::Get()->RobofleetClient->GetUidFromAgentStatus(RobotName);
	}
	return TEXT("");
}

FString URobofleetBPFunctionLibrary::GetAgentCallsign(const FString& RobotName)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		return FRobofleetUnrealClientModule::Get()->RobofleetClient->GetAgentCallsign(RobotName);
	}
	return TEXT("");
}

FString URobofleetBPFunctionLibrary::GetAgentType(const FString& RobotName)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		return FRobofleetUnrealClientModule::Get()->RobofleetClient->GetAgentType(RobotName);
	}
	return TEXT("");
}

float URobofleetBPFunctionLibrary::GetRobotBatteryLevel(const FString& RobotName)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		return FRobofleetUnrealClientModule::Get()->RobofleetClient->GetRobotBatteryLevel(RobotName);
	}
	return 0.0f;
}

FString URobofleetBPFunctionLibrary::GetOwner(const FString& RobotName)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		return FRobofleetUnrealClientModule::Get()->RobofleetClient->GetOwner(RobotName);
	}
	return TEXT("");
}

FString URobofleetBPFunctionLibrary::GetControlStatus(const FString& RobotName)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		return FRobofleetUnrealClientModule::Get()->RobofleetClient->GetControlStatus(RobotName);
	}
	return TEXT("");
}

FString URobofleetBPFunctionLibrary::GetRobotLocationString(const FString& RobotName)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		return FRobofleetUnrealClientModule::Get()->RobofleetClient->GetRobotLocationString(RobotName);
	}
	return TEXT("");
}

FVector URobofleetBPFunctionLibrary::GetRobotPosition(const FString& RobotName)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		return FRobofleetUnrealClientModule::Get()->RobofleetClient->GetRobotPosition(RobotName);
	}
	return FVector(0, 0, 0);
}

void URobofleetBPFunctionLibrary::GetGoalPose(const FString& RobotName, FPoseStamped& PoseStampedMsg, bool& MsgReceived)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		PoseStamped pose_stamped;

		FRobofleetUnrealClientModule::Get()->RobofleetClient->GetGoalPose(RobotName, pose_stamped, MsgReceived);
		if (!MsgReceived) return;

		PoseStampedMsg.header.frame_id = FString(pose_stamped.header.frame_id.c_str());
		PoseStampedMsg.Transform.SetLocation(FVector(pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z));
		PoseStampedMsg.Transform.SetRotation(FQuat(pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y, pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w));
	}
}

FTransform URobofleetBPFunctionLibrary::GetFrameTransform(const FString& NodeName)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		return FRobofleetUnrealClientModule::Get()->RobofleetClient->GetFrameTransform(NodeName);
	}
	return FTransform();
}

FTransform URobofleetBPFunctionLibrary::GetFrameWorldTransform(const FString& NodeName)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		return FRobofleetUnrealClientModule::Get()->RobofleetClient->GetFrameWorldTransform(NodeName);
	}
	return FTransform();
}

TArray<FString> URobofleetBPFunctionLibrary::GetAllRobotsAtSite(const FString& Location)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		return FRobofleetUnrealClientModule::Get()->RobofleetClient->GetAllRobotsAtSite(Location);
	}
	return TArray<FString>();
}

TArray<FString> URobofleetBPFunctionLibrary::GetAllAgents()
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		return FRobofleetUnrealClientModule::Get()->RobofleetClient->GetAllAgents();
	}
	return TArray<FString>();
}

TArray<FString> URobofleetBPFunctionLibrary::GetAllFrames()
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		return FRobofleetUnrealClientModule::Get()->RobofleetClient->GetAllFrames();
	}
	return TArray<FString>();
}

bool URobofleetBPFunctionLibrary::isFrameAvailable(const FString& FrameName)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		return FRobofleetUnrealClientModule::Get()->RobofleetClient->isFrameAvailable(FrameName);
	}
	return false;
}

TArray<FString> URobofleetBPFunctionLibrary::GetChildrenFrameId(const FString& NodeName)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		return FRobofleetUnrealClientModule::Get()->RobofleetClient->GetChildrenFrameId(NodeName);
	}
	return TArray<FString>();
}

FTransform URobofleetBPFunctionLibrary::LookupTransform(const FString& target_frame, const FString& source_frame)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		return FRobofleetUnrealClientModule::Get()->RobofleetClient->LookupTransform(target_frame, source_frame);
	}
	return FTransform();
}

void URobofleetBPFunctionLibrary::GetRobotImage(const FString& RobotName, const FString& StreamName, TArray<uint8>& Image)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		return FRobofleetUnrealClientModule::Get()->RobofleetClient->GetRobotImage(RobotName, StreamName, Image);
	}
}

TSet<FString> URobofleetBPFunctionLibrary::GetImageStreams(const FString& RobotName)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		return FRobofleetUnrealClientModule::Get()->RobofleetClient->GetImageStreams(RobotName);
	}
	return TSet<FString>();
}

bool URobofleetBPFunctionLibrary::IsRobotImageCompressed(const FString& RobotName, const FString& StreamName)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		return FRobofleetUnrealClientModule::Get()->RobofleetClient->IsRobotImageCompressed(RobotName, StreamName);
	}
	return bool();
}

TArray<uint8> URobofleetBPFunctionLibrary::GetOccupancyGridImage(const FString& RobotName)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		return FRobofleetUnrealClientModule::Get()->RobofleetClient->GetOccupancyGridImage(RobotName);
	}
	return TArray<uint8>();
}

FMapMetaData URobofleetBPFunctionLibrary::GetOccupancyGridInfo(const FString& RobotName)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		return FRobofleetUnrealClientModule::Get()->RobofleetClient->GetOccupancyGridInfo(RobotName);
	}
	return FMapMetaData();
}


FTransform URobofleetBPFunctionLibrary::CreateGridToAnchorTransform(const FMapMetaData& OccupancyGridInfo,
	const FTransform& AnchorTransform)
{
	// meters (ROS) to centimeters (Unreal)
	int mToCmScalar = 100;

	// Z-offset
	int z_offset = -12;

	// get the grid size
	FVector OccupancyGridSize = FVector(OccupancyGridInfo.width, OccupancyGridInfo.height, 0);
	// resolution is also used and referenced directly as OccupancyGridInfo.resolution

	//get the vector from grid to map origin (0,0,0 pose in the map) and scale it from m to cm
	FVector originTranslation = FVector(OccupancyGridInfo.origin.GetLocation() * mToCmScalar);

	// since we flipped the incoming grid data from bottom to top, we need to adjust the origin vector
	// new y = -1 * grid_height - origin.y
	originTranslation.Y = (-OccupancyGridSize.Y * OccupancyGridInfo.resolution * mToCmScalar) - originTranslation.Y;

	// grid LL corner to center for grid to plane center translation is ( width/2 , height/2 )
	// convert from grid cells to cm
	FVector gridToPlaneTranslation = FVector(OccupancyGridSize * OccupancyGridInfo.resolution * mToCmScalar * 0.5);
	//make the plane lower than the anchor for iROS video
	gridToPlaneTranslation.Z = gridToPlaneTranslation.Z + z_offset;

	// get local transform into the anchor frame
	FVector rotatedVector = AnchorTransform.GetRotation().RotateVector(originTranslation + gridToPlaneTranslation);

	FTransform output;
	// add the rotated local vector to the anchor location
	output.SetLocation(AnchorTransform.GetLocation() + rotatedVector);
	// combine the local rotation and anchor rotation
	output.SetRotation(OccupancyGridInfo.origin.GetRotation() * AnchorTransform.GetRotation());

	//when applying this transform in blueprints, do rotation and then translation
	return output;

}


void URobofleetBPFunctionLibrary::PrintRobotsSeen()
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		FRobofleetUnrealClientModule::Get()->RobofleetClient->PrintRobotsSeen();
	}
}

void URobofleetBPFunctionLibrary::RegisterRobotSubscription(FString TopicName, FString RobotName)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		FRobofleetUnrealClientModule::Get()->RobofleetClient->RegisterRobotSubscription(TopicName, RobotName);
	}
}

URobofleetBase* URobofleetBPFunctionLibrary::GetClientReference()
{
	return FRobofleetUnrealClientModule::Get()->RobofleetClient;
}

FString URobofleetBPFunctionLibrary::ConvertAsaToFrameId(const FString& asa, const FString& tf_prefix)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		return FRobofleetUnrealClientModule::Get()->RobofleetClient->ConvertAsaToFrameId(asa, tf_prefix);
	}
	return TEXT("");
}

FString URobofleetBPFunctionLibrary::ConvertFrameIdToAsa(const FString& frame_id, const FString& tf_prefix)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		return FRobofleetUnrealClientModule::Get()->RobofleetClient->ConvertFrameIdToAsa(frame_id, tf_prefix);
	}
	return TEXT("");
}


FPose URobofleetBPFunctionLibrary::ConvertPoseToRightHandMeters(const FPose& pose_in) {
	FPose pose_out;

	if (!(FRobofleetUnrealClientModule::Get()->IsSessionRunning()))
	{
		return pose_out;
	}

	pose_out.position.x = pose_in.position.x / 100;
	pose_out.position.y = -pose_in.position.y / 100;
	pose_out.position.z = pose_in.position.z / 100;
	pose_out.orientation.X = -pose_in.orientation.X;
	pose_out.orientation.Y = -pose_in.orientation.Y;
	pose_out.orientation.Z = -pose_in.orientation.Z;
	pose_out.orientation.W = pose_in.orientation.W;

	return pose_out;
}

FTransform URobofleetBPFunctionLibrary::ConvertTransformToLeftHand(const FTransform ROSPose) {
	// Decompose the right-hand transform
	FVector RightLocation, RightScale;
	//FRotator RightRotation;
	FQuat RightRotation;

	RightLocation = ROSPose.GetTranslation();
	//RightRotation = ROSPose.Rotator();
	RightRotation = ROSPose.GetRotation();
	RightScale = ROSPose.GetScale3D();


	// Convert the location from right-hand to left-hand (negate the X-axis)
	//FVector LeftLocation(-RightLocation.X, RightLocation.Y, RightLocation.Z);
	FVector LeftLocation(RightLocation.Y, RightLocation.X, RightLocation.Z);

	// Convert the rotation from right-hand to left-hand (negate pitch and roll, swap yaw and pitch)
	//FRotator LeftRotation(-RightRotation.Pitch, RightRotation.Yaw, -RightRotation.Roll);
	FQuat LeftRotation = FQuat(-RightRotation.X, -RightRotation.Z, -RightRotation.Y, RightRotation.W);

	// Left-hand scale is the same as right-hand scale in most cases
	FVector LeftScale = RightScale;

	// Compose the left-hand transform
	//FTransform LeftHandTransform(LeftRotation.Quaternion(), LeftLocation, LeftScale);
	//FTransform LeftHandTransform(LeftRotation, LeftLocation, LeftScale);
	FTransform LeftHandTransform;
	LeftHandTransform.SetLocation(LeftLocation);
	LeftHandTransform.SetRotation(RightRotation);

	return LeftHandTransform;
}

FVector URobofleetBPFunctionLibrary::ConvertPositionToRightHandMeters(const FVector& pos_in) {
	FVector pos_out;

	if (!(FRobofleetUnrealClientModule::Get()->IsSessionRunning()))
	{
		return pos_out;
	}

	pos_out.X = pos_in.X / 100;
	pos_out.Y = -pos_in.Y / 100;
	pos_out.Z = pos_in.Z / 100;

	return pos_out;
}

FQuat URobofleetBPFunctionLibrary::ConvertQuaternionToRightHand(const FQuat& rot_in) {
	FQuat rot_out;

	if (!(FRobofleetUnrealClientModule::Get()->IsSessionRunning()))
	{
		return rot_out;
	}

	// Quaternion to RH untested
	rot_out.X = -rot_in.X;
	rot_out.Y = -rot_in.Z;
	rot_out.Z = -rot_in.Y;
	rot_out.W = rot_in.W;

	return rot_out;
}

FRotator URobofleetBPFunctionLibrary::ConvertEulerToRightHand(const FRotator& rot_in) {
	FRotator rot_out;

	if (!(FRobofleetUnrealClientModule::Get()->IsSessionRunning()))
	{
		return rot_out;
	}

	rot_out.Roll = -rot_in.Roll;
	rot_out.Pitch = rot_in.Pitch;
	rot_out.Yaw = -rot_in.Yaw;

	return rot_out;
}

FQuat URobofleetBPFunctionLibrary::ConvertEulerToRightHandQuaternion(const FRotator& rot_in) {
	FQuat rot_out;

	if (!(FRobofleetUnrealClientModule::Get()->IsSessionRunning()))
	{
		return rot_out;
	}

	rot_out.MakeFromEuler(FVector(rot_in.Roll, rot_in.Pitch, rot_in.Yaw));
	// Quaternion to RH untested
	rot_out.X = -rot_out.X;
	rot_out.Y = -rot_out.Z;
	rot_out.Z = -rot_out.Y;
	rot_out.W = rot_out.W;

	return rot_out;
}


FRotator URobofleetBPFunctionLibrary::ConvertQuaternionToRightHandEuler(const FQuat& rot_in) {
	FRotator rot_out;

	if (!(FRobofleetUnrealClientModule::Get()->IsSessionRunning()))
	{
		return rot_out;
	}

	rot_out.Roll = -rot_in.Euler().X;
	rot_out.Pitch = rot_in.Euler().Y;
	rot_out.Yaw = -rot_in.Euler().Z;

	return rot_out;
}

FVector URobofleetBPFunctionLibrary::ConvertDegToRad(const FVector& omega_vec) {

	return omega_vec * (M_PI / 180);
}

FVector URobofleetBPFunctionLibrary::ConvertAngleToRightHand(const FVector& omega_vec) {

	return FVector(-omega_vec.X, -omega_vec.Y, omega_vec.Z);
}

FString URobofleetBPFunctionLibrary::GetDetectedName(const FString& DetectedItemUid)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		return FRobofleetUnrealClientModule::Get()->RobofleetClient->GetDetectedName(DetectedItemUid);
	}
	return TEXT("");
}

FString URobofleetBPFunctionLibrary::GetDetectedType(const FString& DetectedItemUid)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		return FRobofleetUnrealClientModule::Get()->RobofleetClient->GetDetectedType(DetectedItemUid);
	}
	return TEXT("");
}

FString URobofleetBPFunctionLibrary::GetDetectedTypeLabel(const FString& DetectedItemUid)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		return FRobofleetUnrealClientModule::Get()->RobofleetClient->GetDetectedTypeLabel(DetectedItemUid);
	}
	return TEXT("");
}

FString URobofleetBPFunctionLibrary::GetDetectedHow(const FString& DetectedItemUid)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		return FRobofleetUnrealClientModule::Get()->RobofleetClient->GetDetectedHow(DetectedItemUid);
	}
	return TEXT("");
}

FString URobofleetBPFunctionLibrary::GetDetectedHowLabel(const FString& DetectedItemUid)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		return FRobofleetUnrealClientModule::Get()->RobofleetClient->GetDetectedHowLabel(DetectedItemUid);
	}
	return TEXT("");
}

FPoseStamped URobofleetBPFunctionLibrary::GetDetectedItemPose(const FString& DetectedItemUid)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		return FRobofleetUnrealClientModule::Get()->RobofleetClient->GetDetectedItemPose(DetectedItemUid);
	}
	return FPoseStamped();
}

FDateTime URobofleetBPFunctionLibrary::GetDetectedItemTimeStamped(const FString& DetectedItemUid)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		return FRobofleetUnrealClientModule::Get()->RobofleetClient->GetDetectedItemTimeStamped(DetectedItemUid);
	}
	return FDateTime();
}

TArray<uint8> URobofleetBPFunctionLibrary::GetDetectedImage(const FString& DetectedItemUid)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		return FRobofleetUnrealClientModule::Get()->RobofleetClient->GetDetectedImage(DetectedItemUid);
	}
	return TArray<uint8>();
}

FVector URobofleetBPFunctionLibrary::GetDetectedImageSize(const FString& DetectedItemUid)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		return FRobofleetUnrealClientModule::Get()->RobofleetClient->GetDetectedImageSize(DetectedItemUid);
	}
	return FVector(0, 0, 0);
	// FVector.X = height
	// FVector.Y = width
	// FVector.Z = Not used
}

FVector URobofleetBPFunctionLibrary::GetDetectedItemPosition(const FString& DetectedItemUid)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		return FRobofleetUnrealClientModule::Get()->RobofleetClient->GetDetectedItemPosition(DetectedItemUid);
	}
	return FVector(0, 0, 0);
}

FString URobofleetBPFunctionLibrary::GetDetectedItemImageURL(const FString& DetectedItemUid)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		return FRobofleetUnrealClientModule::Get()->RobofleetClient->GetDetectedItemImageURL(DetectedItemUid);
	}
	return TEXT("No Robofleet Connection");

}


TArray<FString> URobofleetBPFunctionLibrary::GetAllDetectedItems()
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		return FRobofleetUnrealClientModule::Get()->RobofleetClient->GetAllDetectedItems();
	}
	return TArray<FString>();
}

void URobofleetBPFunctionLibrary::RemoveDetectedItem(const FString& DetectedItemUid)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		FRobofleetUnrealClientModule::Get()->RobofleetClient->RemoveDetectedItem(DetectedItemUid);
	}
}

FVector URobofleetBPFunctionLibrary::GetScrewAxisPoint(const FString& RobotName)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		return FRobofleetUnrealClientModule::Get()->RobofleetClient->GetScrewAxisPoint(RobotName);
	}
	return FVector(0, 0, 0);
}

FVector URobofleetBPFunctionLibrary::GetScrewAxis(const FString& RobotName)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		return FRobofleetUnrealClientModule::Get()->RobofleetClient->GetScrewAxis(RobotName);
	}
	return FVector(0, 0, 0);
}

float URobofleetBPFunctionLibrary::GetScrewAxisPitch(const FString& RobotName)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		return FRobofleetUnrealClientModule::Get()->RobofleetClient->GetScrewAxisPitch(RobotName);
	}
	return float{ 0 };
}

void URobofleetBPFunctionLibrary::GetGestureResult(const FString& RobotName, FString& Result) {
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning()) {
		
		std::string gesture_result{ "" };
		FRobofleetUnrealClientModule::Get()->RobofleetClient->GetGestureResult(RobotName, gesture_result);
		Result = FString(gesture_result.c_str());
		UE_LOG(LogTemp, Warning, TEXT("Gesture Result in BP FUnction: %s"), *Result);
	}
}

void URobofleetBPFunctionLibrary::GetSpeechResult(const FString& RobotName, FString& Result) {
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning()) {

		std::string speech_result{ "" };
		FRobofleetUnrealClientModule::Get()->RobofleetClient->GetSpeechResult(RobotName, speech_result);
		Result = FString(speech_result.c_str());
	}
}

void URobofleetBPFunctionLibrary::GetFusedResult(const FString& RobotName, FString& Result) {
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning()) {

		std::string fused_result{ "" };
		FRobofleetUnrealClientModule::Get()->RobofleetClient->GetFusedResult(RobotName, fused_result);
		Result = FString(fused_result.c_str());
	}
}


void URobofleetBPFunctionLibrary::GetNonLegClusters(const FString& RobotName, FDetectionArray& NonLegClusterArray_)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		// Get Array
		DetectionArray NonLegClusterArray{};
		FRobofleetUnrealClientModule::Get()->RobofleetClient->GetNonLegClusters(RobotName, NonLegClusterArray);

		// Fill Header
		NonLegClusterArray_.header.frame_id = FString(NonLegClusterArray.header.frame_id.c_str());

		// Fill Vector
		FDetection temp{};
		for (auto& this_detection : NonLegClusterArray.detections)
		{
			temp.position.x = this_detection.position.x;
			temp.position.y = this_detection.position.y;
			temp.position.z = this_detection.position.z;
			temp.confidence = this_detection.confidence;
			temp.label = this_detection.label;
			NonLegClusterArray_.detections.Add(temp);
		}
	}
}

void URobofleetBPFunctionLibrary::GetDetectedLegClusters(const FString& RobotName, FDetectionArray& DetectedLegClusterArray_)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		// Get Array
		DetectionArray DetectedLegClusterArray{};
		FRobofleetUnrealClientModule::Get()->RobofleetClient->GetDetectedLegClusters(RobotName, DetectedLegClusterArray);

		// Fill Header
		DetectedLegClusterArray_.header.frame_id = FString(DetectedLegClusterArray.header.frame_id.c_str());

		// Fill Vector
		FDetection temp{};
		DetectedLegClusterArray_ = {};
		for (auto& this_detection : DetectedLegClusterArray.detections)
		{
			temp.position.x = this_detection.position.x;
			temp.position.y = this_detection.position.y;
			temp.position.z = this_detection.position.z;
			temp.confidence = this_detection.confidence;
			temp.label = this_detection.label;
			DetectedLegClusterArray_.detections.Add(temp);
		}
	}
}

void URobofleetBPFunctionLibrary::GetPeopleDetected(const FString& RobotName, FPersonArray& PeopleDetectedArray_)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		// Get Array
		PersonArray PeopleDetectedArray{};
		FRobofleetUnrealClientModule::Get()->RobofleetClient->GetPeopleDetected(RobotName, PeopleDetectedArray);

		// Fill Frame Id
		PeopleDetectedArray_.header.frame_id = FString(PeopleDetectedArray.header.frame_id.c_str());

		// Fill Vector
		FPerson temp{};
		for (auto& this_detection : PeopleDetectedArray.people)
		{
			temp.pose.position.x = this_detection.pose.position.x;
			temp.pose.position.y = this_detection.pose.position.y;
			temp.pose.position.z = this_detection.pose.position.z;
			temp.pose.orientation.X = this_detection.pose.orientation.x;
			temp.pose.orientation.Y = this_detection.pose.orientation.y;
			temp.pose.orientation.Z = this_detection.pose.orientation.z;
			temp.pose.orientation.W = this_detection.pose.orientation.w;
			temp.id = this_detection.id;
			PeopleDetectedArray_.people.Add(temp);
		}
	}
}

void URobofleetBPFunctionLibrary::GetPeopleTracked(const FString& RobotName, FPersonArray& PeopleTrackedArray_)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		PersonArray PeopleTrackedArray;
		FRobofleetUnrealClientModule::Get()->RobofleetClient->GetPeopleDetected(RobotName, PeopleTrackedArray);

		PeopleTrackedArray_.header.frame_id = FString(PeopleTrackedArray.header.frame_id.c_str());

		// Fill Detection Vector
		FPerson temp{};
		for (auto& this_detection : PeopleTrackedArray.people)
		{
			temp.pose.position.x = this_detection.pose.position.x;
			temp.pose.position.y = this_detection.pose.position.y;
			temp.pose.position.z = this_detection.pose.position.z;
			temp.pose.orientation.X = this_detection.pose.orientation.x;
			temp.pose.orientation.Y = this_detection.pose.orientation.y;
			temp.pose.orientation.Z = this_detection.pose.orientation.z;
			temp.pose.orientation.W = this_detection.pose.orientation.w;
			temp.id = this_detection.id;
			PeopleTrackedArray_.people.Add(temp);
		}
	}
}

// Publish Messages to Robofleet
// *TODO: These need to be rate limited somewhere in the client as there is the potential to overload the server
void URobofleetBPFunctionLibrary::PublishLocationMsg(const FString& RobotName, const FRobotLocationStamped& LocationMsg)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		RobotLocationStamped robot_location;
		robot_location.header.frame_id = std::string(TCHAR_TO_UTF8(*LocationMsg.header.frame_id));
		robot_location.header.stamp._nsec = LocationMsg.header.stamp._nsec;
		robot_location.header.stamp._sec = LocationMsg.header.stamp._sec;
		robot_location.header.seq = LocationMsg.header.seq;
		robot_location.x = LocationMsg.x;
		robot_location.y = LocationMsg.y;
		robot_location.z = LocationMsg.z;
		robot_location.theta = LocationMsg.theta;

		FRobofleetUnrealClientModule::Get()->RobofleetClient->PublishLocationMsg(RobotName, robot_location);
	}
}

void URobofleetBPFunctionLibrary::PublishTransformWithCovarianceStampedMsg(const FString& TopicName, const FTransformWithCovarianceStamped& FTfWithCovarianceStampedmsg)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		TransformWithCovarianceStamped TFwithCovStamped;

		std::string asa_header = std::string(TCHAR_TO_UTF8(*FTfWithCovarianceStampedmsg.transform.header.frame_id));
		std::replace(asa_header.begin(), asa_header.end(), '-', '_');

		TFwithCovStamped.transform.header.frame_id = asa_header;
		TFwithCovStamped.transform.header.stamp._nsec = FDateTime::Now().GetMillisecond() * 1000000;
		//TFwithCovStamped.transform.header.stamp._nsec = FTfWithCovarianceStampedmsg.transform.header.stamp._nsec;
		TFwithCovStamped.transform.header.stamp._sec = FDateTime::Now().ToUnixTimestamp();
		//TFwithCovStamped.transform.header.stamp._sec = FTfWithCovarianceStampedmsg.transform.header.stamp._sec;
		TFwithCovStamped.transform.header.seq = FTfWithCovarianceStampedmsg.transform.header.seq;

		std::string child_frame_id = std::string(TCHAR_TO_UTF8(*FTfWithCovarianceStampedmsg.transform.child_frame_id));
		std::replace(child_frame_id.begin(), child_frame_id.end(), '-', '_');

		TFwithCovStamped.transform.child_frame_id = child_frame_id;
		TFwithCovStamped.transform.transform.translation.x = FTfWithCovarianceStampedmsg.transform.Transform.GetTranslation().X;
		TFwithCovStamped.transform.transform.translation.y = FTfWithCovarianceStampedmsg.transform.Transform.GetTranslation().Y;
		TFwithCovStamped.transform.transform.translation.z = FTfWithCovarianceStampedmsg.transform.Transform.GetTranslation().Z;
		TFwithCovStamped.transform.transform.rotation.x = FTfWithCovarianceStampedmsg.transform.Transform.GetRotation().X;
		TFwithCovStamped.transform.transform.rotation.y = FTfWithCovarianceStampedmsg.transform.Transform.GetRotation().Y;
		TFwithCovStamped.transform.transform.rotation.z = FTfWithCovarianceStampedmsg.transform.Transform.GetRotation().Z;
		TFwithCovStamped.transform.transform.rotation.w = FTfWithCovarianceStampedmsg.transform.Transform.GetRotation().W;

		//Convert from TArray to std::vector
		/*std::vector<float> stdev = { 0.1, 0.1, 0.1, 0.1, 0.1, 0.1 };

		float varXX = 0;
		float varYY = 0;
		float varZZ = 0;
		float varRR = 0;
		float varPP = 0;
		float varYwYw = 0;
		std::vector<float> fake_cov = { varXX,	0,		0,		0,		0,		0,
										0,		varYY,	0,		0,		0,		0,
										0,		0,		varZZ,  0,		0,		0,
										0,		0,		0,		varRR,	0,		0,
										0,		0,		0,		0,		varPP,	0,
										0,		0,		0,		0,		0,		varYwYw };*/
		for (auto& cov : FTfWithCovarianceStampedmsg.covariance)
		{
			TFwithCovStamped.covariance.push_back(cov);
		}

		/*for (auto& cov : fake_cov)
		{
			TFwithCovStamped.covariance.push_back(cov);
		}*/

		FRobofleetUnrealClientModule::Get()->RobofleetClient->PublishTransformWithCovarianceStampedMsg(TopicName, TFwithCovStamped);
	}
}

void URobofleetBPFunctionLibrary::PublishBoundingObject3DArrayMsg(const FString& TopicName, const FString& Namespace, const FBoundingObject3DArray& Msg)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		BoundingObject3DArray dst_array;

		for (FBoundingObject3D src : Msg.objects)
		{
			BoundingObject3D dst;
			// Check for out-of-range values to prevent data loss
			if (src.shape < 0) {
				dst.shape = 0; // If the Unreal Engine Integer is negative, return 0 for uint8_t
			}
			else if (src.shape > 255) {
				dst.shape = 255; // If the Unreal Engine Integer is greater than 255, return 255 for uint8_t
			}
			else {
				dst.shape = static_cast<uint8_t>(src.shape); // Perform the cast for valid values
			}

			// Check for out-of-range values to prevent data loss
			if (src.action < 0) {
				dst.action = 0; // If the Unreal Engine Integer is negative, return 0 for uint8_t
			}
			else if (src.action > 255) {
				dst.action = 255; // If the Unreal Engine Integer is greater than 255, return 255 for uint8_t
			}
			else {
				dst.action = static_cast<uint8_t>(src.action); // Perform the cast for valid values
			}

			dst.uid = std::string(TCHAR_TO_UTF8(*src.uid));
			dst.size_x = src.size_x;
			dst.size_y = src.size_y;
			dst.size_z = src.size_z;
			dst.radius = src.radius;
			dst.centroid.header.frame_id = std::string(TCHAR_TO_UTF8(*src.centroid.header.frame_id));
			dst.centroid.header.stamp._nsec = FDateTime::Now().GetMillisecond() * 1000000;
			dst.centroid.header.stamp._sec = FDateTime::Now().ToUnixTimestamp();
			dst.centroid.header.seq = src.centroid.header.seq;
			dst.centroid.pose.position.x = src.centroid.Transform.GetLocation().X;
			dst.centroid.pose.position.y = src.centroid.Transform.GetLocation().Y;
			dst.centroid.pose.position.z = src.centroid.Transform.GetLocation().Z;
			dst.centroid.pose.orientation.x = src.centroid.Transform.GetRotation().X;
			dst.centroid.pose.orientation.y = src.centroid.Transform.GetRotation().Y;
			dst.centroid.pose.orientation.z = src.centroid.Transform.GetRotation().Z;
			dst.centroid.pose.orientation.w = src.centroid.Transform.GetRotation().W;

			dst_array.objects.push_back(dst);
		}

		FRobofleetUnrealClientModule::Get()->RobofleetClient->PublishBoundingObject3DArrayMsg(std::string(TCHAR_TO_UTF8(*TopicName)), std::string(TCHAR_TO_UTF8(*Namespace)), dst_array);
	}
}


void URobofleetBPFunctionLibrary::PublishAzureSpatialAnchorMsg(const FString& RobotName, const FAzureSpatialAnchor& FAzureSpatialAnchorMsg)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		AzureSpatialAnchor AzureSpatialAnchor;
		AzureSpatialAnchor.asa_id = std::string(TCHAR_TO_UTF8(*FAzureSpatialAnchorMsg.asa_id));
		AzureSpatialAnchor.rep_id = std::string(TCHAR_TO_UTF8(*FAzureSpatialAnchorMsg.rep_id));
		AzureSpatialAnchor.ns = std::string(TCHAR_TO_UTF8(*FAzureSpatialAnchorMsg.ns));
		AzureSpatialAnchor.anchor_type = std::string(TCHAR_TO_UTF8(*FAzureSpatialAnchorMsg.anchor_type));
		AzureSpatialAnchor.timestamp._nsec = FDateTime::Now().GetMillisecond() * 1000000;
		AzureSpatialAnchor.timestamp._sec = FDateTime::Now().ToUnixTimestamp();
		//AzureSpatialAnchor.timestamp._sec = FAzureSpatialAnchorMsg.timestamp._sec;

		// PoseWithCovarianceStamped
		AzureSpatialAnchor.pose.header.frame_id = std::string(TCHAR_TO_UTF8(*FAzureSpatialAnchorMsg.pose.header.frame_id));
		AzureSpatialAnchor.pose.header.stamp._nsec = FDateTime::Now().GetMillisecond() * 1000000;
		AzureSpatialAnchor.pose.header.stamp._sec = FDateTime::Now().ToUnixTimestamp();
		//AzureSpatialAnchor.pose.header.stamp._sec = FAzureSpatialAnchorMsg.pose.header.stamp._sec;
		AzureSpatialAnchor.pose.header.seq = FAzureSpatialAnchorMsg.pose.header.seq;
		AzureSpatialAnchor.pose.pose.pose.position.x = FAzureSpatialAnchorMsg.pose.pose.pose.GetTranslation().X;
		AzureSpatialAnchor.pose.pose.pose.position.y = FAzureSpatialAnchorMsg.pose.pose.pose.GetTranslation().Y;
		AzureSpatialAnchor.pose.pose.pose.position.z = FAzureSpatialAnchorMsg.pose.pose.pose.GetTranslation().Z;
		AzureSpatialAnchor.pose.pose.pose.orientation.x = FAzureSpatialAnchorMsg.pose.pose.pose.GetRotation().X;
		AzureSpatialAnchor.pose.pose.pose.orientation.y = FAzureSpatialAnchorMsg.pose.pose.pose.GetRotation().Y;
		AzureSpatialAnchor.pose.pose.pose.orientation.z = FAzureSpatialAnchorMsg.pose.pose.pose.GetRotation().Z;
		AzureSpatialAnchor.pose.pose.pose.orientation.w = FAzureSpatialAnchorMsg.pose.pose.pose.GetRotation().W;

		// TODO: GET ANCHOR COVARIANCES

		for (auto& cov : FAzureSpatialAnchorMsg.pose.pose.covariance)
		{
			AzureSpatialAnchor.pose.pose.covariance.push_back(cov);
		}

		// GeoPoseWithCovarianceStamped
		AzureSpatialAnchor.geopose.header.frame_id = std::string(TCHAR_TO_UTF8(*FAzureSpatialAnchorMsg.geopose.header.frame_id));
		AzureSpatialAnchor.geopose.header.stamp._nsec = FDateTime::Now().GetMillisecond() * 1000000;
		AzureSpatialAnchor.geopose.header.stamp._sec = FDateTime::Now().ToUnixTimestamp();
		//AzureSpatialAnchor.geopose.header.stamp._sec = FAzureSpatialAnchorMsg.geopose.header.stamp._sec;
		AzureSpatialAnchor.geopose.header.seq = FAzureSpatialAnchorMsg.geopose.header.seq;
		AzureSpatialAnchor.geopose.pose.pose.position.latitude = FAzureSpatialAnchorMsg.geopose.pose.pose.GetTranslation().X;
		AzureSpatialAnchor.geopose.pose.pose.position.longitude = FAzureSpatialAnchorMsg.geopose.pose.pose.GetTranslation().Y;
		AzureSpatialAnchor.geopose.pose.pose.position.altitude = FAzureSpatialAnchorMsg.geopose.pose.pose.GetTranslation().Z;
		AzureSpatialAnchor.geopose.pose.pose.orientation.x = FAzureSpatialAnchorMsg.geopose.pose.pose.GetRotation().X;
		AzureSpatialAnchor.geopose.pose.pose.orientation.y = FAzureSpatialAnchorMsg.geopose.pose.pose.GetRotation().Y;
		AzureSpatialAnchor.geopose.pose.pose.orientation.z = FAzureSpatialAnchorMsg.geopose.pose.pose.GetRotation().Z;
		AzureSpatialAnchor.geopose.pose.pose.orientation.w = FAzureSpatialAnchorMsg.geopose.pose.pose.GetRotation().W;

		// TODO: GET GEOPOSE ANCHOR COVARIANCES
		//Convert from TArray to std::vector
		std::vector<float> geopose_anchor_cov = { 71.1655073735117,	3.45821576403310,	0.593090091521150,	1.30137103899806,	0.664337209474972,	15.3887697684447,
										3.45821576403310,	2.17138180031235,	0.0889157579901040,	0.356811074478031, -1.54724979011027,	1.08088777296042,
										0.593090091521150,	0.0889157579901040,	0.312597527415474, -0.0669388300717367,	1.34318992031335,	0.200723741020181,
										1.30137103899806,	0.356811074478031, -0.0669388300717367,	0.770845292528924, -0.520578503254728,	0.245532369990909,
										0.664337209474972, -1.54724979011027,	1.34318992031335, -0.520578503254728,	9.36524337618332,	0.0369266731945052,
										15.3887697684447,	1.08088777296042,	0.200723741020181,	0.245532369990909,	0.0369266731945052,	4.16394130032915 };
		for (auto& cov : geopose_anchor_cov)
		{
			AzureSpatialAnchor.geopose.pose.covariance.push_back(cov);
		}

		// BLANK FOR ANOTHER DAY
		AzureSpatialAnchor.neighbors.push_back("");

		FRobofleetUnrealClientModule::Get()->RobofleetClient->PublishAzureSpatialAnchorMsg(RobotName, AzureSpatialAnchor);
	}
}


void URobofleetBPFunctionLibrary::PublishAgentStatusMsg(const FString& RobotName, const FAgentStatus& StatusMsg)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		AgentStatus agent_status;
		agent_status.uid = std::string(TCHAR_TO_UTF8(*StatusMsg.uid));
		agent_status.callsign = std::string(TCHAR_TO_UTF8(*StatusMsg.callsign));
		agent_status.agent_type = std::string(TCHAR_TO_UTF8(*StatusMsg.agent_type));
		agent_status.battery = StatusMsg.battery;
		agent_status.commander = std::string(TCHAR_TO_UTF8(*StatusMsg.commander));
		agent_status.control_status = std::string(TCHAR_TO_UTF8(*StatusMsg.control_status));

		FRobofleetUnrealClientModule::Get()->RobofleetClient->PublishAgentStatusMsg(RobotName, agent_status);
	}
}


// Odom Study
void URobofleetBPFunctionLibrary::PublishHololensOdom(const FString& RobotName, const FPoseStamped& PoseStampedMsg)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		PoseStamped Goal;
		Goal.header.frame_id = std::string(TCHAR_TO_UTF8(*PoseStampedMsg.header.frame_id));
		Goal.header.stamp._nsec = PoseStampedMsg.header.stamp._nsec;
		Goal.header.stamp._sec = PoseStampedMsg.header.stamp._sec;
		Goal.header.seq = PoseStampedMsg.header.seq;

		Goal.pose.position.x = PoseStampedMsg.Transform.GetLocation().X;
		Goal.pose.position.y = PoseStampedMsg.Transform.GetLocation().Y;
		Goal.pose.position.z = PoseStampedMsg.Transform.GetLocation().Z;
		Goal.pose.orientation.x = PoseStampedMsg.Transform.GetRotation().Euler().X;
		Goal.pose.orientation.y = PoseStampedMsg.Transform.GetRotation().Euler().Y;
		Goal.pose.orientation.z = PoseStampedMsg.Transform.GetRotation().Euler().Z;
		Goal.pose.orientation.w = 0; // PoseStampedMsg.Transform.GetRotation().W;

		FRobofleetUnrealClientModule::Get()->RobofleetClient->PublishHololensOdom(RobotName, Goal);
	}
}

void URobofleetBPFunctionLibrary::PublishPoseStamped(const FString& RobotUid, const FString& TopicName, const FPoseStamped& PoseStampedMsg)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		PoseStamped Goal;
		Goal.header.frame_id = std::string(TCHAR_TO_UTF8(*PoseStampedMsg.header.frame_id));
		Goal.header.stamp._nsec = FDateTime::Now().GetMillisecond() * 1000000;
		Goal.header.stamp._sec = FDateTime::Now().ToUnixTimestamp();
		Goal.header.seq = PoseStampedMsg.header.seq;

		Goal.pose.position.x = PoseStampedMsg.Transform.GetLocation().X;
		Goal.pose.position.y = PoseStampedMsg.Transform.GetLocation().Y;
		Goal.pose.position.z = PoseStampedMsg.Transform.GetLocation().Z;
		Goal.pose.orientation.x = PoseStampedMsg.Transform.GetRotation().X;
		Goal.pose.orientation.y = PoseStampedMsg.Transform.GetRotation().Y;
		Goal.pose.orientation.z = PoseStampedMsg.Transform.GetRotation().Z;
		Goal.pose.orientation.w = PoseStampedMsg.Transform.GetRotation().W;

		FRobofleetUnrealClientModule::Get()->RobofleetClient->PublishPoseStamped(RobotUid, TopicName, Goal);
	}
}

void URobofleetBPFunctionLibrary::PublishCancel(const FString& RobotUid, const FString& TopicName) {
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		FRobofleetUnrealClientModule::Get()->RobofleetClient->PublishCancel(RobotUid, TopicName);
	}
}

void URobofleetBPFunctionLibrary::PublishEmptyMsg(const FString& TopicName, const FString& Namespace) {
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		FRobofleetUnrealClientModule::Get()->RobofleetClient->PublishEmptyMsg(TopicName, Namespace);
	}
}

void URobofleetBPFunctionLibrary::PublishStringMsg(const FString& TopicName, const FString& Namespace, const FString& StringMessage) {

	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		FRobofleetUnrealClientModule::Get()->RobofleetClient->PublishStringMsg(std::string(TCHAR_TO_UTF8(*TopicName)),
			std::string(TCHAR_TO_UTF8(*Namespace)),
			std::string(TCHAR_TO_UTF8(*StringMessage)));
	}
}

void  URobofleetBPFunctionLibrary::PublishUInt8Msg(const FString& TopicName, const FString& Namespace, const int& UInt8Message) {

	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		// Check for out-of-range values to prevent data loss
		uint8_t uint8_t_msg;
		if (UInt8Message < 0) {
			uint8_t_msg = 0; // If the Unreal Engine Integer is negative, return 0 for uint8_t
		}
		else if (UInt8Message > 255) {
			uint8_t_msg = 255; // If the Unreal Engine Integer is greater than 255, return 255 for uint8_t
		}
		else {
			uint8_t_msg = static_cast<uint8_t>(UInt8Message); // Perform the cast for valid values
		}

		FRobofleetUnrealClientModule::Get()->RobofleetClient->PublishUInt8Msg(std::string(TCHAR_TO_UTF8(*TopicName)),
			std::string(TCHAR_TO_UTF8(*Namespace)),
			uint8_t_msg);
	}
}

void URobofleetBPFunctionLibrary::PublishBoolMsg(const FString& TopicName, const FString& Namespace, const bool& cmd) {
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		FRobofleetUnrealClientModule::Get()->RobofleetClient->PublishBoolMsg(TopicName, Namespace, cmd);
	}
}

void URobofleetBPFunctionLibrary::PublishStringCommand(const FString& cmd) {
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		FRobofleetUnrealClientModule::Get()->RobofleetClient->PublishStringCommand(cmd);
	}
}

void URobofleetBPFunctionLibrary::PublishMoveBaseSimpleGoal(const FString& RobotName, const FPoseStamped& PoseStampedMsg)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		PoseStamped Goal;
		Goal.header.frame_id = std::string(TCHAR_TO_UTF8(*PoseStampedMsg.header.frame_id));
		Goal.header.stamp._nsec = PoseStampedMsg.header.stamp._nsec;
		Goal.header.stamp._sec = PoseStampedMsg.header.stamp._sec;
		Goal.header.seq = PoseStampedMsg.header.seq;

		Goal.pose.position.x = PoseStampedMsg.Transform.GetLocation().X;
		Goal.pose.position.y = PoseStampedMsg.Transform.GetLocation().Y;
		Goal.pose.position.z = PoseStampedMsg.Transform.GetLocation().Z;

		Goal.pose.orientation.x = PoseStampedMsg.Transform.GetRotation().X;
		Goal.pose.orientation.y = PoseStampedMsg.Transform.GetRotation().Y;
		Goal.pose.orientation.z = PoseStampedMsg.Transform.GetRotation().Z;
		Goal.pose.orientation.w = PoseStampedMsg.Transform.GetRotation().W;

		FRobofleetUnrealClientModule::Get()->RobofleetClient->PublishMoveBaseSimpleGoal(RobotName, Goal);
	}
}

void URobofleetBPFunctionLibrary::PublishHandPose(const FString& RobotName, const FPoseStamped& PoseStampedMsg, FDateTime CurTimeStamp)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		PoseStamped Goal;
		Goal.header.frame_id = std::string(TCHAR_TO_UTF8(*PoseStampedMsg.header.frame_id));
		Goal.header.stamp._nsec = PoseStampedMsg.header.stamp._nsec;
		Goal.header.stamp._sec = (int)CurTimeStamp.ToUnixTimestamp();
		Goal.header.seq = PoseStampedMsg.header.seq;

		Goal.pose.position.x = PoseStampedMsg.Transform.GetLocation().X;
		Goal.pose.position.y = PoseStampedMsg.Transform.GetLocation().Y;
		Goal.pose.position.z = PoseStampedMsg.Transform.GetLocation().Z;

		Goal.pose.orientation.x = PoseStampedMsg.Transform.GetRotation().X;
		Goal.pose.orientation.y = PoseStampedMsg.Transform.GetRotation().Y;
		Goal.pose.orientation.z = PoseStampedMsg.Transform.GetRotation().Z;
		Goal.pose.orientation.w = PoseStampedMsg.Transform.GetRotation().W;

		FRobofleetUnrealClientModule::Get()->RobofleetClient->PublishHandPose(RobotName, Goal);
	}
}

void URobofleetBPFunctionLibrary::PublishNavigationPath(const FString& RobotName, const FPath& PathMsg)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		Path navigation_path;

		std::string asa_header = std::string(TCHAR_TO_UTF8(*PathMsg.header.frame_id));
		std::replace(asa_header.begin(), asa_header.end(), '-', '_');

		navigation_path.header.frame_id = asa_header;
		navigation_path.header.stamp._nsec = PathMsg.header.stamp._nsec;
		navigation_path.header.stamp._sec = PathMsg.header.stamp._sec;
		navigation_path.header.seq = PathMsg.header.seq;

		for (auto& poses : PathMsg.poses)
		{
			PoseStamped poseTemp;

			std::string asa_poses = std::string(TCHAR_TO_UTF8(*poses.header.frame_id));
			std::replace(asa_poses.begin(), asa_poses.end(), '-', '_');
			poseTemp.header.frame_id = asa_poses;
			poseTemp.header.stamp._nsec = poses.header.stamp._nsec;
			poseTemp.header.stamp._sec = poses.header.stamp._sec;
			poseTemp.header.seq = poses.header.seq;
			poseTemp.pose.position.x = poses.Transform.GetTranslation().X / 100;
			poseTemp.pose.position.y = -poses.Transform.GetTranslation().Y / 100;
			poseTemp.pose.position.z = poses.Transform.GetTranslation().Z / 100;
			poseTemp.pose.orientation.x = poses.Transform.GetRotation().X;
			poseTemp.pose.orientation.y = poses.Transform.GetRotation().Y;
			poseTemp.pose.orientation.z = poses.Transform.GetRotation().Z;
			poseTemp.pose.orientation.w = poses.Transform.GetRotation().W;
			navigation_path.poses.push_back(poseTemp);
		}
		FRobofleetUnrealClientModule::Get()->RobofleetClient->PublishNavigationPath(RobotName, navigation_path);
	}
}

void URobofleetBPFunctionLibrary::PublishTwistMsg(const FString& RobotName, const FString& TopicName, const FTwist& TwistMsg)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		Twist cmd_vel;
		cmd_vel.linear.x = TwistMsg.linear.X;
		cmd_vel.linear.y = -TwistMsg.linear.Y;
		cmd_vel.linear.z = TwistMsg.linear.Z;
		cmd_vel.angular.x = TwistMsg.angular.X;
		cmd_vel.angular.y = TwistMsg.angular.Y;
		cmd_vel.angular.z = TwistMsg.angular.Z;

		FRobofleetUnrealClientModule::Get()->RobofleetClient->PublishTwistMsg(RobotName, TopicName, cmd_vel);
	}
}

void URobofleetBPFunctionLibrary::PublishTwistStampedMsg(const FString& RobotName, const FString& TopicName, const FTwistStamped& TwistStampedMsg)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		TwistStamped twistStpd;

		twistStpd.header.frame_id = std::string(TCHAR_TO_UTF8(*TwistStampedMsg.header.frame_id));
		twistStpd.header.stamp._nsec = TwistStampedMsg.header.stamp._nsec;
		twistStpd.header.stamp._sec = TwistStampedMsg.header.stamp._sec;
		twistStpd.header.seq = TwistStampedMsg.header.seq;
		twistStpd.twist.linear.x = TwistStampedMsg.twist.linear.X;
		twistStpd.twist.linear.y = -TwistStampedMsg.twist.linear.Y;
		twistStpd.twist.linear.z = TwistStampedMsg.twist.linear.Z;
		twistStpd.twist.angular.x = TwistStampedMsg.twist.angular.X;
		twistStpd.twist.angular.y = TwistStampedMsg.twist.angular.Y;
		twistStpd.twist.angular.z = TwistStampedMsg.twist.angular.Z;

		FRobofleetUnrealClientModule::Get()->RobofleetClient->PublishTwistStampedMsg(RobotName, TopicName, twistStpd);
	}
}

/*
void URobofleetBPFunctionLibrary::PublishTFMessageMsg(const FTFMessage& TFMessageMsg)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		TFMessage tf;

		for (auto& transforms : TFMessageMsg.transforms)
		{
			TransformStamped tf_stamped;
			tf_stamped.header.frame_id = std::string(TCHAR_TO_UTF8(*transforms.header.frame_id));
			tf_stamped.header.stamp._nsec = FDateTime::UtcNow().GetMillisecond() * 1000000;
			tf_stamped.header.stamp._sec = FDateTime::UtcNow().ToUnixTimestamp();
			tf_stamped.header.seq = transforms.header.seq;
			tf_stamped.child_frame_id = std::string(TCHAR_TO_UTF8(*transforms.child_frame_id));
			tf_stamped.transform.translation.x = transforms.Transform.GetTranslation().X;
			tf_stamped.transform.translation.y = transforms.Transform.GetTranslation().Y;
			tf_stamped.transform.translation.z = transforms.Transform.GetTranslation().Z;
			tf_stamped.transform.rotation.x = transforms.Transform.GetRotation().X;
			tf_stamped.transform.rotation.y = transforms.Transform.GetRotation().Y;
			tf_stamped.transform.rotation.z = transforms.Transform.GetRotation().Z;
			tf_stamped.transform.rotation.w = transforms.Transform.GetRotation().W;

			tf.transforms.push_back(tf_stamped);
		}
		FRobofleetUnrealClientModule::Get()->RobofleetClient->PublishTFMessage(tf);
	}
}*/

// making the above more efficient^
void URobofleetBPFunctionLibrary::PublishTFMessageMsg(const FTFMessage& Msg)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		TFMessage tf;
		const size_t tf_size = Msg.transforms.Num();
		tf.transforms.resize(tf_size);

		const FDateTime now = FDateTime::UtcNow();
		const uint32_t sec = now.ToUnixTimestamp();
		const uint32_t nsec = now.GetMillisecond() * 1000000;

		for (size_t i{ 0 }; i < tf_size; i++)
		{	
			TransformStamped tf_stamped;
			tf_stamped.header.frame_id = std::string(TCHAR_TO_UTF8(*Msg.transforms[i].header.frame_id));
			tf_stamped.header.stamp._nsec = nsec;
			tf_stamped.header.stamp._sec = sec;
			tf_stamped.header.seq = Msg.transforms[i].header.seq;
			tf_stamped.child_frame_id = std::string(TCHAR_TO_UTF8(*Msg.transforms[i].child_frame_id));
			tf_stamped.transform.translation.x = Msg.transforms[i].Transform.GetTranslation().X;
			tf_stamped.transform.translation.y = Msg.transforms[i].Transform.GetTranslation().Y;
			tf_stamped.transform.translation.z = Msg.transforms[i].Transform.GetTranslation().Z;
			tf_stamped.transform.rotation.x = Msg.transforms[i].Transform.GetRotation().X;
			tf_stamped.transform.rotation.y = Msg.transforms[i].Transform.GetRotation().Y;
			tf_stamped.transform.rotation.z = Msg.transforms[i].Transform.GetRotation().Z;
			tf_stamped.transform.rotation.w = Msg.transforms[i].Transform.GetRotation().W;

			tf.transforms[i] = std::move(tf_stamped);
		}
		FRobofleetUnrealClientModule::Get()->RobofleetClient->PublishTFMessage(tf);
	}
}

// this is only used in AR-Affordance work. This should be deleted and that code should call PublishTFMessageMsg
void URobofleetBPFunctionLibrary::PublishTFMsg(const FString& TopicName, const FString& Namespace, const FTFMessage& Msg)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		TFMessage tf;
		TransformStamped tf_stamped;
		size_t tf_size = (size_t)Msg.transforms.Num();
		tf.transforms.reserve(tf_size);

		//for (auto& transforms : TFMessageMsg.transforms)
		for (size_t i{ 0 }; i < tf_size; i++)
		{
			tf_stamped.header.frame_id = std::string(TCHAR_TO_UTF8(*Msg.transforms[i].header.frame_id));
			tf_stamped.header.stamp._nsec = FDateTime::UtcNow().GetMillisecond() * 1000000;
			tf_stamped.header.stamp._sec = FDateTime::UtcNow().ToUnixTimestamp();
			tf_stamped.header.seq = Msg.transforms[i].header.seq;
			tf_stamped.child_frame_id = std::string(TCHAR_TO_UTF8(*Msg.transforms[i].child_frame_id));
			tf_stamped.transform.translation.x = Msg.transforms[i].Transform.GetTranslation().X;
			tf_stamped.transform.translation.y = Msg.transforms[i].Transform.GetTranslation().Y;
			tf_stamped.transform.translation.z = Msg.transforms[i].Transform.GetTranslation().Z;
			tf_stamped.transform.rotation.x = Msg.transforms[i].Transform.GetRotation().X;
			tf_stamped.transform.rotation.y = Msg.transforms[i].Transform.GetRotation().Y;
			tf_stamped.transform.rotation.z = Msg.transforms[i].Transform.GetRotation().Z;
			tf_stamped.transform.rotation.w = Msg.transforms[i].Transform.GetRotation().W;
			tf.transforms[i] = tf_stamped;
		}

		FRobofleetUnrealClientModule::Get()->RobofleetClient->PublishTFMsg(TopicName, Namespace, tf);
	}
}

void URobofleetBPFunctionLibrary::PublishDetection(const FDetectedItem& detection)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		DetectedItem_augre detection_augre;

		detection_augre.uid = std::string(TCHAR_TO_UTF8(*detection.uid));
		detection_augre.callsign = std::string(TCHAR_TO_UTF8(*detection.callsign));
		detection_augre.type = std::string(TCHAR_TO_UTF8(*detection.type));
		detection_augre.type_label = std::string(TCHAR_TO_UTF8(*detection.type_label));
		detection_augre.how = std::string(TCHAR_TO_UTF8(*detection.how));
		detection_augre.how_label = std::string(TCHAR_TO_UTF8(*detection.how_label));

		detection_augre.pose.header.frame_id = std::string(TCHAR_TO_UTF8(*detection.pose.header.frame_id));
		detection_augre.pose.header.seq = detection.pose.header.seq;
		detection_augre.pose.header.stamp._nsec = FDateTime::UtcNow().GetMillisecond() * 1000000;
		detection_augre.pose.header.stamp._sec = FDateTime::UtcNow().ToUnixTimestamp();

		detection_augre.pose.pose.position.x = detection.pose.Transform.GetTranslation().X;
		detection_augre.pose.pose.position.y = detection.pose.Transform.GetTranslation().Y;
		detection_augre.pose.pose.position.z = detection.pose.Transform.GetTranslation().Z;
		detection_augre.pose.pose.orientation.x = detection.pose.Transform.GetRotation().X;
		detection_augre.pose.pose.orientation.y = detection.pose.Transform.GetRotation().Y;
		detection_augre.pose.pose.orientation.z = detection.pose.Transform.GetRotation().Z;
		detection_augre.pose.pose.orientation.w = detection.pose.Transform.GetRotation().W;

		detection_augre.cmpr_image.header.frame_id = std::string(TCHAR_TO_UTF8(*detection.cmpr_image.header.frame_id));
		detection_augre.cmpr_image.header.seq = detection.cmpr_image.header.seq;
		detection_augre.cmpr_image.header.stamp._nsec = FDateTime::UtcNow().GetMillisecond() * 1000000;
		detection_augre.cmpr_image.header.stamp._sec = FDateTime::UtcNow().ToUnixTimestamp();
		detection_augre.cmpr_image.format = std::string(TCHAR_TO_UTF8(*detection.cmpr_image.format));

		for (auto& data : detection.cmpr_image.data)
		{
			detection_augre.cmpr_image.data.push_back(data);
		}

		//// Get UTexture2D into a TArray
		//TArray<uint8> out_compressed_image;
		//GetCompressedImageData(detection.cmpr_image.data, out_compressed_image);
		//
		//// reserve space in image vector to avoid reallocations
		//detection_augre.cmpr_image.data.reserve(out_compressed_image.Num());
		//
		//// copy the contents of the TArray to the std::vector
		//detection_augre.cmpr_image.data.assign(out_compressed_image.GetData(), out_compressed_image.GetData() + out_compressed_image.Num());
		//
		//detection_augre.url = std::string(TCHAR_TO_UTF8(*detection.url));

		FRobofleetUnrealClientModule::Get()->RobofleetClient->PublishDetection(detection_augre);
		FRobofleetUnrealClientModule::Get()->RobofleetClient->AddDetectionToAugReMap(detection_augre);
	}
}

// TODO: remove 
// 
//void URobofleetBPFunctionLibrary::AddDetection(const FDetectedItem& detection)
//{
//	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
//	{
//		DetectedItem_augre detection_augre;
//
//		detection_augre.uid = std::string(TCHAR_TO_UTF8(*detection.uid));
//		detection_augre.callsign = std::string(TCHAR_TO_UTF8(*detection.callsign));
//		detection_augre.type = std::string(TCHAR_TO_UTF8(*detection.type));
//		detection_augre.type_label = std::string(TCHAR_TO_UTF8(*detection.type_label));
//		detection_augre.how = std::string(TCHAR_TO_UTF8(*detection.how));
//		detection_augre.how_label = std::string(TCHAR_TO_UTF8(*detection.how_label));
//
//		detection_augre.pose.header.frame_id = std::string(TCHAR_TO_UTF8(*detection.pose.header.frame_id));
//		detection_augre.pose.header.seq = detection.pose.header.seq;
//		detection_augre.pose.header.stamp._nsec = FDateTime::UtcNow().GetMillisecond() * 1000000;
//		detection_augre.pose.header.stamp._sec = FDateTime::UtcNow().ToUnixTimestamp();
//
//		detection_augre.pose.pose.position.x = detection.pose.Transform.GetTranslation().X;
//		detection_augre.pose.pose.position.y = detection.pose.Transform.GetTranslation().Y;
//		detection_augre.pose.pose.position.z = detection.pose.Transform.GetTranslation().Z;
//		detection_augre.pose.pose.orientation.x = detection.pose.Transform.GetRotation().X;
//		detection_augre.pose.pose.orientation.y = detection.pose.Transform.GetRotation().Y;
//		detection_augre.pose.pose.orientation.z = detection.pose.Transform.GetRotation().Z;
//		detection_augre.pose.pose.orientation.w = detection.pose.Transform.GetRotation().W;
//
//		detection_augre.cmpr_image.header.frame_id = std::string(TCHAR_TO_UTF8(*detection.cmpr_image.header.frame_id));
//		detection_augre.cmpr_image.header.seq = detection.cmpr_image.header.seq;
//		detection_augre.cmpr_image.header.stamp._nsec = FDateTime::UtcNow().GetMillisecond() * 1000000;
//		detection_augre.cmpr_image.header.stamp._sec = FDateTime::UtcNow().ToUnixTimestamp();
//		detection_augre.cmpr_image.format = std::string(TCHAR_TO_UTF8(*detection.cmpr_image.format));
//
//		for (auto& data : detection.cmpr_image.data)
//		{
//			detection_augre.cmpr_image.data.push_back(data);
//		}
//		FRobofleetUnrealClientModule::Get()->RobofleetClient->AddDetectionToAugReMap(detection_augre);
//	}
//}

void URobofleetBPFunctionLibrary::PublishImuMsg(const FString& TopicName, const FString& Namespace, const FImu& Msg)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		// init
		Imu dst;

		// Convert and fill ROS sensor_msgs/PointCloud2 message
		dst.header.frame_id = std::string(TCHAR_TO_UTF8(*Msg.header.frame_id));
		dst.header.stamp._sec = Msg.header.stamp._sec;
		dst.header.stamp._nsec = Msg.header.stamp._nsec * 1000; // fill this with millisecond data available on hololens
		dst.header.seq = Msg.header.seq;

		// fill orientation
		FQuat rh_quat = ConvertQuaternionToRightHand(Msg.orientation);
		//dst.orientation.x = rh_quat.X;
		//dst.orientation.y = rh_quat.Y;
		//dst.orientation.z = rh_quat.Z;
		//dst.orientation.w = rh_quat.W;

		dst.orientation.x = 0;
		dst.orientation.y = 0;
		dst.orientation.z = 0;
		dst.orientation.w = 0;

		// fill ang vel
		dst.angular_velocity.x = Msg.angular_velocity.X; // not sure what this unit is? deg/s? rad/s? deg/ms? rad/ms?
		dst.angular_velocity.y = Msg.angular_velocity.Y;
		dst.angular_velocity.z = Msg.angular_velocity.Z;

		// fill lin accel
		dst.linear_acceleration.x = Msg.linear_acceleration.X; // not sure what this unit is? m/s^2? cm/s^2 m/ms^2 cm/ms^2
		dst.linear_acceleration.y = Msg.linear_acceleration.Y;
		dst.linear_acceleration.z = Msg.linear_acceleration.Z;

		// size of covariance matrix ROS message expects
		size_t vec_size = 9;
		
		// fill empty
		dst.orientation_covariance.assign(vec_size,0);
		dst.angular_velocity_covariance.assign(vec_size, 0);
		dst.linear_acceleration_covariance.assign(vec_size, 0);

		// seting to -1 lets ros know we do not have this information.
		dst.orientation_covariance[0] = -1.0;
		dst.angular_velocity_covariance[0] = -1.0;
		dst.linear_acceleration_covariance[0] = -1.0;

		// if input covariance arrays have data and are equal to 9, copy over the data to the ros msg struct
		bool is_orien_covar_data{ (Msg.orientation_covariance.Num() == vec_size) ? true : false };
		bool is_ang_vel_covar_data{ (Msg.orientation_covariance.Num() == vec_size) ? true : false };
		bool is_lin_acc_covar_data{ (Msg.orientation_covariance.Num() == vec_size) ? true : false };

		for (size_t i{ 0 }; i < vec_size; i++)
		{
			if (is_orien_covar_data) { dst.orientation_covariance[i] = static_cast<double>(Msg.orientation_covariance[i]); }
			if (is_ang_vel_covar_data) { dst.angular_velocity_covariance[i] = static_cast<double>(Msg.angular_velocity_covariance[i]); }
			if (is_lin_acc_covar_data) { dst.linear_acceleration_covariance[i] = static_cast<double>(Msg.linear_acceleration_covariance[i]); }
		}

		FRobofleetUnrealClientModule::Get()->RobofleetClient->PublishImuMsg(
			std::string(TCHAR_TO_UTF8(*TopicName)),
			std::string(TCHAR_TO_UTF8(*Namespace)),
			dst);
	}
}

void URobofleetBPFunctionLibrary::PublishOdomMsg(const FString& AgentName, const FOdometry& Msg)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		// init
		Odometry dst;

		// fill header
		dst.header.frame_id = std::string(TCHAR_TO_UTF8(*Msg.header.frame_id));
		dst.header.stamp._sec = FDateTime::UtcNow().ToUnixTimestamp();
		dst.header.stamp._nsec = FDateTime::UtcNow().GetMillisecond() * 1000000;
		dst.header.seq = Msg.header.seq;
		dst.child_frame_id = std::string(TCHAR_TO_UTF8(*Msg.child_frame_id));

		// convert pose and twist
		dst.pose.pose.position.x = Msg.pose.pose.GetLocation().X;
		dst.pose.pose.position.y = Msg.pose.pose.GetLocation().Y;
		dst.pose.pose.position.z = Msg.pose.pose.GetLocation().Z;
		dst.pose.pose.orientation.x = Msg.pose.pose.GetRotation().X;
		dst.pose.pose.orientation.y = Msg.pose.pose.GetRotation().Y;
		dst.pose.pose.orientation.z = Msg.pose.pose.GetRotation().Z;
		dst.pose.pose.orientation.z = Msg.pose.pose.GetRotation().W;

		dst.twist.twist.linear.x = Msg.twist.twist.linear.X;
		dst.twist.twist.linear.y = Msg.twist.twist.linear.Y;
		dst.twist.twist.linear.z = Msg.twist.twist.linear.Z;
		dst.twist.twist.angular.x = Msg.twist.twist.angular.X;
		dst.twist.twist.angular.y = Msg.twist.twist.angular.Y;
		dst.twist.twist.angular.z = Msg.twist.twist.angular.Z;

		// init covariances
		size_t vec_size = 36;
		dst.pose.covariance.assign(vec_size, 0);
		dst.twist.covariance.assign(vec_size, 0);

		// handle empty covariance matrices from unreal
		bool is_pose_covar_data = (Msg.pose.covariance.Num() > 0) ? true : false;
		bool is_twist_covar_data = (Msg.twist.covariance.Num() > 0) ? true : false;

		// fill covariance matrices
		for (size_t i{ 0 }; i < vec_size; i++)
		{
			if (is_pose_covar_data) {dst.pose.covariance[i] = static_cast<double>(Msg.pose.covariance[i]);}
			if (is_twist_covar_data) { dst.twist.covariance[i] = static_cast<double>(Msg.twist.covariance[i]); }
		}
		
		// send over robofleet
		FRobofleetUnrealClientModule::Get()->RobofleetClient->PublishOdomMsg(
			AgentName, 
			dst);
	}
}


FPath URobofleetBPFunctionLibrary::GetRobotPath(const FString& RobotName, const FString& Type)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		FPath Fp = FRobofleetUnrealClientModule::Get()->RobofleetClient->GetFPath(RobotName, Type);
		return Fp;
	}
	return FPath();
}

void URobofleetBPFunctionLibrary::PublishHapticsResearchMsg(const FString& RobotName, const FPoseStamped& PoseStampedMsg, FDateTime CurTimeStamp)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		PoseStamped Goal;
		Goal.header.frame_id = std::string(TCHAR_TO_UTF8(*PoseStampedMsg.header.frame_id));
		Goal.header.stamp._nsec = PoseStampedMsg.header.stamp._nsec;
		Goal.header.stamp._sec = (int)CurTimeStamp.ToUnixTimestamp();
		Goal.header.seq = PoseStampedMsg.header.seq;

		Goal.pose.position.x = PoseStampedMsg.Transform.GetLocation().X;
		Goal.pose.position.y = 0.00;
		Goal.pose.position.z = 0.00;

		Goal.pose.orientation.x = 0.00;
		Goal.pose.orientation.y = 0.00;
		Goal.pose.orientation.z = 0.00;
		Goal.pose.orientation.w = 0.00;

		FRobofleetUnrealClientModule::Get()->RobofleetClient->PublishHapticsResearchMsg(RobotName, Goal);
	}
}

void URobofleetBPFunctionLibrary::PublishPointCloudMsg(const FString& TopicName, const FString& Namespace, const FPointCloud2& Msg)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		// Init
		PointCloud2 temp;
		uint32 time_stamp = FDateTime::UtcNow().GetMillisecond();

		// Convert and fill ROS sensor_msgs/PointCloud2 message
		temp.header.frame_id = std::string(TCHAR_TO_UTF8(*Msg.header.frame_id));
		temp.height = Msg.height;
		temp.width = Msg.width;
		temp.is_bigendian = Msg.is_bigendian;
		temp.point_step = Msg.point_step;
		temp.row_step = Msg.row_step;
		temp.is_dense = Msg.is_dense;

		// Fill data vector
		temp.data.reserve(Msg.data.Num());
		temp.data.assign(Msg.data.GetData(), Msg.data.GetData() + Msg.data.Num());

		// Fill fields vector
		temp.fields.reserve(Msg.fields.Num());
		for (const FPointField& src : Msg.fields)
		{
			PointField dst;
			dst.name = std::string(TCHAR_TO_UTF8(*src.name));;
			dst.offset = src.offset;
			dst.datatype = src.datatype;
			dst.count = src.count;
			temp.fields.push_back(dst);
		}

		FRobofleetUnrealClientModule::Get()->RobofleetClient->PublishPointCloudMsg(TopicName, Namespace, temp);
	}
}

void URobofleetBPFunctionLibrary::IsPointCloudReceived(const FString& RobotName, bool& MsgReceived)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		FRobofleetUnrealClientModule::Get()->RobofleetClient->IsPointCloudReceived(RobotName, MsgReceived);
	}
}

void URobofleetBPFunctionLibrary::GetPointCloudFrame(const FString& RobotName, FString& PointCloudFrame)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		FRobofleetUnrealClientModule::Get()->RobofleetClient->GetPointCloudFrame(RobotName, PointCloudFrame);
	}
}

void URobofleetBPFunctionLibrary::GetPointCloud(const FString& RobotName, FPointCloud2& PointCloudMsg, bool& MsgReceived)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		// Get reference to robofleet PointCloud2 struct
		TSharedPtr<PointCloud2> src;
		FRobofleetUnrealClientModule::Get()->RobofleetClient->GetPointCloud(RobotName, src, MsgReceived);
		if (!MsgReceived) return;

		// Assign data from robofleet PointCloud2 struct to Unreal Engine PointCloud2 struct
		PointCloudMsg.header.frame_id = FString(src->header.frame_id.c_str());
		PointCloudMsg.height = src->height;
		PointCloudMsg.width = src->width;
		PointCloudMsg.is_bigendian = src->is_bigendian;
		PointCloudMsg.point_step = src->point_step;
		PointCloudMsg.row_step = src->row_step;
		PointCloudMsg.is_dense = src->is_dense;

		// Copy data array
		PointCloudMsg.data.SetNum(src->data.size());
		FMemory::Memcpy(PointCloudMsg.data.GetData(), src->data.data(), src->data.size() * sizeof(uint8_t));

		// Copy robofleet PointField struct to Unreal Engine FPointField struct
		for (const PointField& field : src->fields)
		{
			FPointField dst;
			dst.name = FString(field.name.c_str());
			dst.offset = field.offset;
			dst.datatype = field.datatype;
			dst.count = field.count;
			PointCloudMsg.fields.Add(dst);
		}
	}
}

void URobofleetBPFunctionLibrary::GetMarkerArray(const FString& RobotName, FMarkerArray& Markers)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		TSharedPtr<MarkerArray> src;
		FRobofleetUnrealClientModule::Get()->RobofleetClient->GetMarkerArray(RobotName, src);

		Markers.points.Empty();
		for (auto& it_marker : src->markers)
		{
			FMarker marker_obj;
			marker_obj.header.frame_id = FString(it_marker.header.frame_id.c_str());
			marker_obj.header.seq = it_marker.header.seq;
			marker_obj.header.stamp._nsec = FDateTime::UtcNow().GetMillisecond() * 1000000;
			marker_obj.header.stamp._sec = FDateTime::UtcNow().ToUnixTimestamp();

			marker_obj.ns = FString(it_marker.ns.c_str());
			marker_obj.id = it_marker.id;
			marker_obj.type = it_marker.type;
			marker_obj.action = it_marker.action;

			marker_obj.pose.SetTranslation(FVector(it_marker.pose.position.x, it_marker.pose.position.y, it_marker.pose.position.z));
			marker_obj.pose.SetRotation(FQuat(it_marker.pose.orientation.x, it_marker.pose.orientation.y, it_marker.pose.orientation.z, it_marker.pose.orientation.w));

			marker_obj.scale.X = it_marker.scale.x;
			marker_obj.scale.Y = it_marker.scale.y;
			marker_obj.scale.Z = it_marker.scale.z;

			marker_obj.color.R = it_marker.color.r;
			marker_obj.color.G = it_marker.color.g;
			marker_obj.color.B = it_marker.color.b;
			marker_obj.color.A = it_marker.color.a;

			marker_obj.frame_locked = it_marker.frame_locked;

			for (const Point& point : it_marker.points)
			{
				FPoint dst;
				dst.x = point.x;
				dst.y = point.y;
				dst.z = point.z;
				marker_obj.points.Add(dst);
			}
			for (const ColorRGBA& color : it_marker.colors)
			{
				FColor dst;
				dst.R = color.r;
				dst.G = color.g;
				dst.B = color.b;
				dst.A = color.a;
				marker_obj.colors.Add(dst);
			}

			marker_obj.text = FString(it_marker.text.c_str());
			Markers.points.Add(marker_obj);
		}
	}
}

void URobofleetBPFunctionLibrary::ReadPixelData(UTextureRenderTarget2D* RenderTarget, TArray<FColor>& OutPixels)
{
	if (RenderTarget)
	{
		FRenderTarget* RenderTargetResource = RenderTarget->GameThread_GetRenderTargetResource();
		if (RenderTargetResource)
		{
			FReadSurfaceDataFlags ReadDataFlags;
			ReadDataFlags.SetLinearToGamma(false); // Set this to true if gamma correction is needed

			RenderTargetResource->ReadPixels(OutPixels, ReadDataFlags);
			UE_LOG(LogTemp, Warning, TEXT("Pixels: %d %d %d"), OutPixels[0].R, OutPixels[0].G, OutPixels[0].B);
			UE_LOG(LogTemp, Warning, TEXT("Pixels: %d %d %d"), OutPixels[100].R, OutPixels[100].G, OutPixels[100].B);
			UE_LOG(LogTemp, Warning, TEXT("Pixels: %d %d %d"), OutPixels[200].R, OutPixels[200].G, OutPixels[200].B);
		}
	}
}

void URobofleetBPFunctionLibrary::PublishVLCImageMsg(const FString& TopicName, const FString& Namespace, const FImageROS& ImageMsg)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		// Init
		Image temp;
		uint32 time_stamp = FDateTime::UtcNow().GetMillisecond();

		// Convert and fill ROS sensor_msgs/Image message
		temp.header.frame_id = std::string(TCHAR_TO_UTF8(*ImageMsg.header.frame_id));
		temp.header.seq = ImageMsg.header.seq;
		temp.header.stamp._nsec = time_stamp * 1000000;
		temp.header.stamp._sec = time_stamp;
		temp.height = ImageMsg.height;
		temp.width = ImageMsg.width;
		temp.encoding = std::string(TCHAR_TO_UTF8(*ImageMsg.encoding));
		temp.is_bigendian = ImageMsg.is_bigendian;
		temp.step = ImageMsg.step;

		// Fill robofleet vector with data
		temp.data.reserve(ImageMsg.data.Num());
		temp.data.assign(ImageMsg.data.GetData(), ImageMsg.data.GetData() + ImageMsg.data.Num());

		// Call RobofleetClient Publish Image Msg
		FRobofleetUnrealClientModule::Get()->RobofleetClient->PublishImageMsg(TopicName, Namespace, temp);

		//UE_LOG(LogTemp, Warning, TEXT("Byte Array Size: %d"), ImageMsg.data.Num());
	}
	else { UE_LOG(LogTemp, Error, TEXT("Robofleet Session Not Runnning")); }

}

void URobofleetBPFunctionLibrary::PublishVLCCompressedImageMsg(const FString& TopicName,
	const FString& Namespace,
	const FCompressedImage& CompressedImageMsg,
	const int& ImageHeight,
	const int& ImageWidth,
	const int& BitDepth)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		// Init
		CompressedImage temp;
		TArray<uint8> OutCompressedImageMsg;
		uint32 time_stamp = FDateTime::UtcNow().GetMillisecond();

		// Convert and fill ROS sensor_msgs/CompressedImage message
		temp.header.frame_id = std::string(TCHAR_TO_UTF8(*CompressedImageMsg.header.frame_id));
		temp.header.seq = CompressedImageMsg.header.seq;
		temp.header.stamp._nsec = time_stamp * 1000000;
		temp.header.stamp._sec = time_stamp;
		temp.format = std::string(TCHAR_TO_UTF8(*CompressedImageMsg.format));

		// Compress image to jpeg
		ToCompressedJPEGImage(CompressedImageMsg.data.GetData(), ImageHeight, ImageWidth, BitDepth, ERGBFormat::Gray, OutCompressedImageMsg);

		// Fill robofleet vector with data
		temp.data.reserve(OutCompressedImageMsg.Num());
		temp.data.assign(OutCompressedImageMsg.GetData(), OutCompressedImageMsg.GetData() + OutCompressedImageMsg.Num());

		//UE_LOG(LogTemp, Warning, TEXT("Compression Size: %d"), OutCompressedImageMsg.Num());

		// Call RobofleetClient publish msg
		FRobofleetUnrealClientModule::Get()->RobofleetClient->PublishCompressedImageMsg(TopicName, Namespace, temp);
	}
	else { UE_LOG(LogTemp, Error, TEXT("Robofleet Session Not Runnning")); }

}

void  URobofleetBPFunctionLibrary::PublishPVImageMsg(const FString& TopicName,
	const FString& Namespace,
	const FImageROS& ImageMsg,
	UTextureRenderTarget2D* RenderTarget)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		// Init
		Image temp;
		TArray<uint8> out_image;
		uint32 Height{ 0 };
		uint32 Width{ 0 };
		uint32 BitDepth{ 0 };

		// Set header and format
		temp.header.frame_id = std::string(TCHAR_TO_UTF8(*ImageMsg.header.frame_id));
		temp.header.seq = ImageMsg.header.seq;
		temp.header.stamp._nsec = FDateTime::UtcNow().GetMillisecond() * 1000000;
		temp.header.stamp._sec = FDateTime::UtcNow().ToUnixTimestamp();
		temp.height = ImageMsg.height;
		temp.width = ImageMsg.width;
		temp.encoding = std::string(TCHAR_TO_UTF8(*ImageMsg.encoding));
		temp.is_bigendian = ImageMsg.is_bigendian;
		temp.step = ImageMsg.step;

		// Get UTexture2D into a TArray
		GetByteArrayFromTextureRenderTarget(RenderTarget, out_image, Height, Width, BitDepth);

		// Fill robofleet vector with data
		temp.data.reserve(out_image.Num());
		temp.data.assign(out_image.GetData(), out_image.GetData() + out_image.Num());

		// Call RobofleetClient publish msg
		FRobofleetUnrealClientModule::Get()->RobofleetClient->PublishImageMsg(TopicName, Namespace, temp);

	}
	else { UE_LOG(LogTemp, Error, TEXT("Robofleet Session Not Runnning")); }

}

void  URobofleetBPFunctionLibrary::PublishPVCompressedImageMsg(const FString& TopicName,
	const FString& Namespace,
	const FCompressedImage& ImageMsg,
	UTextureRenderTarget2D* RenderTarget)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		CompressedImage temp;
		TArray<uint8> out_image;

		// Set header and format
		temp.header.frame_id = std::string(TCHAR_TO_UTF8(*ImageMsg.header.frame_id));
		temp.header.seq = ImageMsg.header.seq;
		temp.header.stamp._nsec = FDateTime::UtcNow().GetMillisecond() * 1000000;
		temp.header.stamp._sec = FDateTime::UtcNow().ToUnixTimestamp();
		temp.format = std::string(TCHAR_TO_UTF8(*ImageMsg.format));

		// Convert UTextureRenderTarget2D into a TArray
		GetCompressedByteArrayFromTextureRenderTarget(RenderTarget, out_image);

		// reserve space in image vector to avoid reallocations
		temp.data.resize(out_image.Num());

		// copy the contents of the TArray to the std::vector
		temp.data.assign(out_image.GetData(), out_image.GetData() + out_image.Num());

		UE_LOG(LogTemp, Warning, TEXT("JPEG Size: %d"), temp.data.size());

		FRobofleetUnrealClientModule::Get()->RobofleetClient->PublishCompressedImageMsg(TopicName, Namespace, temp);
	}

}

void URobofleetBPFunctionLibrary::ToCompressedJPEGImage(const void* InRawImageData,
	const uint32& InHeight,
	const uint32& InWidth,
	const uint32& BitDepth,
	const ERGBFormat RawFormat,
	TArray<uint8>& OutCompressedImageData)
{
	// Create image wrapper class for JPEG image
	static IImageWrapperModule& jpeg_image_wrapper_module = FModuleManager::LoadModuleChecked<IImageWrapperModule>(FName("ImageWrapper"));
	static TSharedPtr<IImageWrapper> jpeg_image_wrapper = jpeg_image_wrapper_module.CreateImageWrapper(EImageFormat::JPEG);

	// Set the size of color pixel array
	const uint32 byte_array_size = InHeight * InWidth * BitDepth;

	// Set image wrapper raw data
	jpeg_image_wrapper->SetRaw(InRawImageData,
		byte_array_size,
		InHeight,
		InWidth,
		RawFormat, 8);

	// Assign compressed image
	OutCompressedImageData = jpeg_image_wrapper->GetCompressed();

}

void URobofleetBPFunctionLibrary::GetByteArrayFromTextureRenderTarget(UTextureRenderTarget2D* RenderTarget, TArray<uint8>& OutRawImageData, uint32& OutHeight, uint32& OutWidth, uint32& OutBitDepth)
{
	// Get render target resource
	FRenderTarget* render_target = RenderTarget->GameThread_GetRenderTargetResource();

	// Set the size of color pixel array
	FIntPoint render_target_size = render_target->GetSizeXY();

	const uint32 raw_image_array_size = render_target_size.X * render_target_size.Y * 4;

	/*OutWidth = render_target_size.X;
	OutHeight = render_target_size.Y;*/

	OutWidth = RenderTarget->SizeX;
	OutHeight = RenderTarget->SizeY;

	// Use a preallocated buffer for the raw byte array
	static TArray<FColor> raw_color_buffer;
	raw_color_buffer.SetNumUninitialized(raw_image_array_size);
	render_target->ReadPixelsPtr(raw_color_buffer.GetData());

	OutBitDepth = 4;
	const uint32 ColorDataSize = raw_color_buffer.Num();
	const uint32 TextureSize = raw_color_buffer.Num() * OutBitDepth;
	OutRawImageData.SetNumUninitialized(TextureSize);
	UE_LOG(LogTemp, Error, TEXT("Texture Size: %d"), TextureSize);

	for (uint32 i = 0; i < ColorDataSize; i++)
	{
		OutRawImageData[(i * 4)] = raw_color_buffer[i].R;
		OutRawImageData[(i * 4) + 1] = raw_color_buffer[i].G;
		OutRawImageData[(i * 4) + 2] = raw_color_buffer[i].B;
		OutRawImageData[(i * 4) + 3] = raw_color_buffer[i].A;
	}
}

void URobofleetBPFunctionLibrary::GetCompressedByteArrayFromTextureRenderTarget(UTextureRenderTarget2D* RenderTarget, TArray<uint8>& OutCompressedImageData)
{
	TArray<uint8> RawImageData;
	uint32 Height{ 0 };
	uint32 Width{ 0 };
	uint32 BitDepth{ 0 };

	GetByteArrayFromTextureRenderTarget(RenderTarget, RawImageData, Height, Width, BitDepth);
	ToCompressedJPEGImage(RawImageData.GetData(), Height, Width, BitDepth, ERGBFormat::RGBA, OutCompressedImageData);
}

void URobofleetBPFunctionLibrary::GetRenderTargetFormat(UTextureRenderTarget2D* RenderTarget, FString& Type)
{
	// Print out pixel format
	EPixelFormat pixel_format = RenderTarget->GetFormat();
	const UEnum* EnumPtr = FindObject<UEnum>(ANY_PACKAGE, TEXT("EPixelFormat"), true);
	if (EnumPtr)
	{
		Type = EnumPtr->GetDisplayNameTextByValue((int64)pixel_format).ToString();
		UE_LOG(LogTemp, Warning, TEXT("Render target format: %s"), *Type);
	}
	else
	{
		Type = "Cannot Get Type";
	}

}

// Utility Functions
void URobofleetBPFunctionLibrary::GetRelativeTransform(const FTransform& target_frame, const FTransform& source_frame, FTransform& transform)
{
	source_frame.Inverse();
	transform.SetTranslation(source_frame.Rotator().GetInverse().RotateVector(target_frame.GetTranslation() - source_frame.GetTranslation()));
	transform.SetRotation(FQuat(source_frame.Rotator().GetInverse()) * FQuat(target_frame.Rotator()));
}

void URobofleetBPFunctionLibrary::ConvertTransformToRobotConvention(const FTransform& unreal_transform, FTransform& robot_transform)
{
	// To Right Hand and Meters
	// translation
	FVector trans_comp{ 0,0,0 };
	trans_comp.X = unreal_transform.GetTranslation().X / 100;
	trans_comp.Y = -unreal_transform.GetTranslation().Y / 100;
	trans_comp.Z = unreal_transform.GetTranslation().Z / 100;
	robot_transform.SetTranslation(trans_comp);

	// rotation
	FRotator rot_comp{ 0,0,0 };
	rot_comp.Roll = unreal_transform.Rotator().Roll * -1;
	rot_comp.Pitch = unreal_transform.Rotator().Pitch;
	rot_comp.Yaw = unreal_transform.Rotator().Yaw * -1;
	robot_transform.SetRotation(rot_comp.Quaternion());
}


void URobofleetBPFunctionLibrary::PublishAudioData(const FString& TopicName, const FString& Namespace, const FAudioData& Msg)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		AudioData dst;

		// Fill data vector
		dst.data.reserve(Msg.data.Num());
		dst.data.assign(Msg.data.GetData(), Msg.data.GetData() + Msg.data.Num());

		FRobofleetUnrealClientModule::Get()->RobofleetClient->PublishAudioData(std::string(TCHAR_TO_UTF8(*TopicName)),
			std::string(TCHAR_TO_UTF8(*Namespace)), dst);
	}
}

void URobofleetBPFunctionLibrary::PublishAudioDataStamped(const FString& TopicName, const FString& Namespace, const FAudioDataStamped& Msg)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		AudioDataStamped dst;
		uint32 time_stamp = FDateTime::UtcNow().GetMillisecond();

		// Convert and fill ROS sensor_msgs/PointCloud2 message
		dst.header.frame_id = std::string(TCHAR_TO_UTF8(*Msg.header.frame_id));
		dst.header.stamp._sec = time_stamp;
		dst.header.stamp._nsec = time_stamp * 1000000;

		// Fill data vector
		dst.audio.data.reserve(Msg.audio.data.Num());
		dst.audio.data.assign(Msg.audio.data.GetData(), Msg.audio.data.GetData() + Msg.audio.data.Num());

		FRobofleetUnrealClientModule::Get()->RobofleetClient->PublishAudioDataStamped(std::string(TCHAR_TO_UTF8(*TopicName)),
			std::string(TCHAR_TO_UTF8(*Namespace)), dst);
	}
}

void URobofleetBPFunctionLibrary::PublishAudioInfo(const FString& TopicName, const FString& Namespace, const FAudioInfo& Msg)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		AudioInfo dst;
		dst.channels = Msg.channels;
		dst.sample_rate = Msg.sample_rate;
		dst.sample_format = std::string(TCHAR_TO_UTF8(*Msg.sample_format));
		dst.bitrate = Msg.bitrate;
		dst.coding_format = std::string(TCHAR_TO_UTF8(*Msg.coding_format));

		FRobofleetUnrealClientModule::Get()->RobofleetClient->PublishAudioInfo(std::string(TCHAR_TO_UTF8(*TopicName)),
			std::string(TCHAR_TO_UTF8(*Namespace)), dst);
	}
}

