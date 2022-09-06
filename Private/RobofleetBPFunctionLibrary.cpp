// Fill out your copyright notice in the Description page of Project Settings.


#include "RobofleetBPFunctionLibrary.h"
#include "Modules/ModuleManager.h"
#include "RobofleetUnrealClientModule.h"
#include "RobofleetClientBase.h"

void URobofleetBPFunctionLibrary::StartRobofleetSession(FString HostUrl, const UObject* WorldContextObject)
{
	FRobofleetUnrealClientModule::Get()->StartRobofleetSession(HostUrl, WorldContextObject);
}

FString URobofleetBPFunctionLibrary::GetRobotStatus(const FString& RobotName)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		return FRobofleetUnrealClientModule::Get()->RobofleetClient->GetRobotStatus(RobotName);
	}
	return TEXT("");
}


//augre_msgs

FString URobofleetBPFunctionLibrary::GetName(const FString& RobotName)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		return FRobofleetUnrealClientModule::Get()->RobofleetClient->GetName(RobotName);
	}
	return TEXT("");
}

FString URobofleetBPFunctionLibrary::GetAgentDisplayName(const FString& RobotName)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		return FRobofleetUnrealClientModule::Get()->RobofleetClient->GetAgentDisplayName(RobotName);
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
	return FVector(0,0,0);
}

TArray<FString> URobofleetBPFunctionLibrary::GetAllRobotsAtSite(const FString& Location)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		return FRobofleetUnrealClientModule::Get()->RobofleetClient->GetAllRobotsAtSite(Location);
	}
	return TArray<FString>();
}

bool URobofleetBPFunctionLibrary::IsRobotOk(const FString& RobotName)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		return FRobofleetUnrealClientModule::Get()->RobofleetClient->IsRobotOk(RobotName);
	}
	return false;
}

TArray<uint8> URobofleetBPFunctionLibrary::GetRobotImage(const FString& RobotName)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		return FRobofleetUnrealClientModule::Get()->RobofleetClient->GetRobotImage(RobotName);
	}
	return TArray<uint8>();
}

bool URobofleetBPFunctionLibrary::IsRobotImageCompressed(const FString& RobotName)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		return FRobofleetUnrealClientModule::Get()->RobofleetClient->IsRobotImageCompressed(RobotName);
	}
	return bool();
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

FString URobofleetBPFunctionLibrary::GetDetectedName(const FString& RobotName)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		return FRobofleetUnrealClientModule::Get()->RobofleetClient->GetDetectedName(RobotName);
	}
	return TEXT("");
}


// TODO: REMOVE REP ID
FString URobofleetBPFunctionLibrary::GetDetectedRepIDRef(const FString& RobotName)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		return FRobofleetUnrealClientModule::Get()->RobofleetClient->GetDetectedRepIDRef(RobotName);
	}
	return TEXT("");
}

FString URobofleetBPFunctionLibrary::GetDetectedAnchorIDRef(const FString& RobotName)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		return FRobofleetUnrealClientModule::Get()->RobofleetClient->GetDetectedAnchorIDRef(RobotName);
	}
	return TEXT("");
}

FVector URobofleetBPFunctionLibrary::GetDetectedPositionRef(const FString& RobotName)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		return FRobofleetUnrealClientModule::Get()->RobofleetClient->GetDetectedPositionRef(RobotName);
	}
	return FVector(0, 0, 0);
}

FVector URobofleetBPFunctionLibrary::GetDetectedPositionGlobal(const FString& RobotName)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		return FRobofleetUnrealClientModule::Get()->RobofleetClient->GetDetectedPositionGlobal(RobotName);
	}
	return FVector(0, 0, 0);
}

TArray<uint8> URobofleetBPFunctionLibrary::GetDetectedImage(const FString& RobotName)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		return FRobofleetUnrealClientModule::Get()->RobofleetClient->GetDetectedImage(RobotName);
	}
	return TArray<uint8>();
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


// Publish Messages to Robofleet
// *TODO: These need to be rate limited somewhere in the client as there is the potential to overload the server
void URobofleetBPFunctionLibrary::PublishStatusMsg(const FString& RobotName, const FRobotStatus& StatusMsg)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		RobotStatus robot_status;
		robot_status.battery_level = StatusMsg.battery_level;
		robot_status.location = std::string(TCHAR_TO_UTF8(*StatusMsg.location));
		robot_status.status = std::string(TCHAR_TO_UTF8(*StatusMsg.status));
		robot_status.is_ok = StatusMsg.is_ok;

		FRobofleetUnrealClientModule::Get()->RobofleetClient->PublishStatusMsg(RobotName, robot_status);
	}
}

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

void URobofleetBPFunctionLibrary::PublishAzureSpatialAnchorMsg(const FString& RobotName, const FAzureSpatialAnchor& FAzureSpatialAnchorMsg)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		AzureSpatialAnchor AzureSpatialAnchor;
		AzureSpatialAnchor.asa_id = std::string(TCHAR_TO_UTF8(*FAzureSpatialAnchorMsg.asa_id));
		AzureSpatialAnchor.rep_id = std::string(TCHAR_TO_UTF8(*FAzureSpatialAnchorMsg.rep_id));
		AzureSpatialAnchor.ns = std::string(TCHAR_TO_UTF8(*FAzureSpatialAnchorMsg.ns));
		AzureSpatialAnchor.timestamp._nsec = FAzureSpatialAnchorMsg.timestamp._nsec;
		AzureSpatialAnchor.timestamp._sec = FDateTime::Now().ToUnixTimestamp();
		//AzureSpatialAnchor.timestamp._sec = FAzureSpatialAnchorMsg.timestamp._sec;
		
		// PoseWithCovarianceStamped
		AzureSpatialAnchor.pose.header.frame_id = std::string(TCHAR_TO_UTF8(*FAzureSpatialAnchorMsg.pose.header.frame_id));
		AzureSpatialAnchor.pose.header.stamp._nsec = FAzureSpatialAnchorMsg.pose.header.stamp._nsec;
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
		//Convert from TArray to std::vector
		std::vector<float> pose_anchor_cov = { 71.1655073735117,	3.45821576403310,	0.593090091521150,	1.30137103899806,	0.664337209474972,	15.3887697684447,
										3.45821576403310,	2.17138180031235,	0.0889157579901040,	0.356811074478031, -1.54724979011027,	1.08088777296042,
										0.593090091521150,	0.0889157579901040,	0.312597527415474, -0.0669388300717367,	1.34318992031335,	0.200723741020181,
										1.30137103899806,	0.356811074478031, -0.0669388300717367,	0.770845292528924, -0.520578503254728,	0.245532369990909,
										0.664337209474972, -1.54724979011027,	1.34318992031335, -0.520578503254728,	9.36524337618332,	0.0369266731945052,
										15.3887697684447,	1.08088777296042,	0.200723741020181,	0.245532369990909,	0.0369266731945052,	4.16394130032915 };
		for (auto& cov : pose_anchor_cov)
		{
			AzureSpatialAnchor.pose.pose.covariance.push_back(cov);
		}

		// GeoPoseWithCovarianceStamped
		AzureSpatialAnchor.geopose.header.frame_id = std::string(TCHAR_TO_UTF8(*FAzureSpatialAnchorMsg.geopose.header.frame_id));
		AzureSpatialAnchor.geopose.header.stamp._nsec = FAzureSpatialAnchorMsg.geopose.header.stamp._nsec;
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
		agent_status.name = std::string(TCHAR_TO_UTF8(*StatusMsg.name));
		agent_status.display_name = std::string(TCHAR_TO_UTF8(*StatusMsg.display_name));
		agent_status.agent_type = std::string(TCHAR_TO_UTF8(*StatusMsg.agent_type));
		agent_status.battery = StatusMsg.battery;
		agent_status.owner = std::string(TCHAR_TO_UTF8(*StatusMsg.owner));
		agent_status.anchor_localization = StatusMsg.anchor_localization;
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

		navigation_path.header.frame_id = "anchor_" + asa_header;
		navigation_path.header.stamp._nsec = PathMsg.header.stamp._nsec;
		navigation_path.header.stamp._sec = PathMsg.header.stamp._sec;
		navigation_path.header.seq = PathMsg.header.seq;

		for (auto& poses : PathMsg.poses)
		{
			PoseStamped poseTemp;

			std::string asa_poses = std::string(TCHAR_TO_UTF8(*poses.header.frame_id));
			std::replace(asa_poses.begin(), asa_poses.end(), '-', '_');
			poseTemp.header.frame_id = "anchor_" + asa_poses;
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
		FRobofleetUnrealClientModule::Get()->RobofleetClient->PublishPath(RobotName, navigation_path);
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

// Publish TeMoto msgs
void URobofleetBPFunctionLibrary::PublishStartUMRFMsg(const FStartUMRF& StartUMRFMsg)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		StartUMRF StartUMRF;		

		StartUMRF.umrf_graph_name = std::string(TCHAR_TO_UTF8(*StartUMRFMsg.umrf_graph_name));
		StartUMRF.name_match_required = StartUMRFMsg.name_match_required;
		for (auto& Str : StartUMRFMsg.targets)
		{
			StartUMRF.targets.push_back(TCHAR_TO_UTF8(*Str));
		}

		StartUMRF.umrf_graph_json = std::string(TCHAR_TO_UTF8(*StartUMRFMsg.umrf_graph_json));

		//Convert from FUMRFgraphDiff to UMRFgraphDiff
		for (auto& umrfDiff : StartUMRFMsg.umrf_graph_diffs)
		{
			UMRFgraphDiff umrfTemp;
			umrfTemp.ADD = std::string(TCHAR_TO_UTF8(*umrfDiff.ADD));
			umrfTemp.SUBTRACT = std::string(TCHAR_TO_UTF8(*umrfDiff.SUBTRACT));
			umrfTemp.operation = std::string(TCHAR_TO_UTF8(*umrfDiff.operation));
			umrfTemp.umrf_json = std::string(TCHAR_TO_UTF8(*umrfDiff.umrf_json));				 
			StartUMRF.umrf_graph_diffs.push_back(umrfTemp);
		}

		FRobofleetUnrealClientModule::Get()->RobofleetClient->PublishStartUMRFMsg(StartUMRF);
	}
	else 
	{
		UE_LOG(LogTemp, Warning, TEXT("RobofleetClient No running"));
	}
	
}

void URobofleetBPFunctionLibrary::PublishStopUMRFMsg(const FStopUMRF& StopUMRFMsg)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		StopUMRF StopUMRF;		
		StopUMRF.umrf_graph_name = std::string(TCHAR_TO_UTF8(*StopUMRFMsg.umrf_graph_name));		
		for (auto& Str : StopUMRFMsg.targets)
		{
			StopUMRF.targets.push_back(TCHAR_TO_UTF8(*Str));
		}
		FRobofleetUnrealClientModule::Get()->RobofleetClient->PublishStopUMRFMsg(StopUMRF);
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("RobofleetClient No running"));
	}
}

FPath URobofleetBPFunctionLibrary::GetRobotPath(const FString& RobotName)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		FPath Fp = FRobofleetUnrealClientModule::Get()->RobofleetClient->GetFPath(RobotName);
		return Fp;
	}
	return FPath();
}
