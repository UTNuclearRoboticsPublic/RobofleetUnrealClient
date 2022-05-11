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

float URobofleetBPFunctionLibrary::GetRobotBatteryLevel(const FString& RobotName)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		return FRobofleetUnrealClientModule::Get()->RobofleetClient->GetRobotBatteryLevel(RobotName);
	}
	return 0.0f;
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

void URobofleetBPFunctionLibrary::PublishAgentStatusMsg(const FString& RobotName, const FAgentStatus& StatusMsg)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		AgentStatus agent_status;
		agent_status.name = std::string(TCHAR_TO_UTF8(*StatusMsg.name));
		agent_status.battery = StatusMsg.battery;
		agent_status.owner = std::string(TCHAR_TO_UTF8(*StatusMsg.owner));
		agent_status.anchor_localization = StatusMsg.anchor_localization;
		agent_status.control_status = std::string(TCHAR_TO_UTF8(*StatusMsg.control_status));

		FRobofleetUnrealClientModule::Get()->RobofleetClient->PublishAgentStatusMsg(RobotName, agent_status);
	}
}



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

// need to add detected image 

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

FPath URobofleetBPFunctionLibrary::GetRobotPath(const FString& RobotName)
{
	if (FRobofleetUnrealClientModule::Get()->IsSessionRunning())
	{
		FPath Fp = FRobofleetUnrealClientModule::Get()->RobofleetClient->GetFPath(RobotName);
		return Fp;
	}
	return FPath();
}
