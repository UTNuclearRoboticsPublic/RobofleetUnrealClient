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