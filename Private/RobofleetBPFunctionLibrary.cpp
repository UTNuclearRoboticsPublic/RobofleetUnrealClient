// Fill out your copyright notice in the Description page of Project Settings.


#include "RobofleetBPFunctionLibrary.h"
#include "Modules/ModuleManager.h"
#include "RobofleetUnrealClientModule.h"
#include "RobofleetClientBase.h"

void URobofleetBPFunctionLibrary::ConfigRobofleetSession(FString HostUrl, const UObject* WorldContextObject)
{
	URobofleetBase* RoboClient = FRobofleetUnrealClientModule::Get()->RobofleetClient;
	if (IsValid(FRobofleetUnrealClientModule::Get()->RobofleetClient))
	{
		FRobofleetUnrealClientModule::Get()->RobofleetClient->Initialize(HostUrl, WorldContextObject);
	}
	else
	{
		UE_LOG(LogRobofleet, Error, TEXT("Robofleet session not running"));
	}
}

FString URobofleetBPFunctionLibrary::GetRobotStatus(const FString& RobotName)
{
	URobofleetBase* RoboClient = FRobofleetUnrealClientModule::Get()->RobofleetClient;
	if (IsValid(RoboClient))
	{
		return RoboClient->GetRobotStatus(RobotName);
	}
	return TEXT("");
}

