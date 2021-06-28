// Fill out your copyright notice in the Description page of Project Settings.


#include "RobofleetBPFunctionLibrary.h"
#include "Modules/ModuleManager.h"
#include "RobofleetUnrealClientModule.h"
#include "RobofleetClientBase.h"

void URobofleetBPFunctionLibrary::StartRobofleetSession()
{
	FRobofleetUnrealClientModule::Get()->StartRobofleetSession();
}

void URobofleetBPFunctionLibrary::ConfigRobofleetSession(FString HostUrl, const UObject* WorldContextObject)
{
	FRobofleetUnrealClientModule::Get()->RobofleetClient->Initialize(HostUrl, WorldContextObject);
}


