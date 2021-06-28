// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "RobofleetBPFunctionLibrary.generated.h"

/**
 * 
 */
UCLASS()
class ROBOFLEETUNREALCLIENT_API URobofleetBPFunctionLibrary : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()
	
	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	static void StartRobofleetSession();

	UFUNCTION(BlueprintCallable, Category = "Robofleet", meta = (WorldContext = "WorldContextObject"))
	static void ConfigRobofleetSession(FString HostUrl, const UObject* WorldContextObject);

};