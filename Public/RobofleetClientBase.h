// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UObject/Object.h"
#include "RobofleetClientBase.generated.h"


//#include "LiveLinkSourceFactory.h"

/**
 *
 */
UCLASS(Blueprintable)
class ROBOFLEETUNREALCLIENT_API URobofleetBase : public UObject
{
	GENERATED_BODY()

public:
	// TODO: expose constructor to blueprints
	URobofleetBase();
	int MaxQueueBeforeWaiting;
	int Verbosity;
	FString HostUrl;

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	void deneme();

	void CallbackTest(const void* Data);
};
