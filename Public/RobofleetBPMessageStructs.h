// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "RobofleetBPMessageStructs.generated.h"

/**
 *
 */
USTRUCT(BlueprintType)
struct FHeader
{
	GENERATED_BODY()
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	float x;

};

USTRUCT(BlueprintType)
struct FRobotStatus
{
	GENERATED_BODY()
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	float x;

};

USTRUCT(BlueprintType)
struct FRobotLocationStamped
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	float x;
};
