// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "RobofleetBPMessageStructs.generated.h"

/*
 * ROS message clones to message_structs.h in robofleet_client_lib
 */

USTRUCT(BlueprintType)
struct FTime
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	int32 _sec;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	int32 _nsec;
};



USTRUCT(BlueprintType)
struct FHeader
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	int32 seq;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FTime stamp;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FString frame_id;


};

USTRUCT(BlueprintType)
struct FRobotStatus
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FString status;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	bool is_ok;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	float battery_level;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FString location;
};

USTRUCT(BlueprintType)
struct FRobotLocationStamped
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FHeader header;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	float x;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	float y;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	float z;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	float theta;
};
