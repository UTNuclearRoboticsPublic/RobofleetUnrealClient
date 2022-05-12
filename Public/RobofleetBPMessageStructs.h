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

USTRUCT(BlueprintType)
struct FAgentStatus
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FString name;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	float battery;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FString owner;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	bool anchor_localization;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FString control_status;
};

USTRUCT(BlueprintType)
struct FTransformStamped
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FHeader header;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FString child_frame_id;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FTransform Transform;
};

USTRUCT(BlueprintType)
struct FTransformWithCovarianceStamped
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FTransformStamped transform;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	TArray<float> covariance;
};

USTRUCT(BlueprintType)
struct FUMRFgraphDiff
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FString ADD;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FString SUBTRACT;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FString operation;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FString umrf_json;
};

USTRUCT(BlueprintType)
struct FStartUMRF
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FString umrf_graph_name;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	bool name_match_required;

	//vector
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	TArray<FString> targets;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FString umrf_graph_json;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	TArray<FUMRFgraphDiff> umrf_graph_diffs;

}; 

USTRUCT(BlueprintType)
struct FStopUMRF
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FString umrf_graph_name;

	//vector
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FString targets;

};

USTRUCT(BlueprintType)
struct FPoseStamped
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FHeader header;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FTransform Transform;
};

USTRUCT(BlueprintType)
struct FHttpDatabaseEntry
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "AugREDatabase")
	FString AsaId;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "AugREDatabase")
	FString RepresentationId;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "AugREDatabase")
	FString AnchorType;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "AugREDatabase")
	TArray<FString> Neighbors;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "AugREDatabase")
	FString Namespace;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "AugREDatabase")
	FDateTime TimeStamp;

};

USTRUCT(BlueprintType)
struct FPath
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FHeader header;

	//vector
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	TArray<FPoseStamped> poses;

};