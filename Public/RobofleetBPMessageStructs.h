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
struct FHeaderArrayStamped
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FHeader header;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	TArray<FHeader> data;
};


USTRUCT(BlueprintType)
struct FPoint
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	float x;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	float y;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	float z;
};

USTRUCT(BlueprintType)
struct FPose
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FPoint position;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FQuat orientation;
};

USTRUCT(BlueprintType)
struct FPose2D
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	float x;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	float y;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	float theta;
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
	FString uid;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FString agent_type;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FString callsign;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	float battery;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FString commander;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FString control_status;
};

// geometry_msgs

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
struct FPoseWithCovariance
{
	GENERATED_BODY()
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FTransform pose;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	TArray<float> covariance;
};

USTRUCT(BlueprintType)
struct FPoseWithCovarianceStamped
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FHeader header;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FPoseWithCovariance pose;
};

USTRUCT(BlueprintType)
struct FGeoPoseWithCovariance
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FTransform pose;
		
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	TArray<float> covariance;
};

USTRUCT(BlueprintType)
struct FGeoPoseWithCovarianceStamped
{
	GENERATED_BODY()
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FHeader header;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FGeoPoseWithCovariance pose;
};

USTRUCT(BlueprintType)
struct FTFMessage
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	TArray<FTransformStamped> transforms;
};

USTRUCT(BlueprintType)
struct FAzureSpatialAnchor
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FString asa_id;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FString rep_id;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FString ns;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FString anchor_type;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FTime timestamp;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FPoseWithCovarianceStamped pose;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FGeoPoseWithCovarianceStamped geopose;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	TArray<FString> neighbors;
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

USTRUCT(BlueprintType)
struct FTwist
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FVector linear;

	//vector
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FVector angular;

};

USTRUCT(BlueprintType)
struct FTwistStamped
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FHeader header;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FTwist twist;

};

USTRUCT(BlueprintType)
struct FTwistWithCovariance
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FTwist twist;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	TArray<float> covariance;
};


USTRUCT(BlueprintType)
struct FScrew
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FString action_name;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FString ee_name;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FString screw_frame;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FString is_pure_translation;
		
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FVector screw_axis;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FVector screw_origin;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	float screw_distance;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	float screw_pitch;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	float task_impedance_rotation;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	float task_impedance_translation;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	float theta_dot;
};


USTRUCT(BlueprintType)
struct FDetection
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FPoint position;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	float confidence;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	int label;
};

USTRUCT(BlueprintType)
struct FDetectionArray
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FHeader header;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	TArray<FDetection> detections;
};


USTRUCT(BlueprintType)
struct FPerson
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FPose pose;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	int id;
};

USTRUCT(BlueprintType)
struct FPersonArray
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FHeader header;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	TArray<FPerson> people;
};

// Affordance Primitive 
USTRUCT(BlueprintType)
struct FScrewStamped
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FHeader header;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FPoint origin;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FVector axis;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	bool is_pure_translation;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	float pitch;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	float min;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	float max;
};

// Occupancy Grid
USTRUCT(BlueprintType)
struct FMapMetaData
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FTime map_load_time;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	float resolution;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	int32 width;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	int32 height;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FTransform origin;

};


USTRUCT(BlueprintType)
struct FImageROS
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FHeader header;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	int32 height;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	int32 width;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FString encoding;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	uint8 is_bigendian;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	int32 step;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	TArray<uint8> data;
};

USTRUCT(BlueprintType)
struct FCompressedImage
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FHeader header;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FString format;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	TArray<uint8> data;
};

USTRUCT(BlueprintType)
struct FPointField
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FString name;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	int32 offset;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	uint8 datatype;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	int32 count;
};

USTRUCT(BlueprintType)
struct FPointCloud2
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FHeader header;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	int32 height;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	int32 width;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	TArray<FPointField> fields;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	bool is_bigendian;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	int32 point_step;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	int32 row_step;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	TArray<uint8> data;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	bool is_dense;
};

USTRUCT(BlueprintType)
struct FDetectedItem
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FString uid;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FString callsign;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FString type;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FString type_label;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FString how;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FString how_label;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FPoseStamped pose;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FCompressedImage cmpr_image;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FString url;
};

USTRUCT(BlueprintType)
struct FBoundingObject3D
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	int action;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	int shape;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FString uid;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	float size_x;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	float size_y;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	float size_z;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	float radius;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FPoseStamped centroid;

};

USTRUCT(BlueprintType)
struct FBoundingObject3DArray
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	TArray<FBoundingObject3D> objects;

};

USTRUCT(BlueprintType)
struct FMarker
{
	GENERATED_BODY()

		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
		FHeader header;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
		FString ns;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
		int32 id;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
		int32 type;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
		int32 action;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
		FTransform pose;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
		FVector scale;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
		FLinearColor color;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
		bool frame_locked;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
		TArray<FPoint> points;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
		TArray<FLinearColor> colors;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
		FString text;
};

USTRUCT(BlueprintType)
struct FMarkerArray
{
	GENERATED_BODY()

		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
		TArray<FMarker> points;
};

USTRUCT(BlueprintType)
struct FAudioData
{
	GENERATED_BODY()

		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
		TArray<uint8> data;
};

USTRUCT(BlueprintType)
struct FAudioDataStamped
{
	GENERATED_BODY()

		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
		FHeader header;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
		FAudioData audio;
};

USTRUCT(BlueprintType)
struct FAudioInfo
{
	GENERATED_BODY()

		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
		int channels;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
		int sample_rate;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
		FString sample_format;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
		int bitrate;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
		FString coding_format;
};

USTRUCT(BlueprintType)
struct FImu
{
	GENERATED_BODY()
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FHeader header;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FQuat orientation;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	TArray<float> orientation_covariance;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FVector angular_velocity;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	TArray<float> angular_velocity_covariance;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FVector linear_acceleration;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	TArray<float> linear_acceleration_covariance;
};


USTRUCT(BlueprintType)
struct FOdometry
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FHeader header;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FString child_frame_id;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FPoseWithCovariance pose;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Robofleet")
	FTwistWithCovariance twist;

};