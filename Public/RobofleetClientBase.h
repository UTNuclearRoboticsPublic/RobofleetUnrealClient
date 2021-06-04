// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UObject/Object.h"
#include "MessageSchedulerLib.hpp"
#include "include/schema_generated.h"
#include <flatbuffers/flatbuffers.h>
#include "encode.hpp"
#include "decode.hpp"
#include "message_structs.h"
#include <string>
#include <map>
#include <memory> // TODO use UE4 shared ptr instead
#include "WebsocketClient.h"
#include<cstdint>
#include<string>

#include "RobofleetClientBase.generated.h"

struct RobotData {
	RobotLocation Location;
	RobotStatus Status;
	bool IsAlive;
};

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
	UWebsocketClient* hobarey;
	std::map<std::string, std::shared_ptr<RobotData> > RobotMap;

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	void deneme();

	void WebsocketDataCB(const void* Data);

	template <typename T> typename T DecodeMsg(const void* Data);
	void DecodeMsg(const void* Data, std::string topic, std::string RobotNamespace);

	void RegisterRobotSubscription(std::string TopicName, std::string RobotName, std::string MessageType);
	void RegisterRobotStatusSubscription();

	void PrintRobotsSeen();

	UFUNCTION()
	void PruneInactiveRobots();


	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	FString GetRobotStatus(const FString& RobotName);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	bool IsRobotOk(const FString& RobotName);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	float GetRobotBatteryLevel(const FString& RobotName);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	FString GetRobotLocationString(const FString& RobotName);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	FVector GetRobotPosition(const FString& RobotName);
	
	template <typename T>
	void encode_ros_msg(
		const T& msg, const std::string& msg_type, std::string&  from_topic,
		const std::string& to_topic) {

		// encode message
		flatbuffers::FlatBufferBuilder fbb;
		auto metadata = encode_metadata(fbb, msg_type, to_topic);
		auto root_offset = encode<T>(fbb, msg, metadata);
		fbb.Finish(flatbuffers::Offset<void>(root_offset));
		hobarey->Send(fbb.GetBufferPointer(), fbb.GetSize(), true);
	}
};