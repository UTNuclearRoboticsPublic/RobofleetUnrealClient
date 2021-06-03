// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UObject/Object.h"
#include "MessageSchedulerLib.hpp"
#include "include/schema_generated.h"
#include "encode.hpp"
#include "decode.hpp"
#include "message_structs.h"
#include <string>
#include "WebsocketClient.h"

#include "RobofleetClientBase.generated.h"


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

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
	void deneme();

	void WebsocketDataCB(const void* Data);

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
