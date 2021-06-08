#pragma once

#include "CoreMinimal.h"
#include "UObject/Object.h"
#include "include/schema_generated.h"
#include "decode.hpp"
#include "encode.hpp"
#include "flatbuffers/flatbuffers.h"
#include "message_structs.h"
#include "MessageSchedulerLib.hpp"
#include "WebsocketClient.h"

#include <cstdint>
#include <map>
#include <string>

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
	URobofleetBase(int VerbosityLevel);

	int MaxQueueBeforeWaiting;
	int Verbosity = 0;
	UWebsocketClient* SocketClient;
	std::map<FString, TSharedPtr<RobotData> > RobotMap;
	std::map<FString, FDateTime> RobotsSeenTime;
	std::set<FString> RobotsSeen = {};

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
		void Connect(FString HostUrl);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
		void Disconnect();

	template <typename T> typename T DecodeMsg(const void* Data);
	void DecodeMsg(const void* Data, FString topic, FString RobotNamespace);

	template <typename T>
	void EncodeRosMsg(
		const T& msg, const std::string& msg_type, std::string& from_topic,
		const std::string& to_topic) {

		// encode message
		flatbuffers::FlatBufferBuilder fbb;
		auto metadata = encode_metadata(fbb, msg_type, to_topic);
		auto root_offset = encode<T>(fbb, msg, metadata);
		fbb.Finish(flatbuffers::Offset<void>(root_offset));
		SocketClient->Send(fbb.GetBufferPointer(), fbb.GetSize(), true);
	}

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
		FString GetRobotStatus(const FString& RobotName);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
		float GetRobotBatteryLevel(const FString& RobotName);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
		FString GetRobotLocationString(const FString& RobotName);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
		FVector GetRobotPosition(const FString& RobotName);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
		bool IsRobotOk(const FString& RobotName);

	UFUNCTION(BlueprintCallable, Category = "Robofleet")
		void PrintRobotsSeen();

	UFUNCTION()
		void PruneInactiveRobots();

	void WebsocketDataCB(const void* Data);

	void RegisterRobotSubscription(FString TopicName, FString RobotName, FString MessageType);
	void RegisterRobotStatusSubscription();

};