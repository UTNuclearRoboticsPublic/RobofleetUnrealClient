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
	PoseStamped RobotPose; // TODO: Need to define class within robofleet_client_lib to accept geometry_msgs/PoseStamped - See Below
	bool IsAlive;
};

//Define Log Category and Verbosity
DECLARE_LOG_CATEGORY_EXTERN(LogRobofleet, Log, All);

//OnNewRobotSeen event
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnNewRobotSeen, FString, RobotName);

//OnRobotPruned event
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnRobotPruned, FString, RobotName);


UCLASS(Blueprintable)
class ROBOFLEETUNREALCLIENT_API URobofleetBase : public UObject
{
	GENERATED_BODY()

public:
	// TODO: expose constructor to blueprints
	URobofleetBase();
	~URobofleetBase();

private:

	int MaxQueueBeforeWaiting;
	int Verbosity = 0;
	bool bIsInitilized = false;

	UPROPERTY()
	UWebsocketClient* SocketClient;

	FTimerHandle RefreshTimerHandle;

	std::map<FString, TSharedPtr<RobotData> > RobotMap;
	std::map<FString, FDateTime> RobotsSeenTime;
	std::set<FString> RobotsSeen = {};

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
		if (SocketClient->IsValidLowLevel())
		{
			SocketClient->Send(fbb.GetBufferPointer(), fbb.GetSize(), true);
		}
		else
		{
			UE_LOG(LogRobofleet, Warning, TEXT("Message not sent since socket is destroyed"));
		}
	}

	void WebsocketDataCB(const void* Data);

public:

	bool IsInitilized();
	bool IsConnected();
	// TODO: Move the Blueprint exposure to the BP function library

	void Initialize(FString HostUrl, const UObject* WorldContextObject);

	void Disconnect();

	FString GetRobotStatus(const FString& RobotName);

	float GetRobotBatteryLevel(const FString& RobotName);

	FString GetRobotLocationString(const FString& RobotName);

	FVector GetRobotPosition(const FString& RobotName);

	FVector GetDronePosition(const FString& RobotName);

	bool IsRobotOk(const FString& RobotName);

	void PrintRobotsSeen();
	
	UFUNCTION()
	void RefreshRobotList();

	void PruneInactiveRobots();
	
	void RegisterRobotStatusSubscription();

	void RegisterRobotSubscription(FString TopicName, FString RobotName);

	UPROPERTY(BlueprintAssignable, Category = "Robofleet")
	FOnNewRobotSeen OnNewRobotSeen;

	UPROPERTY(BlueprintAssignable, Category = "Robofleet")
	FOnRobotPruned OnRobotPruned;

	//TODO: fix this terrible Idea for demo crunch. This is an extremely hacky way to avoid GC
	UFUNCTION(BlueprintCallable)
	void RemoveObjectFromRoot();
};