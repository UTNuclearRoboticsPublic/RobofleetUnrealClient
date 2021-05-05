// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"

#include "robofleet_client_lib/MessageSchedulerLib.hpp"

#include "MessageScheduler.generated.h"



/**
 * 
 */
UCLASS()
class UMessageScheduler : public UObject
{
    GENERATED_BODY()

	// TODO: eliminate raw pointer
	MessageSchedulerLib<const void*>* MSLibrary;

public:
	UMessageScheduler();
	~UMessageScheduler();

	void SetupSchedulerLib(uint64_t MaxQueueBeforeWaiting);
	void SchedulingCallback(const void* Data);
	void Scheduled(const void* Data);
	void Enqueue(
		const FString& Topic, const void* Data, double Priority,
		bool bNoDrop);
	void BackpressureUpdate(uint64_t MessageIndex, uint64_t LastPongedIndex);
	void Schedule();  
};
