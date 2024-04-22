// Fill out your copyright notice in the Description page of Project Settings.


#include "MessageScheduler.h"
#include <memory>

UMessageScheduler::UMessageScheduler() 
{
	
	UE_LOG(LogTemp, Warning, TEXT("Module loaded"));
}

UMessageScheduler::~UMessageScheduler()
{
	// TODO: eliminate delete
	delete MSLibrary;
}

void UMessageScheduler::SetupSchedulerLib(uint64_t MaxQueueBeforeWaiting)
{
	auto BoundCallbackFunction = std::bind(&UMessageScheduler::SchedulingCallback, this, std::placeholders::_1);
	MSLibrary = new  MessageSchedulerLib<const void*>(MaxQueueBeforeWaiting, BoundCallbackFunction);
}

void UMessageScheduler::SchedulingCallback(const void* Data)
{
	//Scheduled(Data);
}


