// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"

#include "WebSocketsModule.h"
#include "IWebSocket.h"
#include <vector>
#include <functional>

#include "WebsocketClient.generated.h"

/**
 * 
 */
UCLASS()
class UWebsocketClient : public UObject
{
	GENERATED_BODY()

public:
	UWebsocketClient();
	~UWebsocketClient();
	
	TSharedPtr<IWebSocket> Socket;
	std::function< void (const void*) > callback;
	bool callbackRegistered;
	std::function<void(const void*)> OnReceivedCB;
	bool verbose;

	void Initialize(FString ServerURL = TEXT("ws://localhost:8080"), FString ServerProtocol = TEXT("ws"), bool isVerbose = false);

	void Disconnect();

	UFUNCTION()
	void OnConnected();

	UFUNCTION()
	void OnClosed();
	
	UFUNCTION()
	void OnConnectionError();
	
	UFUNCTION()
	void OnMessageSent();
	
	void OnMessageReceived(const void* Data, SIZE_T Size, SIZE_T BytesRemaining);

	void Send(const void* ptr, uint32_t size, bool isBinary);

	void Ping(std::vector<char> Payload);	

	std::vector<char> GetFrameHeader(char opCode, uint32 payloadLenght, uint32 maskingKey, bool lastFrame);

	void IsCallbackRegistered(bool val);

	// TODO: Make a buffer struct and clean up the raw pointer, optimization wise the current implementation should be close to fastest but can be organized a bit better and made safer.
	char* DataBuffer;
	int PrevSize;
	bool bIsBuffering = false;
};
