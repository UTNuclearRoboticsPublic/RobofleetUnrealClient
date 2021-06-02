// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"

#include "WebSocketsModule.h"
#include "IWebSocket.h"
#include <vector>

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


	void Initialize(FString ServerURL = TEXT("ws://localhost:8080"), FString ServerProtocol = TEXT("ws"));
	void OnConnected();
	void OnConnectionError();
	void OnMessageSent();
	void OnMessageReceived(const void* Data, SIZE_T Size, SIZE_T BytesRemaining);

	void Send(const void* ptr, uint32_t size, bool isBinary);

	void Ping(std::vector<char> Payload);	

	std::vector<char> GetFrameHeader(char opCode, uint32 payloadLenght, uint32 maskingKey, bool lastFrame);
};
