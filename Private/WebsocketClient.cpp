// Fill out your copyright notice in the Description page of Project Settings.


#include "WebsocketClient.h"

DECLARE_LOG_CATEGORY_EXTERN(LogRobofleet, Log, All);

//DEFINE_LOG_CATEGORY(LogRobofleet);

UWebsocketClient::UWebsocketClient()
{
	
}

UWebsocketClient::~UWebsocketClient()
{
	callbackRegistered = false;
	Disconnect();
	UE_LOG(LogRobofleet, Warning, TEXT("Websocket Client destroyed"));	
}

void UWebsocketClient::Initialize(FString ServerURL /*= TEXT("ws://localhost:8080")*/, FString ServerProtocol /*= TEXT("ws")*/, bool isVerbose /*= false*/) 
{
	UE_LOG(LogRobofleet, Log, TEXT("Robofleet websocket is initializing with Server URL: %s, and Protocol: %s"), *ServerURL, *ServerProtocol);
	Socket = FWebSocketsModule::Get().CreateWebSocket(ServerURL, ServerProtocol);
	UE_LOG(LogRobofleet, Log, TEXT("Robofleet websocket is created"));

	// Bind Socket functions
	Socket->OnConnected().AddUFunction(this, FName("OnConnected"));
	Socket->OnConnectionError().AddUFunction(this, FName("OnConnectionError"));
	Socket->OnMessageSent().AddUFunction(this, FName("OnMessageSent"));
	Socket->OnClosed().AddUFunction(this, FName("OnClosed"));

	// Unreal having a problem with binding to a raw function, and a UFUNCTION doesn't like having a void* as a arg
	// For now, use lambda directly
	Socket->OnRawMessage().AddLambda([this](const void* Data, SIZE_T Size, SIZE_T BytesRemaining) -> void {
		OnMessageReceived(Data, Size, BytesRemaining);
	});
		
	Socket->Connect();
	UE_LOG(LogRobofleet, Log, TEXT("Websocket Client Initialized"));
}

void UWebsocketClient::Disconnect() {
	if(Socket != NULL)
		Socket->Close();
	UE_LOG(LogRobofleet, Log, TEXT("Websocket Disconnected"));
}

void UWebsocketClient::IsCallbackRegistered(bool val)
{
	callbackRegistered = val;
}
void UWebsocketClient::OnConnected()
{
	UE_LOG(LogTemp, Log, TEXT("Connected to websocket."))
}

void UWebsocketClient::OnClosed()
{
	UE_LOG(LogTemp, Log, TEXT("Websocket Closed."))
}

void UWebsocketClient::OnConnectionError()
{
	UE_LOG(LogTemp, Warning, TEXT("Encountered error while trying to connect to websocket."))
}

void UWebsocketClient::OnMessageReceived(const void* Data, SIZE_T Size, SIZE_T BytesRemaining)
{
	UE_LOG(LogTemp, Verbose, TEXT("Message Received"));
	// TODO: fix it frank, actually its not too terrible atm.
	
	if (BytesRemaining)
	{
		if (!bIsBuffering)
		{
			DataBuffer = new char[Size + BytesRemaining];
			memcpy(DataBuffer, Data, Size);
			PrevSize = Size;
			bIsBuffering = true;
		}
		else
		{
			memcpy(DataBuffer+PrevSize, Data, Size);
			PrevSize = PrevSize + Size;
		}
	}
	else
	{
		if (bIsBuffering)
		{
			memcpy(DataBuffer + PrevSize, Data, Size);
			PrevSize = 0;
			bIsBuffering = false;
			OnReceivedCB(DataBuffer);
			delete DataBuffer;
		}
		else
		{
			OnReceivedCB(Data);
		}
	}
}

void UWebsocketClient::OnMessageSent()
{
	UE_LOG(LogTemp, Verbose, TEXT("Message Sent"));
}

void UWebsocketClient::Send(const void* ptr, uint32_t size, bool isBinary)
{
	UE_LOG(LogTemp, Verbose, TEXT("Message Sending"));
	Socket->Send(ptr, size, isBinary);
}

























void UWebsocketClient::Ping(std::vector<char> Payload)
{
	bool m_mustMask = false;
	// make sure payload is < 125
	//QByteArray payloadTruncated = payload.left(125);
	
	//m_pingTimer.restart();
	uint32 maskingKey = 0;
	//if (m_mustMask)
		//maskingKey = generateMaskingKey();
	std::vector<char> pingFrame = GetFrameHeader(0x9, Payload.size(), 0, false);
	//if (m_mustMask)
		//QWebSocketProtocol::mask(&payloadTruncated, maskingKey);
	//pingFrame.append(payloadTruncated);

	std::vector<char> CompleteFrame = pingFrame;
	CompleteFrame.insert(CompleteFrame.end(), Payload.begin(), Payload.end());


	Socket->Send(TEXT("isthisping"));
	//Socket->Send(CompleteFrame.data());
	//Socket->Send(CompleteFrame.data());
}


std::vector<char> UWebsocketClient::GetFrameHeader(char opCode, uint32 payloadLength, uint32 maskingKey, bool lastFrame)
{
	std::vector<char> Header;
	//int HeaderIndex = 0;
	//int payloadLength = 125;
	//int maskingKey = 0;

	//FIN, RSV1-3, opcode (RSV-1, RSV-2 and RSV-3 are zero)
	uint8 byte = static_cast<uint8>((opCode & 0x0F) | (lastFrame ? 0x80 : 0x00));
	Header.push_back(static_cast<char>(byte));

	byte = 0x00;
	if (maskingKey != 0)
		byte |= 0x80;
	if (payloadLength <= 125) {
		byte |= static_cast<uint8>(payloadLength);
		Header.push_back(static_cast<char>(byte));
	}
	/*
	else if (payloadLength <= 0xFFFFU) {
		byte |= 126;
		header.append(static_cast<char>(byte));
		quint16 swapped = qToBigEndian<quint16>(static_cast<quint16>(payloadLength));
		header.append(static_cast<const char*>(static_cast<const void*>(&swapped)), 2);
	}
	else if (payloadLength <= 0x7FFFFFFFFFFFFFFFULL) {
		byte |= 127;
		header.append(static_cast<char>(byte));
		quint64 swapped = qToBigEndian<quint64>(payloadLength);
		header.append(static_cast<const char*>(static_cast<const void*>(&swapped)), 8);
	}

	if (maskingKey != 0) {
		const quint32 mask = qToBigEndian<quint32>(maskingKey);
		header.append(static_cast<const char*>(static_cast<const void*>(&mask)),
			sizeof(quint32));
	}
	*/
	return Header;
}


/*
void UWebsocketClient::Ping()
{
	uint8 OpCode = 0x9;
	unsigned char header[20];
	bool LastFrame = true;
	uint8 qbyte = static_cast<uint8>((OpCode & 0x0F) | (LastFrame ? 0x80 : 0x00));
	header[0] = static_cast<char>(qbyte);
	header[1] = static_cast<char>(1);
	header[2] = static_cast<char>(2);
	header[3] = static_cast<char>(3);

	Socket->Send(header, 20, true);
}
*/




/*
namespace QWebSocketProtocol
{
	enum OpCode
	{
		OpCodeContinue = 0x0,
		OpCodeText = 0x1,
		OpCodeBinary = 0x2,
		OpCodeReserved3 = 0x3,
		OpCodeReserved4 = 0x4,
		OpCodeReserved5 = 0x5,
		OpCodeReserved6 = 0x6,
		OpCodeReserved7 = 0x7,
		OpCodeClose = 0x8,
		OpCodePing = 0x9,
		OpCodePong = 0xA,
		OpCodeReservedB = 0xB,
		OpCodeReservedC = 0xC,
		OpCodeReservedD = 0xD,
		OpCodeReservedE = 0xE,
		OpCodeReservedF = 0xF
	};

{
	void QWebSocketPrivate::ping(const QByteArray & payload)
	{
		QByteArray payloadTruncated = payload.left(125);
		m_pingTimer.restart();
		quint32 maskingKey = 0;
		if (m_mustMask)
			maskingKey = generateMaskingKey();
		QByteArray pingFrame = getFrameHeader(QWebSocketProtocol::OpCodePing,
			quint64(payloadTruncated.size()),
			maskingKey, true);
		if (m_mustMask)
			QWebSocketProtocol::mask(&payloadTruncated, maskingKey);
		pingFrame.append(payloadTruncated);
		qint64 ret = writeFrame(pingFrame);
		Q_UNUSED(ret);
	}
}
*/
