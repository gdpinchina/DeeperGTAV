#pragma once

#include "Scenario.h"

#include <WinSock2.h>
#include <WS2tcpip.h>

// Need to link with Ws2_32.lib
#pragma comment (lib, "ws2_32")

class Server {
private:
	struct SendItem {
		SendItem() {  }
		SendItem(const char **pBuf, unsigned int *len, const char *name) {
			_pBuf = pBuf;
			_len = len;
			_name = name;
		}
		const char **_pBuf;
		unsigned int *_len;
		std::string _name;
	};

	WSADATA wsaData;
	u_long iMode = 1; //non-blocking socket
	SOCKET ServerSocket = INVALID_SOCKET;
	SOCKET ClientSocket = INVALID_SOCKET;

	bool sendOutputs;
	int bytesRead;
	int recvMessageLen;
	int sendMessageLen;
	bool readyToSend;
	bool nonBlockFlag;

	char json[4096];
	unsigned int sentFlag;
	unsigned int allMsgFlag;
	static std::clock_t lastSentMessage;

	void resetState();
	bool sending(const char *buf, unsigned int *len, const char *typeName);
	void addSendItems();

	std::vector<SendItem> _sendItemList;

public:
	bool clientConnected = false;
	Scenario scenario;

	static std::clock_t getLastSentTime();

	Server(unsigned int port);
	void checkRecvMessage();
	void checkSendMessage();
	void checkClient();
};