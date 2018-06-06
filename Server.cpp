#include "Server.h"
#include <thread>
#include "lib/rapidjson/document.h"
#include "lib/rapidjson/stringbuffer.h"
#include "lib/main.h"

using namespace rapidjson;

std::clock_t Server::lastSentMessage;

std::clock_t Server::getLastSentTime()
{
	return lastSentMessage;
}

Server::Server(unsigned int port) :
	sendOutputs(false),
	bytesRead(0),
	recvMessageLen(0),
	sendMessageLen(0),
	readyToSend(false),
	nonBlockFlag(false),
	sentFlag(0x0),
	allMsgFlag(0x0)
{
	struct sockaddr_in server;
	freopen("deepgtav.log", "w", stdout);

	printf("\nInitializing Winsock...");
	if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
		printf("Failed. Error Code: %d", WSAGetLastError());
	}
	printf("Initialized.\n");

	if ((ServerSocket = socket(AF_INET, SOCK_STREAM, 0)) == INVALID_SOCKET) {
		printf("Could not create socket: %d", WSAGetLastError());
	}
	printf("Socket created.\n");

	server.sin_family = AF_INET;
	server.sin_addr.s_addr = INADDR_ANY;
	server.sin_port = htons(port);

	if (bind(ServerSocket, (struct sockaddr *)&server, sizeof(server)) == SOCKET_ERROR) {
		printf("Bind failed with error code: %d", WSAGetLastError());
	}
	printf("Bind done.\n");

	printf("Listening...\n");
	if (listen(ServerSocket, 1) == SOCKET_ERROR) {
		printf("Could not listen: %d", WSAGetLastError());
	}

	if (ioctlsocket(ServerSocket, FIONBIO, &iMode) != NO_ERROR) {
		printf("Server ioctlsocket failed");
	}
}

void Server::checkClient(){
	//The socket can't set to block or use select, because it will block GTA5.
	//The server serves only one client, no neccesary for a rapid monitoring. Therefore: 
	scriptWait(1000);
	SOCKET tmpSocket = SOCKET_ERROR;
	tmpSocket = accept(ServerSocket, NULL, NULL);
	if (tmpSocket != SOCKET_ERROR) {
		printf("Connection accepted.\n");
		ClientSocket = tmpSocket;
		if (ioctlsocket(ClientSocket, FIONBIO, &iMode) != NO_ERROR) {
			printf("Client ioctlsocket failed");
			return;
		}
		clientConnected = true;
	}
}

void Server::checkRecvMessage() {
	int result;
	Document d;
	int error;
	
	if (recvMessageLen == 0) {
		result = recv(ClientSocket, (char*)&recvMessageLen, 4, 0); //Receive message len first
		if (result == 0) {
			printf("\nClient closed");
			resetState();
			return;
		}
		error = WSAGetLastError();
		if (error == WSAEWOULDBLOCK) return;
		if (error != 0) {
			printf("\nError receiving message length: %d", error);
			resetState();
			return;
		}
	}

	while (bytesRead < recvMessageLen){
		result = recv(ClientSocket, json + bytesRead, recvMessageLen - bytesRead, 0);
		if (result == 0) {
			printf("\nClient closed");
			resetState();
			return;
		}
		error = WSAGetLastError();
		if (error == WSAEWOULDBLOCK) return;
		if (error != 0 || result < 1) {
			printf("\nError receiving message: %d", error);
			resetState();
			return;
		}
		bytesRead = bytesRead + result;
	}

	json[bytesRead] = '\0';
	bytesRead = 0;
	recvMessageLen = 0;

	d.Parse(json);

	if (d.HasMember("commands")) {
		printf("Commands received\n");
		const Value& commands = d["commands"];
		scenario.setCommands(commands["throttle"].GetFloat(), commands["brake"].GetFloat(), commands["steering"].GetFloat());
	}
	else if (d.HasMember("config")) {
		//Change the message values and keep the others the same
		printf("Config received\n");
		const Value& config = d["config"];
		const Value& sc = config["scenario"];
		const Value& dc = config["dataset"];
		scenario.config(sc, dc);
		addSendItems();
	}
	else if (d.HasMember("start")) {
		//Set the message values and randomize the others. Start sending the messages
		printf("Start received\n");
		const Value& config = d["start"];
		const Value& sc = config["scenario"];
		const Value& dc = config["dataset"];
		scenario.start(sc, dc);
		addSendItems();
		sendOutputs = true;
	}
	else if (d.HasMember("stop")) {
		//Stop sendig messages, keep client connected
		printf("Stop received\n");
		sendOutputs = false;
		scenario.stop();
	}
	else {
		return; //Invalid message
	}
}

void Server::checkSendMessage() {
	int itemFlag = 0x0;
	bool isTimesup;

	if (sentFlag == allMsgFlag) {
		//printf("\nTime last from finishing sending, receive messages, to scenario.run, then it reaches here, about 0.015s=%f", 
		//	(float)(std::clock() - lastSentMessage) / CLOCKS_PER_SEC);

		//with i7-7700k, GTX1080Ti, 16GB RAM:
		//generateMessage without lidar: 0.006s, receive messages, scenario.run: 0.015
		//generateMessage with scaled lidar(non-visualized, 1000 points): 0.036, receive messages, scenario.run: 0.013
		scenario.generateMessage();
		
		//printf("\nTime=%f", 
		//	(float)(std::clock() - lastSentMessage) / CLOCKS_PER_SEC);

		sentFlag = 0x0;
	}

	isTimesup = (((float)(std::clock() - lastSentMessage) / CLOCKS_PER_SEC) > (1.0 / scenario.rate));
	if (sendOutputs && (isTimesup || nonBlockFlag)) {

		//if (!nonBlockFlag) printf("\nSendlast=%f", (float)(std::clock() - lastSentMessage) / CLOCKS_PER_SEC);//20Hz: 0.063s

		//begin:		itemFlag==000, sentFlag==000, allMsgFlag==111
		//frame:		itemFlag==000, sentFlag==001
		//lidar:		itemFlag==001, sentFlag==011
		//message:		itemFlag==011, sentFlag==111
		for (int i = _sendItemList.size(), j = 0; i > 0; --i, ++j) {
			itemFlag = allMsgFlag >> i;
			if (sentFlag == itemFlag) {
				if (sending(*_sendItemList[j]._pBuf, _sendItemList[j]._len, _sendItemList[j]._name.data())) {
					sentFlag |= 0x1 << j;
					nonBlockFlag = false;
				}
				else {
					nonBlockFlag = true;
					break;
				}
			}
		}
		if (!nonBlockFlag)
			lastSentMessage = std::clock();
	}	
}

void Server::resetState() {
	shutdown(ClientSocket, SD_SEND);
	closesocket(ClientSocket);

	clientConnected = false;
	sendOutputs = false;
	bytesRead = 0;
	recvMessageLen = 0;
	sendMessageLen = 0;
	readyToSend = false;
	nonBlockFlag = false;
	sentFlag = 0x0;
	allMsgFlag = 0x0;

	scenario.stop();
}

bool Server::sending(const char * buf, unsigned int * len, const char * typeName)
{
	int error;
	int r;
	if (!readyToSend) {
		r = send(ClientSocket, (const char*)len, sizeof(*len), 0);
		if (r == 0) {
			printf("\nClient closed");
			resetState();
			return false;
		}
		error = WSAGetLastError();
		if (error == WSAEWOULDBLOCK) return false;
		if (error != 0) {
			printf("\nError sending %s length: %d", typeName, error);
			resetState();
			return false;
		}
		readyToSend = true;
		sendMessageLen = 0;
	}

	while (readyToSend && (sendMessageLen < *len)) {
		r = send(ClientSocket, (const char*)(buf + sendMessageLen), *len - sendMessageLen, 0);
		if (r == 0) {
			printf("\nClient closed");
			resetState();
			return false;
		}
		error = WSAGetLastError();
		if (error == WSAEWOULDBLOCK) return false;
		if (error != 0 || r <= 1) {
			printf("\nError sending %s: %d", typeName, error);
			resetState();
			return false;
		}
		sendMessageLen = sendMessageLen + r;
	}
	readyToSend = false;
	return true;
}

void Server::addSendItems()
{
	int i = 0;
	_sendItemList.clear();
	sentFlag = 0x0;
	allMsgFlag = 0x0;
	nonBlockFlag = false;

	if (scenario.screenCapturer.isActivate) {
		_sendItemList.push_back(SendItem((const char **)&scenario.screenCapturer.pixels, &scenario.screenCapturer.length, "frame"));
		allMsgFlag |= 0x1 << i;
		++i;
	}
	if (scenario.liDARDevice._initType != LiDAR::LIDAR_NOT_INIT_YET) {
		_sendItemList.push_back(SendItem((const char **)&scenario.liDARDevice._pointClouds, &scenario.liDARDevice._lenght, "lidar"));
		allMsgFlag |= 0x1 << i;
		++i;
	}
	_sendItemList.push_back(SendItem(&scenario.chmessage, &scenario.messageSize, "message"));
	allMsgFlag |= 0x1 << i;
	sentFlag = allMsgFlag;

	lastSentMessage = std::clock();
}


