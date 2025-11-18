#pragma once

#include "../Protocol.h"

#include <thread>
#include <set>
#include <mutex>

class ServerTrackedDeviceProvider;

class IPCServer
{
public:
	IPCServer(ServerTrackedDeviceProvider *driver) : driver(driver) { }
	~IPCServer();

	void Run();
	void Stop();

private:
	void HandleRequest(const protocol::Request &request, protocol::Response &response);

	struct SocketInstance
	{
		int clientSocket;
		IPCServer *server;

		protocol::Request request;
		protocol::Response response;
	};

	SocketInstance *CreateSocketInstance(int fd);
	void CloseSocketInstance(SocketInstance *pipeInst);

	static void RunThread(IPCServer *_this);

	std::thread mainThread;

	bool running = false;
	bool stop = false;
	int stopRead = -1;
	int stopWrite = -1;

	ServerTrackedDeviceProvider *driver;
};
