// TestServer.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include "MotionComp.h"

// Link with ws2_32.lib
#pragma comment(lib, "ws2_32")
//using namespace std;

int main() {

	MotionComp m;

	const char* multicast_ip = "239.255.50.10";
	unsigned short multicast_port = 5010;
	SOCKADDR_IN multicast_addr;
	WSADATA wsaData;
	int hr;
	BOOL bOptVal = TRUE;
	ip_mreq mreq;
	int max_length = 16;

	WSAStartup(MAKEWORD(2, 0), &wsaData);

	SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sock == INVALID_SOCKET) {
		wprintf(L"socket failed with error %d\n", WSAGetLastError());
		return 1;
	}

	// construct bind structure
	memset(&multicast_addr, 0, sizeof(multicast_addr));
	multicast_addr.sin_family = AF_INET;
	multicast_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	multicast_addr.sin_port = htons(multicast_port);

	hr = bind(sock, (struct sockaddr *) &multicast_addr, sizeof(multicast_addr));
	if (hr != 0) {
		wprintf(L"bind failed with error %d\n", WSAGetLastError());
		return 1;
	}
	/* Specify the multicast group */
	InetPton(AF_INET, _T("239.255.50.10"), &mreq.imr_multiaddr.s_addr);
	//mreq.imr_multiaddr.s_addr = inet_addr(multicast_ip);
	/* Accept multicast from any interface */
	mreq.imr_interface.s_addr = htonl(INADDR_ANY);
	/* Join the multicast address */
	hr = setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char FAR *) &mreq, sizeof(mreq));
	if (hr != 0) {
		wprintf(L"setsockopt failed with error %d\n", WSAGetLastError());
		return 1;
	}
	int optval = 8;
	hr = setsockopt(sock, IPPROTO_IP, IP_MULTICAST_TTL, (char*)&optval, sizeof(int));
	if (hr != 0) {
		wprintf(L"setsockopt failed with error %d\n", WSAGetLastError());
		return 1;
	}
	hr = setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (char *)&bOptVal, sizeof(bOptVal));
	if (hr != 0) {
		wprintf(L"setsockopt failed with error %d\n", WSAGetLastError());
		return 1;
	}

	int timeout = 1000; // 1 sec
	setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(timeout));

	int cnt = 0;
	int offset = 5;
	while (true) {
		char buffer[4096] = { 0 };	//unsigned 

		int n = recvfrom(sock, (char*)buffer, 4096, 0, NULL, 0);
		if (n == SOCKET_ERROR) {
			//wprintf(L"%d recvfrom failed with error %d\n", cnt++, WSAGetLastError());
			continue;
		}

		if (buffer[0] == '$') {
			printf("New data: ");
			
			/*m._dcsTelemetry.pitch = std::stof(&buffer[1 + 0 * offset]);
			m._dcsTelemetry.roll = std::stof(&buffer[1 + 1 * offset]);
			printf("pitch = %0.2f, roll = %0.2f \n", m._dcsTelemetry.pitch, m._dcsTelemetry.roll);
*/
			/*for (int i = 0; i < 18; i++) {
				printf("%0d: %0d, ", i, buffer[i]);
			}
			printf("\n");*/
		} 		

		printf("%s\n", buffer);
	}
	WSACleanup();
	return 0;
}