#define S_FUNCTION_NAME  gas_monitor_sfun
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include <winsock2.h>
#include <ws2tcpip.h>
#include <iostream>
#include <string>
#include <sstream>

#pragma comment(lib, "Ws2_32.lib")

#define PORT 5005
#define BUFFER_SIZE 1024

static SOCKET clientSocket;
static bool simulationMode = false; // Fallback mode flag

// ---------- Helper: Parse Data ----------
void parseData(const std::string &data, double *outputs) {
    std::stringstream ss(data);
    std::string token;
    int i = 0;

    while (std::getline(ss, token, ',') && i < 4) {
        outputs[i++] = atof(token.c_str());
    }

    // Apply thresholds
    outputs[4] = (outputs[0] > 200)     ? 1.0 : 0.0;  // CO > 200 ppm
    outputs[5] = (outputs[1] > 5000)    ? 1.0 : 0.0;  // CO2 > 5000 ppm
    outputs[6] = (outputs[2] < 19.5)    ? 1.0 : 0.0;  // O2 < 19.5%
    outputs[7] = (outputs[3] > 50000)   ? 1.0 : 0.0;  // CH4 > 50,000 ppm
}

// ---------- Simulation Fallback ----------
void generateDummyData(double *outputs) {
    outputs[0] = 150;     // CO (ppm)
    outputs[1] = 6000;    // CO2 (ppm)
    outputs[2] = 18.8;    // O2 (%)
    outputs[3] = 52000;   // CH4 (ppm)

    outputs[4] = (outputs[0] > 200)     ? 1.0 : 0.0;
    outputs[5] = (outputs[1] > 5000)    ? 1.0 : 0.0;
    outputs[6] = (outputs[2] < 19.5)    ? 1.0 : 0.0;
    outputs[7] = (outputs[3] > 50000)   ? 1.0 : 0.0;
}

// ---------- Initialize ----------
static void mdlInitializeSizes(SimStruct *S) {
    ssSetNumSFcnParams(S, 0);

    ssSetNumInputPorts(S, 0);
    ssSetNumOutputPorts(S, 8); // 4 gas values + 4 alerts

    for (int i = 0; i < 8; i++) {
        ssSetOutputPortWidth(S, i, 1);
    }

    WSADATA wsaData;
    WSAStartup(MAKEWORD(2, 2), &wsaData);

    SOCKET serverSocket = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in serverAddr = {};
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(PORT);
    serverAddr.sin_addr.s_addr = INADDR_ANY;

    bind(serverSocket, (sockaddr*)&serverAddr, sizeof(serverAddr));
    listen(serverSocket, 1);

    std::cout << "Waiting for Raspberry Pi on port " << PORT << "...\n";
    clientSocket = accept(serverSocket, NULL, NULL);

    if (clientSocket == INVALID_SOCKET) {
        std::cout << "No client connected, switching to SIMULATION MODE.\n";
        simulationMode = true;
    } else {
        std::cout << "Connected to Raspberry Pi.\n";
    }

    closesocket(serverSocket);
}

static void mdlOutputs(SimStruct *S, int_T tid) {
    double *y[8];
    for (int i = 0; i < 8; i++) {
        y[i] = ssGetOutputPortRealSignal(S, i);
    }

    double outputs[8] = {0};

    if (simulationMode) {
        generateDummyData(outputs);
    } else {
        char buffer[BUFFER_SIZE] = {0};
        int bytesReceived = recv(clientSocket, buffer, BUFFER_SIZE - 1, 0);

        if (bytesReceived > 0) {
            buffer[bytesReceived] = '\0';
            parseData(std::string(buffer), outputs);
        } else {
            std::cout << "Connection lost. Switching to SIMULATION MODE.\n";
            simulationMode = true;
            generateDummyData(outputs);
        }
    }

    for (int i = 0; i < 8; i++) {
        *y[i] = outputs[i];
    }
}

static void mdlTerminate(SimStruct *S) {
    if (!simulationMode) {
        closesocket(clientSocket);
    }
    WSACleanup();
}

#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif