//============================================================================
// Name        : cmate_client.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <string>
#include <unistd.h>
//#include <stdio.h>


#include "SerialPort.hpp"
using namespace mn::CppLinuxSerial;

#include "commands.h"
#include "packet.h"

typedef enum {EXCLUSIVE_WRITE_MODE, STANDARD_MODE} ConsoleMode;
ConsoleMode consoleMode = STANDARD_MODE;


static void *heartBeatThread(void *threadarg)
{
	SerialPort* serialPort = (SerialPort*)threadarg;
	Commands commands;
	Packet packet;
	std::vector<unsigned char> cmd;
	std::vector<unsigned char> coded;
	commands.sendAlive(cmd);
	packet.encode(cmd, coded);
	cmd.clear();

	for (;;)
	{
		usleep(100000);
		serialPort->Write(coded);
	}
}

static void *timerThread(void *threadarg)
{
	SerialPort* serialPort = (SerialPort*)threadarg;
	std::vector<unsigned char> readData;
	std::vector<unsigned char> stm32decoded;
	Commands commands;
	Packet packet;

	for (;;)
	{
		readData.clear();
		stm32decoded.clear();

		// Read data
		for (;;) {
			if (serialPort->Read(readData)) {
				packet.decode(readData, stm32decoded);
				readData.clear();
				if (!stm32decoded.empty())
					break;
			}
		}

		if (consoleMode == STANDARD_MODE)
			commands.processPacket(stm32decoded);

		//usleep(1000);
	}
}


int main()
{
	// Create serial port object and open serial port
	std::string serial_port_name = "/dev/serial/by-id/usb-STMicroelectronics_ChibiOS_RT_Virtual_COM_Port_304-if00";
	SerialPort serialPort(serial_port_name, BaudRate::B_115200);
	serialPort.SetTimeout(-1); // Block when reading until any data is received
	serialPort.Open();

    pthread_t thread;
	int rc = pthread_create(&thread, NULL, timerThread, (void*)&serialPort);
	if (rc) {
		  std::cout << "Error:unable to create thread," << rc << std::endl;
		  exit(-1);
	}

	// Call for vesc firmware version
	Commands commands;
	std::vector<unsigned char> cmd;
	commands.getFwVersion(cmd);
	Packet packet;
	std::vector<unsigned char> coded;
	packet.encode(cmd, coded);
	serialPort.Write(coded);
	cmd.clear();
	coded.clear();
	std::vector<unsigned char> appData;

	// Start Heartbeat Thread
    pthread_t thread_beat;
	rc = pthread_create(&thread_beat, NULL, heartBeatThread, (void*)&serialPort);
	if (rc) {
		  std::cout << "Error:unable to create thread," << rc << std::endl;
		  exit(-1);
	}

	usleep(1000000);

	int confInd = 0;
	commands.customConfigGet(confInd, false, cmd);
	packet.encode(cmd, coded);
	serialPort.Write(coded);
	cmd.clear();
	coded.clear();

	for (std::string line; std::getline(std::cin, line);) {
		if (line.empty()) {
			if (consoleMode == EXCLUSIVE_WRITE_MODE) {
				consoleMode = STANDARD_MODE;
				std::cout << "Outputting..." << std::endl;
			}
			else {
				consoleMode = EXCLUSIVE_WRITE_MODE;
				std::cout << "Command:" << std::endl;
			}
			//consoleMode = (ConsoleMode)(((int)consoleMode+1) % 2);
			continue;
		}
		else {
			if (consoleMode == EXCLUSIVE_WRITE_MODE) {
				consoleMode = STANDARD_MODE;
				std::cout << "Outputting..." << std::endl;
			}
		}

		//char* tmp1 = (unsigned char*)line.c_str();
		//unsigned int tmp3 = line.size();
		//appData = std::vector<unsigned char>( *tmp1, tmp3 );



		//appData = std::vector<unsigned char>( *(unsigned char*)line.c_str(), line.size() );
		Commands::string2vector(line, appData);
		commands.sendCustomAppData(appData, cmd);
		packet.encode(cmd, coded);
		serialPort.Write(coded);
		cmd.clear();
		coded.clear();

	    //std::cout << line << std::endl;
	}

/*

	Commands commands;
	std::vector<unsigned char> strCmd;
	commands.getFwVersion(strCmd);

	Packet packet;
	std::vector<unsigned char> coded;
	packet.encode(strCmd, coded);







	std::string serial_port_name = "/dev/serial/by-id/usb-STMicroelectronics_ChibiOS_RT_Virtual_COM_Port_304-if00";

	// Create serial port object and open serial port
	SerialPort serialPort(serial_port_name, BaudRate::B_115200);
	// Use SerialPort serialPort("/dev/ttyACM0", 13000); instead if you want to provide a custom baud rate
	serialPort.SetTimeout(-1); // Block when reading until any data is received
	serialPort.Open();

	// Write some ASCII data
	serialPort.Write(coded);

	// Read some data back (will block until at least 1 byte is received due to the SetTimeout(-1) call above)
	std::vector<unsigned char> readData;
	std::vector<unsigned char> stm32decoded;
	while (true) {
		if (serialPort.Read(readData)) {
			packet.decode(readData, stm32decoded);
			readData.clear();
			if (!stm32decoded.empty())
				break;
		}
	}

	commands.processPacket(stm32decoded);
*/

	// Close the serial port
	serialPort.Close();

	return 0;
}
