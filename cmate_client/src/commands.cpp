/*
    Copyright 2016 - 2019 Benjamin Vedder	benjamin@vedder.se

    This file is part of VESC Tool.

    VESC Tool is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    VESC Tool is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "commands.h"
//#include <QDebug>
//#include <QEventLoop>

//#include <cstdlib>
//#include <pthread.h>
#include <unistd.h>
//#include <stdio.h>
//#include <string.h>
#include <iostream>
#include <sstream>

static void *timerThread(void *threadarg)
{
	Commands* comObj = (Commands*)threadarg;

	for (;;)
	{
		usleep(1000);
		comObj->timerSlot();
	}
}

Commands::Commands() //QObject *parent) : QObject(parent)
{
    //mTimer = new QTimer(this);
    //mTimer->setInterval(10);
    //mTimer->start();

    mTimeoutCount = 100;
    mTimeoutFwVer = 0;
    mTimeoutCustomConf = 0;

    //connect(mTimer, SIGNAL(timeout()), this, SLOT(timerSlot()));

    /*pthread_t thread;
	int rc = pthread_create(&thread, NULL, timerThread, (void*)this);
	if (rc) {
		  std::cout << "Error:unable to create thread," << rc << std::endl;
		  exit(-1);
	}*/
}

void Commands::timerSlot()
{
    if (mTimeoutFwVer > 0) mTimeoutFwVer--;
    if (mTimeoutCustomConf > 0) mTimeoutCustomConf--;
}

void Commands::processPacket(std::vector<unsigned char>& data)
{
    //std::string vb(data);
    COMM_PACKET_ID id = (COMM_PACKET_ID)data.front(); data.erase(data.begin()); // atoi(&vb.front()); vb.erase(0,1);//  vb.vbPopFrontUint8());

    switch (id) {
    case COMM_FW_VERSION: {
        mTimeoutFwVer = 0;
        FW_RX_PARAMS params;

        if (data.size() >= 2) {
            params.major = data.front(); data.erase(data.begin());
            params.minor = data.front(); data.erase(data.begin());
            params.hw = (char*)data.data();
            data.erase(data.begin(), data.begin()+params.hw.size()+1);
        }

        if (data.size() >= 12) {
            params.uuid = (char*)data.data();
            data.erase(data.begin(), data.begin()+params.uuid.size()+1);
        }

        if (data.size() >= 1) {
            params.isPaired = data.front(); data.erase(data.begin());
        }

        if (data.size() >= 1) {
            params.isTestFw = data.front(); data.erase(data.begin());
        }

        if (data.size() >= 1) {
            params.hwType = HW_TYPE( data.front() ); data.erase(data.begin());
        }

        if (data.size() >= 1) {
            params.customConfigNum = data.front(); data.erase(data.begin());
        }

        std::cout << "HW Version " << params.major << "." << params.minor << " hw" << params.hw << std::endl;
        std::cout << "uuid " << params.uuid << std::endl;

        //emit fwVersionReceived(params); OBS OBS OBS!!
    } break;

    case COMM_PRINT:
        std::cout << "COMM_PRINT: " << (char*)data.data() << std::endl;
    	//emit printReceived(vb);
        break;

    case COMM_SAMPLE_PRINT:
        std::cout << "COMM_SAMPLE_PRINT" << (char*)data.data() << std::endl;
        //emit samplesReceived(vb);
        break;

    case COMM_CUSTOM_APP_DATA:
    {
    	char* cstr = (char*)data.data();
    	cstr[data.size()] = '\0';
    	//std::string str = cstr;
        std::cout << "COMM_CUSTOM_APP_DATA: " << cstr << std::endl;
        //emit customAppDataReceived(vb);
    }
        break;

    case COMM_SET_CUSTOM_CONFIG:
        std::cout << "COMM_SET_CUSTOM_CONFIG" << (char*)data.data() << std::endl;
        //emit ackReceived("COMM_SET_CUSTOM_CONFIG Write OK");
        break;

    case COMM_GET_CUSTOM_CONFIG:
    case COMM_GET_CUSTOM_CONFIG_DEFAULT: {
        mTimeoutCustomConf = 0;
        int confInd = (int)data.front(); data.erase(data.begin());
        std::cout << "COMM_GET_CUSTOM_CONFIG" << confInd << ", " << std::endl;
        //emit customConfigRx(confInd, vb);
    } break;

    default:
        break;
    }
}

void Commands::getFwVersion(std::vector<unsigned char>& data)
{
    if (mTimeoutFwVer > 0) {
        return;
    }

    mTimeoutFwVer = mTimeoutCount;

    data.push_back( (unsigned char)COMM_FW_VERSION ); //VByteArray vb;
    //vb.append((char)COMM_FW_VERSION); //.vbAppendInt8(COMM_FW_VERSION);
    //emitData(&vb, 1);
}

void Commands::samplePrint(debug_sampling_mode mode, int sample_len, int dec, std::vector<unsigned char>& data)
{
    //char vb[5];
    data.push_back( (unsigned char)COMM_SAMPLE_PRINT );
    data.push_back( (unsigned char)mode );
    data.push_back( (unsigned char)sample_len );
    data.push_back( (unsigned char)dec );
    //emitData(vb, 5);
}

void Commands::reboot(std::vector<unsigned char>& data)
{
	data.push_back( (unsigned char)COMM_REBOOT );
    //emitData(&vb, 1);
}

void Commands::sendAlive(std::vector<unsigned char>& data)
{
	data.push_back( (unsigned char)COMM_ALIVE );
    //emitData(&vb, 1);
}

void Commands::sendCustomAppData(std::vector<unsigned char>& appData, std::vector<unsigned char>& data)
{
    //std::string vb;
	data.push_back( (unsigned char)COMM_CUSTOM_APP_DATA );
	data.insert(data.end(), appData.begin(), appData.end());
    //emitData(vb.c_str(), vb.size());
}

void Commands::customConfigGet(int confInd, bool isDefault, std::vector<unsigned char>& data)
{
    if (mTimeoutCustomConf > 0) {
        return;
    }

    mTimeoutCustomConf = mTimeoutCount;

    //char vb[2];
    if (isDefault)
    	data.push_back( (unsigned char)COMM_GET_CUSTOM_CONFIG_DEFAULT );
    else
    	data.push_back( (unsigned char)COMM_GET_CUSTOM_CONFIG );
    data.push_back( (unsigned char)confInd );
    //emitData(vb, 2);
}

void Commands::customConfigSet(int confInd, std::vector<unsigned char>& confData, std::vector<unsigned char>& data)
{
    //std::string vb;
	data.push_back( (unsigned char)COMM_SET_CUSTOM_CONFIG );
	data.push_back( (unsigned char)confInd );
	data.insert(data.end(), confData.begin(), confData.end());
    //emitData(vb.c_str(), vb.size());
}

/*void Commands::emitData(const char* data, unsigned int len)
{
    //emit dataToSend(data); OBS OBS OBS!!!
}*/
