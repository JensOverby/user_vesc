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

#ifndef COMMANDS_H
#define COMMANDS_H

//#include <QObject>
//#include <QTimer>
#include <string>
#include "datatypes.h"
//#include "packet.h"
#include <vector>
#include <string.h>

class Commands //: public QObject
{
public:
    Commands();

    void processPacket(std::vector<unsigned char>& data);

    void getFwVersion(std::vector<unsigned char>& data);
    void samplePrint(debug_sampling_mode mode, int sample_len, int dec, std::vector<unsigned char>& data);
    void reboot(std::vector<unsigned char>& data);
    void sendAlive(std::vector<unsigned char>& data);
    void sendCustomAppData(std::vector<unsigned char>& appData, std::vector<unsigned char>& data);
    void customConfigGet(int confInd, bool isDefault, std::vector<unsigned char>& data);
    void customConfigSet(int confInd, std::vector<unsigned char>& confData, std::vector<unsigned char>& data);

    void timerSlot();

    static void string2vector(std::string& str, std::vector<unsigned char>& vec);

private:
    //void emitData(const char* data, unsigned int len);

    int mTimeoutFwVer;
    int mTimeoutCustomConf;
    int mTimeoutCount;

    /*
    int mTimeoutFwVer;
    int mTimeoutMcconf;
    int mTimeoutAppconf;
    int mTimeoutValues;
    int mTimeoutValuesSetup;
    int mTimeoutImuData;
    int mTimeoutDecPpm;
    int mTimeoutDecAdc;
    int mTimeoutDecChuk;
    int mTimeoutDecBalance;
    int mTimeoutPingCan;
    int mTimeoutCustomConf;
    int mTimeoutBmsVal;
	*/
};

inline void Commands::string2vector(std::string& str, std::vector<unsigned char>& vec)
{
	unsigned char* tmp = (unsigned char*)str.c_str();
	unsigned int sz = str.size();
	vec.resize(sz);
	memcpy(vec.data(), tmp, sz);
}

#endif // COMMANDS_H
