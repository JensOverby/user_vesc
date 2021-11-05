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

#ifndef PACKET_H
#define PACKET_H

//#include <QObject>
//#include <QTimer>
#include <string>
#include <vector>

class Packet //: public QObject
{
    //Q_OBJECT
public:
    Packet();
    ~Packet();
    void timerSlot();

    bool encode(std::vector<unsigned char>& inData, std::vector<unsigned char>& outData);
    void decode(std::vector<unsigned char>& inData, std::vector<unsigned char>& outData);

private:
    void resetState();
    unsigned short crc16(const unsigned char *buf, unsigned int len);
    //static void timerThread();
    int try_decode_packet(unsigned char *buffer, unsigned int in_len, int *bytes_left, std::vector<unsigned char>& decodedPackets);

    int mByteTimeout;
    unsigned int mMaxPacketLen;
    unsigned int mBufferLen;
    unsigned char *mRxBuffer;
    unsigned int mRxReadPtr;
    unsigned int mRxWritePtr;
    int mBytesLeft;
    int mRxTimer;
};

#endif // PACKET_H
