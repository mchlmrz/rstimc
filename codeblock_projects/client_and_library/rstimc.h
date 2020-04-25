/*
    This is rstimc, a library to command in real-time the RehaStim 
	electrical stimulator from Hasomed. The copyright holder has 
	no direct contact with Hasomed, and all the rights about the
	stimulator belongs to Hasomed.
	
    Copyright (C) 2018  Michele Marazzi <michele[dot]marazzi[dot]it[at]gmail[dot]com>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
	
	See LICENSE for details.
*/


#ifndef RSTIMC_H
#define RSTIMC_H

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <string>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <errno.h>
#include <sys/file.h>

/*
 * Whenever we start reading we should start a timer one shot to check if we get a response
 */

// Serial Settings
#define BAUDRATE_REHA B115200
#define PARITY_REHA false
#define DATABITS_REHA 8
#define STOPBIT_REHA 2
#define FLOWC_RTSCTS true

// Technical details
#define CURRENT_MAX 	126
#define CURRENT_STEP 	2
#define PULSEWDITH_MAX 	500
#define PULSEWIDTH_STEP 1

// COMMANDS
#define CHANNEL_LIST_MODE_INIT 		1
#define CHANNEL_LIST_MODE_UPDATE 	2
#define CHANNEL_LIST_MODE_STOP 		3
#define SINGLE_PULSE_GENERATION 	4

// WARNING, THE FOLLOWING DEPENDS ON THE SYSTEM ARCHITECTURE!!
// michele@michele-VirtualBox:~$ lscpu | grep Endian
// Byte Order:          Little Endian
struct AckStim {
    unsigned char Error_Code : 1;
	unsigned char : 5;
	unsigned char Ident : 2;
};
// ACK by the stimulator

struct SinglePulseGenerationCommand {
	unsigned char Pulse_Current : 7;
	unsigned char zero2 : 1;
	// Byte 4
	unsigned char Pulse_Width_L : 7;
	unsigned char zero1 : 1;
	// Byte 3
	unsigned char Pulse_Width_H : 2;
	unsigned char : 2;
	unsigned char Channel_Number : 3;
	unsigned char zero0 : 1;
	// Byte 2
	unsigned char Check : 5;
	unsigned char Ident : 2;
	unsigned char one : 1;
	// Byte 1
};

struct ChannelListModeStopCommand {
	unsigned char Check : 5;
	unsigned char Ident : 2;
	unsigned char one : 1;
};

struct ChannelListModeUpdateCommand_B234 {
	unsigned char Pulse_Current : 7;
	unsigned char zero2 : 1;
	// Byte 3
	unsigned char Pulse_Width_L : 7;
	unsigned char zero1 : 1;
	// Byte 2
	unsigned char Pulse_Width_H : 2;
	unsigned char : 3;
	unsigned char Mode : 2;
	unsigned char zero0 : 1;
	// Byte 1
};
// they neeed to be sent in increasing order (to respect ch. n.)
// Channel Mode update command, 3 bytes that for each activated channel in the channel list,

struct ChannelListModeUpdateCommand_B1 {
	unsigned char Check : 5;
	unsigned char Ident : 2;
	unsigned char one : 1;
};
// Channel Mode Update command, first byte is standard

struct ChannelListModeInitCommand {
	unsigned char Main_Time_L : 7;
	unsigned char zero4 : 1;
	// Byte 6
	unsigned char Main_Time_H : 4;
	unsigned char Group_Time_L : 3;
	unsigned char zero3 : 1;
	// Byte 5
	unsigned char Group_Time_H : 2;
	unsigned char : 2;
	unsigned char Channel_Lf_L : 5;
	unsigned char zero2 : 1;
	// Byte 4
	unsigned char Channel_Lf_H : 5;
	unsigned char Channel_Stim_L : 2;
	unsigned char zero1 : 1;
	// Byte 3
	unsigned char Channel_Stim_H : 6;
	unsigned char N_Factor_L : 1;
	unsigned char zero0 : 1;				// This and the follows zeroX need to be manually set to 0, ALWAYS
	// Byte 2
	unsigned char N_Factor_H : 2;
	unsigned char Check : 3;
	unsigned char Ident : 2;
	unsigned char one : 1;					// This needs to be manually set to 1, ALWAYS
	// Byte 1
};

// BIG ENDIAN:
/*
// channel list mode initalisation command
// The order should be: byte1 - byte2 - byte3 - byte4 - byte5 - byte6
struct ChannelListModeInitCommand {
	// Byte 1
	unsigned char one : 1;					// This needs to be manually set to 1, ALWAYS
	unsigned char Ident : 2;
	unsigned char Check : 3;
	unsigned char N_Factor_H : 2;
	// Byte 2
	unsigned char zero0 : 1;				// This and the follows zeroX need to be manually set to 0, ALWAYS
	unsigned char N_Factor_L : 1;
	unsigned char Channel_Stim_H : 6;
	// Byte 3
	unsigned char zero1 : 1;
	unsigned char Channel_Stim_L : 2;
	unsigned char Channel_Lf_H : 5;
	// Byte 4
	unsigned char zero2 : 1;
	unsigned char Channel_Lf_L : 5;
	unsigned char : 2;
	unsigned char Group_Time_H : 2;
	// Byte 5
	unsigned char zero3 : 1;
	unsigned char Group_Time_L : 3;
	unsigned char Main_Time_H : 4;
	// Byte 6
	unsigned char zero4 : 1;
	unsigned char Main_Time_L : 7;
};

// Channel Mode Update command, first byte is standard
struct ChannelListModeUpdateCommand_B1 {
	unsigned char one : 1;
	unsigned char Ident : 2;
	unsigned char Check : 5;
};

// Channel Mode update command, 3 bytes that for each activated channel in the channel list,
// they neeed to be sent in increasing order (to respect ch. n.)
struct ChannelListModeUpdateCommand_B234 {
	// Byte 1
	unsigned char zero0 : 1;
	unsigned char Mode : 2;
	unsigned char : 3;
	unsigned char Pulse_Width_H : 2;
	// Byte 2
	unsigned char zero1 : 1;
	unsigned char Pulse_Width_L : 7;
	// Byte 3
	unsigned char zero2 : 1;
	unsigned char Pulse_Current : 7;
};

struct ChannelListModeStopCommand {
	unsigned char one : 1;
	unsigned char Ident : 2;
	unsigned char Check : 5;
};

struct SinglePulseGenerationCommand {
	// Byte 1
	unsigned char one : 1;
	unsigned char Ident : 2;
	unsigned char Check : 5;
	// Byte 2
	unsigned char zero0 : 1;
	unsigned char Channel_Number : 3;
	unsigned char : 2;
	unsigned char Pulse_Width_H : 2;
	// Byte 3
	unsigned char zero1 : 1;
	unsigned char Pulse_Width_L : 7;
	// Byte 4
	unsigned char zero2 : 1;
	unsigned char Pulse_Current : 7;
};

// ACK by the stimulator
struct AckStim {
	unsigned char Ident : 2;
	unsigned char : 5;
	unsigned char Error_Code : 1;
};
*/

class rstimc {
        int fd;
		std::string portName;
		termios SerialSett;
		bool readDato;
		boost::asio::io_service io;
		int sendPacketPeriod = 1000;    // ms
		int checkMainPeriod = 10000;    // ms
		int checkResponsePeriod = 10;

		// Construct a timer without setting an expiry time.
		boost::asio::deadline_timer *timerSendPacket;
        boost::asio::deadline_timer *timerCheckMain;
        boost::asio::deadline_timer *timerCheckResponse;

	public:
		rstimc(std::string _portName = nullptr);
		~rstimc();

		int InitializeSerial();
		// Sending the commands
		int SendPacket(void *bff, int size);
		void TimerSend();
		// This timer checks for commands given to this thread by the main one
		// such as, change type of stim., etc.
		void TimerCheckMain();
		void CheckMain();

		// Reading and checking for errors
		int ReadAndCheck();
		void TimeoutCheck();

		// Creating the commands
		void CreateChannelListModeInitCommand(ChannelListModeInitCommand *clmic, unsigned char N_Factor, unsigned char Channel_Stim, unsigned char Channel_Lf, unsigned char Group_Time, unsigned short Main_Time);
		void CreateChannelListModeUpdateCommand(ChannelListModeUpdateCommand_B1 *b1, ChannelListModeUpdateCommand_B234 *b234, unsigned char Mode, unsigned char Pulse_Width, unsigned char Pulse_Current);
		void CreateChannelListModeStopCommand(ChannelListModeStopCommand *clmsc);
		void CreateSinglePulseGenerationCommand(SinglePulseGenerationCommand *b, unsigned char Channel_Number, unsigned short Pulse_Width, unsigned char Pulse_Current);

		// stimulation patterns
		void StimulateAtFreqFor(int freq, int time, int ch, int pwm, int amp);

};

#endif // rstimc_H
