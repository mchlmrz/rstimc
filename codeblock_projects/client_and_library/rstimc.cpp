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

#include "rstimc.h"

std::condition_variable cvEvent;
std::mutex eventMutex;
bool newEvent = 0;
int spp = 0;

rstimc::rstimc(std::string _portName)
{
    portName = _portName;

    InitializeSerial();

	// aggiungere thread
	timerSendPacket = new boost::asio::deadline_timer(io);
	timerCheckMain = new boost::asio::deadline_timer(io);
	//timerCheckResponse  = new boost::asio::deadline_timer(io);

	timerSendPacket->expires_from_now(boost::posix_time::milliseconds(sendPacketPeriod));
	timerSendPacket->async_wait(boost::bind(&rstimc::TimerSend, this));
    std::cout << "timerSendPacket initiated.. \n";

    timerCheckMain->expires_from_now(boost::posix_time::milliseconds(checkMainPeriod));
	timerCheckMain->async_wait(boost::bind(&rstimc::TimerCheckMain, this));
	std::cout << "Check Main initiated.. \n";

	/*
	timerCheckResponse->expires_from_now(boost::posix_time::milliseconds(checkResponsePeriod));
	timerCheckResponse->async_wait(boost::bind(&rstimc::TimerCheckMain, this));
	std::cout << "Check Response initiated.. \n";
    */

    // The handler will be called ONLY if this function has been reached
    // Also this is a blocking function which runs until there's "work" to do.
    // [..] calling io_service::run() from only one thread ensures that callback handlers cannot run concurrently.
    boost::thread t1(boost::bind(&boost::asio::io_service::run, &io));
	boost::thread t2(boost::bind(&boost::asio::io_service::run, &io));


	boost::thread t4(boost::bind(&rstimc::CheckMain, this));
	boost::thread t3(boost::bind(&rstimc::ReadAndCheck, this));

	t1.join();
	t2.join();
	t3.join();
	t4.join();
}

void rstimc::CheckMain()
{
    std::cout << "CheckMain .. \n";
    std::unique_lock<std::mutex> mlock(eventMutex);
    while (1) {
        std::cout << "waiting for an event.. \n";
        cvEvent.wait(mlock, []{return newEvent;});
        std::cout << "#EV received! \n";
        // lock?
        newEvent = 0;
        sendPacketPeriod = spp;
    }
}

rstimc::~rstimc()
{
    if (fd > 0)
        close(fd);
}

int rstimc::InitializeSerial()
{
    fd = open(portName.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd <= 0) {
        std::cout << "[!!] Error opening the port " + portName + "\n";
        std::cout << "[!!] ERROR: " << strerror(errno) << '\n';
        return -1;
    }

    if (flock(fd, LOCK_EX | LOCK_NB) == -1) {
        std::cout << "[!!] Cannot lock the fd\n";
        std::cout << "[!!] ERROR: " << strerror(errno) << '\n';
        return -1;
    }

    memset(&SerialSett, 0, sizeof(SerialSett));
    if (tcgetattr(fd, &SerialSett) == -1) {
        std::cout << "tcgetattr error\n";
        std::cout << "[!!] ERROR: " << strerror(errno) << '\n';
        return -1;
    }

    // BaudRate
    cfsetospeed(&SerialSett, BAUDRATE_REHA);
    cfsetispeed(&SerialSett, BAUDRATE_REHA);

    // Parity Bit
    if (PARITY_REHA)
        SerialSett.c_cflag |= PARENB;
    else
        SerialSett.c_cflag &= ~PARENB;

    // Stop bits
    if (STOPBIT_REHA == 1)
        SerialSett.c_cflag &= ~CSTOPB;
    else if (STOPBIT_REHA == 2)
        SerialSett.c_cflag |= CSTOPB;       // 2 stop bits

    if (DATABITS_REHA == 8) {
        SerialSett.c_cflag &= ~CSIZE;
        SerialSett.c_cflag |=  CS8;         // 8 bit data
    }

    if (FLOWC_RTSCTS)
        SerialSett.c_cflag |= CRTSCTS;      // flow control on

    SerialSett.c_cflag |= CREAD | CLOCAL;
    SerialSett.c_iflag &= ~(IXON | IXOFF | IXANY);      // software flow control

	// Non canonical
	SerialSett.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);

	// Read
	SerialSett.c_cflag |= CREAD | CLOCAL;
	SerialSett.c_cc[VMIN]  = 1;
	SerialSett.c_cc[VTIME] = 0;

    SerialSett.c_iflag &= ~IGNBRK;         // disable break processing
    SerialSett.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
    SerialSett.c_oflag = 0;                // no remapping, no delays

	// make raw..
	cfmakeraw(&SerialSett);

	// Set the settings
	if (tcsetattr(fd,TCSANOW,&SerialSett) == -1) {
        std::cout << "error in tcsetattr\n";
        std::cout << "[!!] ERROR: " << strerror(errno) << '\n';
    }

    if (tcgetattr(fd, &SerialSett) == -1) {
        std::cout << "tcgetattr error, last\n";
        std::cout << "[!!] ERROR: " << strerror(errno) << '\n';
        return -1;
    }


    return 0;
}

int rstimc::SendPacket(void *bff, int size)
{
	// Construct a timer without setting an expiry time.
	////boost::asio::deadline_timer timer(io);

	// Set an expiry time relative to now.
	////timer.expires_from_now(boost::posix_time::seconds(5));

	// Wait for the timer to expire.
	////readDato = 0;
	// we need to signalize the timer that a new "check" must be made
	// timer.async_wait(boost::bind(&rstimc::TimeoutCheck, this));

    std::cout << "We are about to send: " + std::to_string(size) + " bytes, as: \n";

    char *_bff = reinterpret_cast<char *>(bff);
    for (int x = size-1; x >= 0 ; x-- ) {
        std::bitset<8> bindump(_bff[x]);
        std::cout << bindump << "\n";
    }

    std::string confirm;

    //std::cout << "Confirm Send [yes/no]: ";
    //std::cin >> confirm;
    confirm = "yes";

    if (confirm == "no") {
        std::cout << "[!!] Aborted! \n";
        return 0;
    }
    else if (confirm == "yes") {
        for (int x = size-1; x >= 0 ; x-- ) {
            int pk_snt = 0;
            std::cout << "packet sent..";
            pk_snt = write(fd, &_bff[x], 1);

            std::cout << " sent:" + std::to_string(pk_snt) + " ";
            if (pk_snt == -1)
                std::cout << "[!!] ERROR: " << strerror(errno) << '\n';
            // an additional sleep should be added! to waipackett until we have actually sent the pack
        }
        std::cout << '\n';
        //ReadAndCheck();

        //write(fd, bff, size);
        readDato = 1;
        //ReadAndCheck();
    }

	return 0;
}

int rstimc::ReadAndCheck()
{
    unsigned char buf[2];
    int nb = 0;
    while (1) {
        //std::cout << "Reading the answer\n";
        buf[0] = 0x0;
        buf[1] = 0x0;

        nb = read (fd, buf, 1);
        std::cout << "Bytes read: " + std::to_string(nb) + "\n";
        std::bitset<8> bindump(buf[0]);
        std::cout << bindump << "\n";

        if (buf[0] == 0xc0) {
            std::cout << "[!!] rstimc SENT AN ERROR!\n";
        }
        else if (buf[0] == 0xc1) {
            std::cout << "[OK] rstimc 0xc1\n";
        }
        else {
            std::cout << "[??] non standard response " << std::hex << int(buf[0]) << '\n';
        }

        if (nb == -1)
            std::cout << "[!!] ERROR: " << strerror(errno) << '\n';
    }

    return 0;
}

void rstimc::TimeoutCheck()
{
	if (!readDato)
		std::cout << "NO RESPONSE\n";
	else
		readDato = 0;
}

// Variables should be loaded LSB first!! i.e.: if only 1 bit is used in a char, the max number is 00000001
void rstimc::CreateChannelListModeInitCommand(ChannelListModeInitCommand *clmic, unsigned char N_Factor, unsigned char Channel_Stim, unsigned char Channel_Lf, unsigned char Group_Time, unsigned short Main_Time)
{
	// WARNING : assumption, clmic is already initialized, i.e.: there's memory allocated
	// we need the pointer because we need to modify it

	clmic->one = 1;
	clmic->zero0 = 0;
	clmic->zero1 = 0;
	clmic->zero2 = 0;
	clmic->zero3 = 0;
	clmic->zero4 = 0;

	clmic->Ident = 0;

	clmic->Check = ((N_Factor + Channel_Stim + Channel_Lf + Group_Time + Main_Time)%8) & 7;

	clmic->N_Factor_H = (N_Factor >> 1) & 3;
	clmic->N_Factor_L = N_Factor & 1;
	clmic->Channel_Stim_H = (Channel_Stim >> 2) & 63;
	clmic->Channel_Stim_L = Channel_Stim & 3;
	clmic->Channel_Lf_H = (Channel_Lf >> 5) & 31;
	clmic->Channel_Lf_L = Channel_Lf & 31;
	clmic->Group_Time_H = (Group_Time >> 3) & 3;
	clmic->Group_Time_L = Group_Time & 7;
	clmic->Main_Time_H = (Main_Time >> 7) & 15;
	clmic->Main_Time_L = Main_Time & 127;
}

void rstimc::CreateChannelListModeUpdateCommand(ChannelListModeUpdateCommand_B1 *b1, ChannelListModeUpdateCommand_B234 *b234, unsigned char Mode, unsigned char Pulse_Width, unsigned char Pulse_Current)
{
	b1->one = 1;
	b1->Ident = 1;
	b1->Check = ((Mode + Pulse_Width + Pulse_Current)%32) & 31;

	b234->zero0 = 0;
	b234->zero1 = 0;
	b234->zero2 = 0;

	b234->Mode = Mode & 3;
	b234->Pulse_Width_H = (Pulse_Width >> 7) & 3;
	b234->Pulse_Width_L = Pulse_Width & 127;

	b234->Pulse_Current = Pulse_Current & 127;
}
void rstimc::CreateChannelListModeStopCommand(ChannelListModeStopCommand *clmsc)
{
	// It should be always fixed
	clmsc->one = 1;
	clmsc->Check = 0;
	clmsc->Ident = 2;
}

void rstimc::CreateSinglePulseGenerationCommand(SinglePulseGenerationCommand *b, unsigned char Channel_Number, unsigned short Pulse_Width, unsigned char Pulse_Current)
{
	b->one = 1;
	b->Ident = 3;
	b->Check = ((Channel_Number + Pulse_Width + Pulse_Current)%32) & 31;
	b->Channel_Number = Channel_Number & 7;
	b->Pulse_Width_H = (Pulse_Width >> 7) & 3;
	b->Pulse_Width_L = Pulse_Width & 127;
	b->Pulse_Current = Pulse_Current & 127;

	b->zero0 = 0;
	b->zero1 = 0;
	b->zero2 = 0;
}

// constant stimulation
// time in seconds during which the patient will be stimulated
// with burst with the given freq in Hz
void rstimc::StimulateAtFreqFor(int freq, int time, int ch, int pwm, int amp)
{
    SinglePulseGenerationCommand *spgc = new SinglePulseGenerationCommand ();
    CreateSinglePulseGenerationCommand(spgc, ch, pwm, amp);
    for (int i = 0 ; i < time*freq ; i++) {

        SendPacket(spgc, sizeof(SinglePulseGenerationCommand));

        usleep(1000000/freq) ;
    }
}

void rstimc::TimerSend()
{
    // WARNING!!!! Channel n.3 is "2", because it starts counting from 0!
    // So, to stim. channel n.1 we have to send 0

    StimulateAtFreqFor(50, 2, 0, 100, 2);
    StimulateAtFreqFor(100, 2, 0, 100, 2);
    StimulateAtFreqFor(10, 2, 0, 100, 2);

    usleep(10000000);

	std::cout << "--> I'm running at: " + std::to_string(sendPacketPeriod);

	// Set an expiry time relative to now.
	timerSendPacket->expires_from_now(boost::posix_time::milliseconds(sendPacketPeriod));
	timerSendPacket->async_wait(boost::bind(&rstimc::TimerSend, this));
}

void rstimc::TimerCheckMain()
{
    std::cout << "Main Checked \n";

    timerCheckMain->expires_from_now(boost::posix_time::milliseconds(checkMainPeriod));
	timerCheckMain->async_wait(boost::bind(&rstimc::TimerCheckMain, this));
}
