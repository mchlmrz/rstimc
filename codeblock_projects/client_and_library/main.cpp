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


#include <iostream>
#include <thread>
#include <condition_variable>
#include <mutex>

#include "rstimc.h"

using namespace std;

extern std::condition_variable cvEvent;
extern std::mutex eventMutex;
extern bool newEvent;
extern int spp;

void Threadrstimc()
{
    rstimc rstim("/dev/ttyUSB0");
    //rstimc rstim("/dev/pts/2");

    // Maybe here we'll check for flags etc.
    // while (1);
}

int main()
{
    std::thread trs(Threadrstimc);

    while (1) {
        // std::lock_guard<std::mutex> guard(eventMutex);
        usleep(10000000);
        /*
        spp = 400;
        newEvent = 1;
        cvEvent.notify_all();
        std::cout << "$ event notified\n";
        */
    }

    // Blocks the current thread until the thread finishes
    trs.join();

    return 0;
}
