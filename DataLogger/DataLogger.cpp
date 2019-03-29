#include <string>
#include <iostream>
#include <fstream>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/EntryListenerFlags.h"
#include <stdio.h>
#include <time.h>

bool logData = false; 

void command_input()
{
	 while(true) 
    {
        std::cout << "Press Enter to Start Data Logging";
        if (std::cin.get() == '\n')
        {
            logData = !logData;
                
            if(!logData) 
            {
            std::cout << "Data Logging Ended: Press Enter to Resume Data Logging";
            }
            else 
            {
            std::cout << "Data Logging Started: Press Enter to Stop Data Logging" << std::endl; 
            }   
        }

    }
}

int main(){

// start command input thread
std::thread console_thread(command_input);

// network table setup
auto inst = nt::NetworkTableInstance::GetDefault();
unsigned int port = 1735;
inst.StartClient("10.21.70.2",port);
auto table = inst.GetTable("VisionTable"); 
nt::NetworkTableEntry x_target_error =  table->GetEntry("x_target_error");
nt::NetworkTableEntry tape_align_error =  table->GetEntry("tape_align_error");
nt::NetworkTableEntry distance_to_target =  table->GetEntry("distance_to_target");

// time variables setup, note they are not all in same units
const double NUM_SECONDS = .1; // unit is seconds
clock_t this_time = clock(); // unit is clockticks
clock_t last_time = this_time; // unit is clockticks
double time_counter = 0; // unit is clockticks
int count = 0;

// create dataLog file variable
std::ofstream dataLog;

    while (logData) 
    {
    this_time = clock();
    time_counter += (double)(this_time - last_time);
    last_time = this_time;

        // the code in this if statement runs once every NUM_SECONDS
        if(time_counter > (double)(NUM_SECONDS * CLOCKS_PER_SEC))
        {
        time_counter = 0;
              
        // reading info from network tables
        double xte =  x_target_error.GetDouble(0); 
        double tae =  tape_align_error.GetDouble(0);
        double distance = distance_to_target.GetDouble(0);

        // writing to the dataLog
        dataLog.open("/home/pi/DataLogger/Log");
        dataLog << "Data Log Entry " + std::to_string(count) + "\n";
        dataLog << "Time Since Logging Started: " + std::to_string(time_counter/CLOCKS_PER_SEC + (double)(count * NUM_SECONDS)) + " seconds" + "\n";
        dataLog << "X Target Error: " + std::to_string(xte) + "\n";
        dataLog << "Tape Align Error: " + std::to_string(tae) + "\n";
        dataLog << "Distance (VisionTarget Based): " + std::to_string(distance) + "\n";
        dataLog << "\n";
        dataLog.close();

        count++;
       }
    }
    
   console_thread.join();
   return 0;
}
