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
	//  while(true) 
    // {
        std::cout << "Press Enter to Start Data Logging";
        if (std::cin.get() == '\n')
        {
            logData = true; //!logData;
                
            if(!logData) 
            {
            std::cout << "Data Logging Ended: Press Enter to Resume Data Logging";
            logData = false;
            }
            else 
            {
            std::cout << "Data Logging Started: Press Enter to Stop Data Logging" << std::endl; 
            }   
        }

//    }
}

int main(){

char key_ch;

// start command input thread

//std::thread console_thread(command_input);


// network table setup
auto inst = nt::NetworkTableInstance::GetDefault();
unsigned int port = 1735;
inst.StartClient("10.21.70.2",port);
auto table = inst.GetTable("VisionTable"); 

nt::NetworkTableEntry x_target_error =  table->GetEntry("x_target_error");
nt::NetworkTableEntry distance_to_target =  table->GetEntry("distance_to_target");
nt::NetworkTableEntry tape_align_error = table->GetEntry("tape_align_error");
nt::NetworkTableEntry target_locked =  table->GetEntry("target_locked");
nt::NetworkTableEntry left_tape_area = table->GetEntry("left_tape_area");
nt::NetworkTableEntry right_tape_area = table->GetEntry("right_tape_area");


    /*
    // from Vision Target
	//network tables setup
	unsigned int port = 1735;
	auto inst = nt::NetworkTableInstance::GetDefault();
	std::cout << "server started" << std::endl;
	inst.StartClient("10.21.70.2",port);
	auto table = inst.GetTable("VisionTable");

	nt::NetworkTableEntry left_tape_angle = table->GetEntry("left_tape_angle");
	nt::NetworkTableEntry right_tape_angle = table->GetEntry("right_tape_angle");
    */


//   Use this for floor lines
// nt::NetworkTableEntry x_target_error =  table->GetEntry("x_target_error");
// nt::NetworkTableEntry tape_align_error =  table->GetEntry("tape_align_error");
// nt::NetworkTableEntry distance_to_target =  table->GetEntry("distance_to_target");


std::cout << "Press Enter to Start Data Logging, Enter again to Stop...\n";
//if (std::cin.get() == '\n')
key_ch = getchar();
// while (key_ch != '\n')
//     key_ch = getchar();

logData = true;
std::cout << "Starting Data Logging...\n";
//key_ch = getchar();

// time variables setup, note they are not all in same units
const double PERIOD = 0.050; // unit is seconds
const double DURATION = 10.0;       // [s]  Time duration to log data for

clock_t this_time = clock(); // unit is clockticks
clock_t last_time = this_time; // unit is clockticks
clock_t start_time = this_time; // unit is clockticks
double time_counter = 0; // unit is clockticks

double t =0;
double dt = 0;
int count = 0;

// create dataLog file variable
std::ofstream dataLog;

dataLog.open("/home/pi/DataLogger/Log_0001.txt");
dataLog << "t, x_target_error, distance_to_target, tape_align_error, target_locked, left_tape_area, right_tape_area\n";
    while (logData) 
    {
        this_time = clock();
        //    time_counter += (double)(this_time - last_time);
        //time_counter = (double)(this_time - last_time);

        dt = ((double)this_time - (double)last_time)/(double)CLOCKS_PER_SEC;

       //std::cout << "dt = " + std::to_string(dt) + "\n";
        
        // the code in this if statement runs once every PERIOD
        if (dt >= PERIOD )
        {
            t = ((double)this_time - (double)start_time)/(double)CLOCKS_PER_SEC;
            last_time = this_time;

            std::cout << "Logging at t = " + std::to_string(t) + "\n";

            // reading info from network tables
            double xte =  x_target_error.GetDouble(0); 
            double tae =  tape_align_error.GetDouble(0);
            double distance = distance_to_target.GetDouble(0);
            

            // writing to the dataLog
            // dataLog.open("/home/pi/DataLogger/Log");
            // dataLog << "Data Log Entry " + std::to_string(count) + "\n";
            // dataLog << "Time Since Logging Started: " + std::to_string(time_counter/CLOCKS_PER_SEC + (double)(count * PERIOD)) + " seconds" + "\n";
            // dataLog << "X Target Error: " + std::to_string(xte) + "\n";
            // dataLog << "Tape Align Error: " + std::to_string(tae) + "\n";
            // dataLog << "Distance (VisionTarget Based): " + std::to_string(distance) + "\n";
            // dataLog << "\n";
            
            // time
            dataLog << std::to_string( t ) + ", ";


            dataLog << std::to_string(x_target_error.GetDouble(0)) + ", ";
            dataLog << std::to_string(distance_to_target.GetDouble(0)) + ", ";
            dataLog << std::to_string(tape_align_error.GetDouble(0)) + ", ";
            dataLog << std::to_string(target_locked.GetBoolean(false)) + ", ";
            dataLog << std::to_string(left_tape_area.GetDouble(0)) + ", ";
            dataLog << std::to_string(right_tape_area.GetDouble(0)) + ", ";

            dataLog << "\n";

            count++;
            // key_ch = getchar();
            // if (key_ch = '\n')
            //     logData = false;

            if (t >= DURATION)
                logData = false;
        }
    }
   
   std::cout << "Saving data log file...\n\n";

   dataLog.close();

   //console_thread.join();


   return 0;
}
