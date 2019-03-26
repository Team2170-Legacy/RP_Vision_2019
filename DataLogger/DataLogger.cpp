#include <string>
#include <iostream>
#include <fstream>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/EntryListenerFlags.h"
#include <stdio.h>
#include <time.h>

int main(){
double x_target_error_arr[10000];
bool logData = false; 
auto inst = nt::NetworkTableInstance::GetDefault();
unsigned int port = 1735;
inst.StartClient("10.21.70.2",port);
auto table = inst.GetTable("VisionTable"); 
nt::NetworkTableEntry x_target_error =  table->GetEntry("x_target_error");
const double NUM_SECONDS = .1;
clock_t this_time = clock();
clock_t last_time = this_time;
double time_counter = 0;
int count = 0;
const double timeToLog = 5.0;
std::cout << "Press enter to begin Data Logging";
if (std::cin.get() == '\n'){
    logData = !logData;
}
while (logData){
    if (time_counter > timeToLog){
        logData = false; 
    }
     this_time = clock();

        time_counter += (double)(this_time - last_time);

        last_time = this_time;

        if(time_counter > (double)(NUM_SECONDS * CLOCKS_PER_SEC))
        {
            time_counter -= (double)(NUM_SECONDS * CLOCKS_PER_SEC);
            printf("%d\n", count);
            //std::cout <<  x_target_error.GetDouble("x_target_error") << std::endl;

        x_target_error_arr[count] =  x_target_error.GetDouble("x_target_error"); 
        count ++;
        }
    
   }
}
