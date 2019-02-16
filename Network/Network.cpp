//Updated 2/16/19

//#include "WPILib.h"
#include <cscore.h>
#include <cscore_cpp.h>
#include <cameraserver/CameraServer.h>
#include <cameraserver/CameraServerShared.h>

#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/core/core.hpp>

#include <thread>
#include <chrono>

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include <iostream>
using namespace std;
int main()
{
	unsigned int port = 1735;

	// Create an object for network tables //
	auto inst = nt::NetworkTableInstance::GetDefault();
	std::cout<<"Network Table Created"<<std::endl;
	inst.StartClient("169.254.143.129",port);
	std::cout<<"Network Table Started"<<std::endl;
	//  Create a new table "visiontable"
	auto table = inst.GetTable("visiontable");
	std::cout<<"Got Network Table"<<std::endl;
	// initial counters creation
	nt::NetworkTableEntry e_Vision =  table->GetEntry("e_Vision");
	nt::NetworkTableEntry d_Vision =  table->GetEntry("d_Vision");
	nt::NetworkTableEntry e_FloorLine =  table->GetEntry("e_FloorLine");
	nt::NetworkTableEntry d_FloorLine =  table->GetEntry("d_FloorLine");

	e_Vision.SetDouble(0);
	d_Vision.SetDouble(0);
	e_FloorLine.SetDouble(0);
	d_FloorLine.SetDouble(0);

	//increase values every five seconds
	double  x = 0;
	int seconds = 5;
	while(true){
		x ++;
		std::this_thread::sleep_for(std::chrono::seconds(seconds));
		e_Vision.SetDouble(x);
		d_Vision.SetDouble(2 * x);
		e_FloorLine.SetDouble(4 * x);
		d_FloorLine.SetDouble(8 * x);
	}
	
	return 0;

};
