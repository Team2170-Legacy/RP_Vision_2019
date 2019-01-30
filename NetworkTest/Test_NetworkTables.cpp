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


int main()
{
	// Create an object for network tables //
	auto inst = nt::NetworkTableInstance::GetDefault();
	//  Create a new table "visiontable"
	auto table = inst.GetTable("visiontable");
	// initial counters creation
	nt::NetworkTableEntry xEntry =  table->GetEntry("x");
	nt::NetworkTableEntry yEntry =  table->GetEntry("y");
	// add one to x , 2 to y
	int x = 0;
	int y = 0;
	while(true){
		x = x + 1;
		y = y + 2;
		// Save these values to the table.
		// Save X
		xEntry.SetDouble(x);
		// Save Y
		yEntry.SetDouble(y);
	}
	return 0;


};
