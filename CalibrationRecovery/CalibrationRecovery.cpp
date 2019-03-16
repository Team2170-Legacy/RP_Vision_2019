//#include "WPILib.h"
#include <cscore.h>
#include <cscore_cpp.h>
#include <cameraserver/CameraServer.h>
#include <cameraserver/CameraServerShared.h>

#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/core/core.hpp>

#include <thread>
#include <chrono>


#include "GripPipeline.h"
#include <string>
#include <iostream>
#include <fstream>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"

bool save_image;
int counter = 0;


void command_input()
{
	std::string input_variable;
	 while(true) {
		std::cout << "Input S to Save image" << std::endl;
		std::cin >>  input_variable;
		if ( input_variable == "S" || input_variable == "s")
		{
			//std::cout << "Setting Save image to true" << std::endl;
			save_image = true;
		}
	 }
}

int main(){
	std::thread console_thread(command_input);
	// camera setup
	int cWidth = 320;
	int cHeight = 240;
	int cExposure = 2;
	int cWhiteBalance = 5100;
	cs::UsbCamera camera = frc::CameraServer::GetInstance()->StartAutomaticCapture();
	camera.SetResolution(cWidth,cHeight);
	camera.SetExposureManual(cExposure);
	camera.SetWhiteBalanceManual(cWhiteBalance);
	cs::CvSink cvSink = frc::CameraServer::GetInstance()->GetVideo();
	cs::CvSource outputStreamStd = frc::CameraServer::GetInstance()->PutVideo("hsvoutput", cWidth,cHeight);
	cs::CvSource outputStreamRectStd = frc::CameraServer::GetInstance()->PutVideo("countouroutput", cWidth,cHeight);
	cv::Mat source;
	
  	while(true) {
 	 	cvSink.GrabFrame(source);
		if ( source.rows > 0)  
		{
			if( save_image == true) 
	{
		std::cout <<  "Saving Image:" << std::endl;
		cv::imwrite("/home/pi/RP_Vision_2019/Calibration/images/"+ std::to_string(counter) + ".jpg",source);
		std::cout << "/home/pi/RP_Vision_2019/Calibration/images/"+ std::to_string(counter) + ".jpg" << std::endl;
		std::ofstream myfile;
		counter++;
	}
	save_image = false;

		}
	outputStreamRectStd.PutFrame(source);
			
		}        
	
	console_thread.join();
	return 0;


};