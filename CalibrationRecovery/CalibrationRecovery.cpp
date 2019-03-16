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


bool save_image;
int counter = 0;
int cExposure = 2;


void command_input()
{
	int input_num;
	 while(true) {
		
		std::cin >>  input_num;
		if ( input_num == 0)
		{
			save_image = true;
		}
		else 
		 cExposure = input_num;
	 }
}

int main(){
	std::thread console_thread(command_input);
	// camera setup
	int cWidth = 320;
	int cHeight = 240;
	int cWhiteBalance = 5100;
	cs::UsbCamera camera = frc::CameraServer::GetInstance()->StartAutomaticCapture();
	camera.SetResolution(cWidth,cHeight);
	camera.SetExposureManual(cExposure);
	camera.SetWhiteBalanceManual(cWhiteBalance);
	cs::CvSink cvSink = frc::CameraServer::GetInstance()->GetVideo();
	cv::Mat source;
	std::cout << "Input 0 to save image, Any other number sets exposure" << std::endl;
  	while(true) {
	    camera.SetExposureManual(cExposure);
 	 	cvSink.GrabFrame(source);
		if ( source.rows > 0)  
		{
			if( save_image == true) 
	{
		std::cout <<  "Saving Image:" << std::endl;
		cv::imwrite("/home/pi/RP_Vision_2019/Calibration/images/exposure" +  std::to_string(cExposure) + "_imageNum" + std::to_string(counter) + ".jpg",source);
		std::cout << "/home/pi/RP_Vision_2019/Calibration/images/exposure"+  std::to_string(cExposure)  + "_imageNum" + std::to_string(counter) + ".jpg" << std::endl;
		counter++;
	}
	save_image = false;

		}
			
		}        
	
	console_thread.join();
	return 0;


};