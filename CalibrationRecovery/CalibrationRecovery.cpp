//to copy files from pi to Priyanshu pc use 
//scp -r pi@10.21.70.22:/home/pi/Calibration/images C:\Users\Priyanshu\Documents\FRC_2019\GIT\RP_Vision_2019\CalibrationRecovery

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
int cExposure = 1;

void command_input()
{

	int input_num;
	 while(true) {
		
		std::cin >>  input_num;
		if ( input_num == 0)
		{
			save_image = true;
		}
		else  {
		 cExposure = input_num;
		 std::cout << " Setting exposure to " +  std::to_string(cExposure) << std::endl;
		}
	 }
}

int main(){
	std::thread console_thread(command_input);
	// camera setup
	int camera_dev_number = 0;
	int cWhiteBalance = 5100;
	int fps = 30;
	int cWidth = 320;
	int cHeight = 240;

    cs::UsbCamera *camera_pointer = new cs::UsbCamera("USB Camera 0", camera_dev_number);
    cs::UsbCamera camera = *camera_pointer;
	camera.SetResolution(cWidth, cHeight);
	camera.SetExposureManual(cExposure);
	camera.SetWhiteBalanceManual(cWhiteBalance);

	cs::MjpegServer *mjpegServer1_pointer = new cs::MjpegServer("Forward Camera", 5805);
    cs::MjpegServer mjpegServer1 = *mjpegServer1_pointer;
    mjpegServer1.SetSource(camera);

	cs::CvSink *cvSink_pointer = new cs::CvSink("Calibration USB Camera");
    cs::CvSink cvSink = *cvSink_pointer;
    cvSink.SetSource(camera);
	cv::Mat source;
	
	std::cout << "Current Exposure is " + cExposure << std::endl;
	std::cout << "Input 0 to save image, Any other number sets exposure" << std::endl;

  	while(true) {
	    camera.SetExposureManual(cExposure);
 	 	cvSink.GrabFrame(source);
		if ( source.rows > 0)  
		{
			if( save_image == true) 
	{
		counter++;
		std::cout <<  "Saving Image:" << std::endl;
		cv::imwrite("/home/pi/Calibration/images/" + std::to_string(counter) + ".jpg", source);
		std::cout << "/home/pi/Calibration/images/"+  std::to_string(counter) + ".jpg" << std::endl;
		std::ofstream myfile;
		myfile.open ("/home/pi/Calibration/images/"+ std::to_string(counter) + ".txt");
		myfile << "Exposure: " + std::to_string(cExposure) + "\n";
		myfile.close();
		save_image = false;
	}
	

		}
			
		}        
	
	console_thread.join();
	return 0;


};