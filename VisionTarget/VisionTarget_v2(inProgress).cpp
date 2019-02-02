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
#include "GripPipeline.cpp"
#include <string>
#include <iostream>

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"

int main(){

	  grip::GripPipeline pipeline;
	// camera setup
	int cWidth = 320;
	int cHeight = 240;
	int cExposure = 17;
	int cWhiteBalance = 5100;
        cs::UsbCamera camera = frc::CameraServer::GetInstance()->StartAutomaticCapture();
        camera.SetResolution(cWidth,cHeight);
		camera.SetExposureManual(cExposure);
	    camera.SetWhiteBalanceManual(cWhiteBalance);
        cs::CvSink cvSink = frc::CameraServer::GetInstance()->GetVideo();
        cs::CvSource outputStreamStd = frc::CameraServer::GetInstance()->PutVideo("Olle2", cWidth,cHeight);
       
	   // images
	    cv::Mat source;
        cv::Mat output;
		cv::Mat output_ptr;

		// for contours
		cv::Rect r1, r2;
	std::vector<std::vector<cv::Point> >* contours_ptr;
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Point> contour1;
	std::vector<cv::Point> contour2;

	char input_variable;
	bool exit_loop = false;

//network tables setup
	nt::NetworkTableEntry entry;
	auto inst = nt::NetworkTableInstance::GetDefault();
	auto table = inst.GetTable("table");
	
	while( exit_loop == false )
        {
	std::cout << "Waiting to start camera capture " << std::endl;
	std::cin >>  input_variable;
        int counter = 0;
        while(true) {
            cvSink.GrabFrame(source);
            if ( source.rows > 0)
	    {
	       if ( input_variable == 's' )
	       {
			   // getting contours output from pipeline
			    pipeline.Process(source);
				contours_ptr = pipeline.GetFilterContoursOutput();
	            contours = *contours_ptr;

              // contours need to be converted to an image
			  if(!contours.empty()){
			  output_ptr = contoursToMat(contours);
			  output = *output_ptr;
		   
                 std::string path = "~/RP_Vision_2019/test_stream/images/Image_" + std::to_string(counter) + ".jpg"; 
 	         std::cout << path << std::endl;
	         cv::imwrite( path , output);   
                 counter++;
				 // putting counter in network table
				 table->PutNumber("counter", counter);
	         outputStreamStd.PutFrame(output);
			  } 
		}
		else if ( input_variable == 'e' )
		{
			exit_loop = true;
		}
		else
		{
		  outputStreamStd.PutFrame(source);
		}
	    }
        
	}
	}
	return 0;


};

// method to convert contours to mat(image)

cv::Mat* contoursToMat(std::vector<std::vector<cv::Point> > contours) {
return cv::Mat;
}
