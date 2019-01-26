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

int main(){
	int cWidth = 320;
	int cHeight = 240;
	grip::GripPipeline pipeline;
        cs::UsbCamera camera = frc::CameraServer::GetInstance()->StartAutomaticCapture();
        camera.SetResolution(cWidth,cHeight);
        cs::CvSink cvSink = frc::CameraServer::GetInstance()->GetVideo();
        cs::CvSource outputStreamStd = frc::CameraServer::GetInstance()->PutVideo("Olle2", cWidth,cHeight);
        cv::Mat source;
        cv::Mat output;
	cv::Mat* output_ptr;
	char input_variable;
	bool exit_loop = false;
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
                 std::string path = "~/RP_Vision_2019/test_stream/images/Image_" + std::to_string(counter) + ".jpg"; 
 	         std::cout << path << std::endl;
	         cv::imwrite( path , source);   
                 counter++;
	         outputStreamStd.PutFrame(source);
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
