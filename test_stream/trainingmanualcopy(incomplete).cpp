//#include "WPILib.h"
#include <cscore.h>
#include <cscore_cpp.h>
#include <cameraserver/CameraServer.h>
#include <cameraserver/CameraServerShared.h>

#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/core/core.hpp>

#include <thread>
#include <chrono>
#include <string>
#include <iostream>

#include "GripPipeline.h"

int main()

{
        int cWidth = 320'
        int cHeight = 320;
        grip::GripPipeline pipeline;
     
//    static void VisionThread()

//    {

	grip::GripPipeline pipeline;

        cs::UsbCamera camera = frc::CameraServer::GetInstance()->StartAutomaticCapture();

        camera.SetResolution(640, 480);

        cs::CvSink cvSink = frc::CameraServer::GetInstance()->GetVideo();

        cs::CvSource outputStreamStd = frc::CameraServer::GetInstance()->PutVideo("Olle2", 640, 480);

        cv::Mat source;
        cv::Mat output;
	cv::Mat* output_ptr;
        char input_variable;
        bool exit_loop = false;
        while( exit_loop == false)
        {
int counter = 0;
        while(true) {

            
            cvSink.GrabFrame(source);

//MK	    output = source;
            if ( source.rows > 0)
	    {
                
                 imwrite( "images/Image_"+counter+".jpg", source);   
                  counter++;
//**               cvtColor(source, output, cv::COLOR_BGR2GRAY);
	    

	    outputStreamStd.PutFrame(source);


	    }
        }
        }


//    }



	return 0;


};
