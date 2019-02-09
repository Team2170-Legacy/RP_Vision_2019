//#include "WPILib.h"
#include <cscore.h>
#include <cscore_cpp.h>
#include <cameraserver/CameraServer.h>
#include <cameraserver/CameraServerShared.h>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <thread>
#include <chrono>

#include "GripPipeline.h"

int main()

{
//    static void VisionThread()

//    {

        grip::GripPipeline pipeline;

        cs::UsbCamera camera = frc::CameraServer::GetInstance()->StartAutomaticCapture();

        int     width = 160;
        int     height = 120;
        camera.SetResolution(width, height);

        cs::CvSink cvSink = frc::CameraServer::GetInstance()->GetVideo();

        cs::CvSource outputStreamStd = frc::CameraServer::GetInstance()->PutVideo("Olle2", width, height);

        cv::Mat source;

        cv::Mat output;

        cv::Mat* output_ptr;

// read in image file
//    Mat image; 
    source = cv::imread("2019-02-02_10-30-20-233.png", CV_LOAD_IMAGE_COLOR);
    
    if(! source.data )
    {
            std::cout <<  "Could not open or find the image" << std::endl ;
            return -1;
    }

        while(true) {


//            cvSink.GrabFrame(source);


//MK        output = source;
            if ( source.rows > 0)
            {
//**               cvtColor(source, output, cv::COLOR_BGR2GRAY);
             pipeline.Process(source);
             output_ptr = pipeline.GetCvApplycolormapOutput();
             output = *output_ptr;


             outputStreamStd.PutFrame(output);
//MK        outputStreamStd.PutFrame(source);

//          std::this_thread::sleep_for (std::chrono::milliseconds(100));
            }
        }


//    }

//    std::thread visionThread(VisionThread);

//    visionThread.detach();

        return 0;


};
