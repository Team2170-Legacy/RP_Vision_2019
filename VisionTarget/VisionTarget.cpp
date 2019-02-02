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

        while(true) {

            
            cvSink.GrabFrame(source);


            if ( source.rows > 0)
	    {

	     pipeline.Process(source);
	   //  output_ptr = pipeline.()
	    // output = *output_ptr;


             outputStreamStd.PutFrame(output);



	    }
        }


//    }

//    std::thread visionThread(VisionThread);

//    visionThread.detach();

	return 0;


};
