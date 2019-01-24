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

        camera.SetResolution(640, 480);

        cs::CvSink cvSink = frc::CameraServer::GetInstance()->GetVideo();

        cs::CvSource outputStreamStd = frc::CameraServer::GetInstance()->PutVideo("Olle2", 640, 480);

        cv::Mat source;

        cv::Mat output;

	cv::Mat* output_ptr;

        while(true) {

            
            cvSink.GrabFrame(source);

//MK	    output = source;
            if ( source.rows > 0)
	    {
//**               cvtColor(source, output, cv::COLOR_BGR2GRAY);
	     pipeline.Process(source);
	     output_ptr = pipeline.GetCvApplycolormapOutput();
	     output = *output_ptr;


             outputStreamStd.PutFrame(output);
//MK	    outputStreamStd.PutFrame(source);

//	    std::this_thread::sleep_for (std::chrono::milliseconds(100));
	    }
        }


//    }

//    std::thread visionThread(VisionThread);

//    visionThread.detach();

	return 0;


};
