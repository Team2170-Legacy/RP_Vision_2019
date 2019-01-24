//#include "WPILib.h"
#include <cscore.h>
#include <cscore_cpp.h>
#include <cameraserver/CameraServer.h>
#include <cameraserver/CameraServerShared.h>

#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/core/core.hpp>

int main()

{
//    static void VisionThread()

//    {

        cs::UsbCamera camera = frc::CameraServer::GetInstance()->StartAutomaticCapture();

        camera.SetResolution(640, 480);

        cs::CvSink cvSink = frc::CameraServer::GetInstance()->GetVideo();

        cs::CvSource outputStreamStd = frc::CameraServer::GetInstance()->PutVideo("Gray", 640, 480);

        cv::Mat source;

        cv::Mat output;

        while(true) {

            
            cvSink.GrabFrame(source);

//MK	    output = source;
            if ( source.rows > 0)
	    {
               cvtColor(source, output, cv::COLOR_BGR2GRAY);

               outputStreamStd.PutFrame(output);
//MK	    outputStreamStd.PutFrame(source);
	    }
        }


//    }

//    std::thread visionThread(VisionThread);

//    visionThread.detach();

	return 0;


};
