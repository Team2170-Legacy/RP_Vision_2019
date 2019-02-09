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

        std::vector<std::vector<cv::Point> >* contours_ptr;

        std::vector<std::vector<cv::Point> > contours;

        std::vector<cv::Point> contour1;

        std::vector<cv::Point> contour2;

        cv::Rect r1, r2;

        cv::Point tl1;
        cv::Point br1;
// read in image file
//    Mat image; 
    source = cv::imread("2019-02-02_10-30-20-233.png", CV_LOAD_IMAGE_COLOR);
    
    if(! source.data )
    {
            std::cout <<  "Could not open or find the image" << std::endl ;
            return -1;
    }
  
 //       while(true) 
  //      {


//            cvSink.GrabFrame(source);

//69
//MK        output = source;
            if ( source.rows > 0)
            {
//**               cvtColor(source, output, cv::COLOR_BGR2GRAY);
                pipeline.Process(source);
                contours_ptr = pipeline.GetFilterContoursOutput();
                contours = *contours_ptr;

                contour1 =contours[0];

                r1 = cv::boundingRect(contour1);
//                r2 = cv::boundingRect(contour2);

                tl1 = r1.tl();
                br1 = r1.br();
                std::cout << "Top left of rectangle x: (" << tl1.x << ")" << std::endl;
                std::cout << "Top left of rectangle y: (" << tl1.y << ")" << std::endl;
                std::cout << "Bottom right of rectangle x: (" << br1.x << ")" << std::endl;
                std::cout << "Bottom right of rectangle y: (" << br1.y << ")" << std::endl;
             //output_ptr = pipeline.GetCvApplycolormapOutput();
             //output = *output_ptr;


             outputStreamStd.PutFrame(output);
//MK        outputStreamStd.PutFrame(source);

//          std::this_thread::sleep_for (std::chrono::milliseconds(100));
            }
 //       }


//    }

//    std::thread visionThread(VisionThread);

//    visionThread.detach();

        return 0;


};
