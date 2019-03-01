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


int main()

{
        //    static void VisionThread()

        //    {

        int debug = 1;          // debug flag, set 1 when additional output requested to console output

        cs::UsbCamera camera = frc::CameraServer::GetInstance()->StartAutomaticCapture(1);

        int width = 160;
        int height = 120;
        camera.SetResolution(width, height);

        cs::CvSink cvSink = frc::CameraServer::GetInstance()->GetVideo();

        cs::CvSource outputStreamStd = frc::CameraServer::GetInstance()->PutVideo("Floorlines", width, height);

        cv::Mat source;

        cv::Mat output;

        cv::Mat *output_ptr;


        //cv::Rect r1, r2;
        //cv::Point tl1;
        //cv::Point tl2;
        //cv::Point br1;
        //cv::Point br2;



        uint64_t grab_Frame_Status = 0;

        //-------------------------------------------------------------------------------------------------------------
        //      For testing purposes
        //
        // read in image file
        //    Mat image;
        //source = cv::imread("Test_Image", CV_LOAD_IMAGE_COLOR);
        //grab_Frame_Status = 1;
        //69
        //     if(! source.data )
        //     {
        //             std::cout <<  "Could not open or find the image" << std::endl ;
        //             return -1;
        //     }
        //-------------------------------------------------------------------------------------------------------------

        while (true) // loop forever
        {
                grab_Frame_Status = cvSink.GrabFrame(source);
                if (debug)
                        std::cout << "Grab Frame Status: " << grab_Frame_Status << std::endl;
                if (grab_Frame_Status > 0)
                {

                        output = source;
                        //outputStreamStd.PutFrame(output);
                        //std::cout << "Streaming output frame...." << std::endl; 

                        if (source.rows > 0)
                        {
                                //**               cvtColor(source, output, cv::COLOR_BGR2GRAY);
                                cv::Scalar RED = cv::Scalar(0, 0, 255); //BGR Red Red

                                int THICKNESS_WHITE     = 1;
                                int THICKNESS_RED       = 4;
                                cv::Point tl1(10,10);
                                cv::Point br1(90,90);

                                cv::rectangle(output, tl1, br1, RED, THICKNESS_RED, 8, 0);

                                //Draw a THICC circle
                                cv::Point midpoint(50, 50 );
                                cv::circle(output, midpoint, 5, RED, THICKNESS_RED, 8, 0);


                        } // if ( source.rows > 0)
                        //       }

                        //if ( output.rows > 0)

                        /* if (!source.data)

                        {
                                //std::cout << "output.rows = 0...." << std::endl;
                                std::cout << "! source.data is TRUE...." << std::endl;
                        }
                        else
                                outputStreamStd.PutFrame(output);
                                std::cout << "Streaming output frame...." << std::endl;     
                        */

                        outputStreamStd.PutFrame(output);

                        if (debug) 
                                std::cout << "Streaming output frame...." << std::endl; 


                        
                        
                } // if(grab_Frame_Status > -1)
        } // while(true)


        return 0;
};