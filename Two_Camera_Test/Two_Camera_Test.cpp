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

        cs::UsbCamera camera0 = frc::CameraServer::GetInstance()->StartAutomaticCapture(0);
        cs::UsbCamera camera1 = frc::CameraServer::GetInstance()->StartAutomaticCapture(1);

        int width = 160;
        int height = 120;
        camera0.SetResolution(width, height);
        camera1.SetResolution(width, height);

        cs::CvSink cvSink0 = frc::CameraServer::GetInstance()->GetVideo(camera0);
        cs::CvSink cvSink1 = frc::CameraServer::GetInstance()->GetVideo(camera1);

        cs::CvSource outputStreamStd0 = frc::CameraServer::GetInstance()->PutVideo("Cam0_Proc", width, height);
        cs::CvSource outputStreamStd1 = frc::CameraServer::GetInstance()->PutVideo("Cam1_Proc", width, height);

        cv::Mat source0;
        cv::Mat source1;

        cv::Mat output0;
        cv::Mat output1;

        cv::Mat *output_ptr;


        //cv::Rect r1, r2;
        //cv::Point tl1;
        //cv::Point tl2;
        //cv::Point br1;
        //cv::Point br2;



        uint64_t grab_Frame_Status0 = 0;
        uint64_t grab_Frame_Status1 = 0;

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
                grab_Frame_Status0 = cvSink0.GrabFrame(source0);
                if (debug)
                        std::cout << "Grab Frame Status0: " << grab_Frame_Status0 << std::endl;
                if (grab_Frame_Status0 > 0)
                {

                        output0 = source0;
                        //outputStreamStd.PutFrame(output);
                        //std::cout << "Streaming output frame...." << std::endl; 

                        if (source0.rows > 0)
                        {
                                //**               cvtColor(source, output, cv::COLOR_BGR2GRAY);
                                cv::Scalar RED = cv::Scalar(0, 0, 255); //BGR Red Red

                                int THICKNESS_WHITE     = 1;
                                int THICKNESS_RED       = 4;
                                cv::Point tl1(100,100);
                                cv::Point br1(110,110);

                                cv::rectangle(output0, tl1, br1, RED, THICKNESS_RED, 8, 0);

                                //Draw a THICC circle
                                cv::Point midpoint(80, 80 );
                                cv::circle(output0, midpoint, 5, RED, THICKNESS_RED, 8, 0);


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

                        outputStreamStd0.PutFrame(output0);

                        if (debug) 
                                std::cout << "Streaming output0 frame...." << std::endl; 


                        
                        
                } // if(grab_Frame_Status > -1)


                grab_Frame_Status1 = cvSink1.GrabFrame(source1);
                if (debug)
                        std::cout << "Grab Frame Status1: " << grab_Frame_Status1 << std::endl;
                if (grab_Frame_Status1 > 0)
                {

                        output1 = source1;
                        //outputStreamStd.PutFrame(output);
                        //std::cout << "Streaming output frame...." << std::endl; 

                        if (source1.rows > 0)
                        {
                                //**               cvtColor(source, output, cv::COLOR_BGR2GRAY);
                                cv::Scalar RED = cv::Scalar(0, 0, 255); //BGR Red Red

                                int THICKNESS_WHITE     = 1;
                                int THICKNESS_RED       = 4;
                                cv::Point tl1(10,10);
                                cv::Point br1(90,90);

                                cv::rectangle(output1, tl1, br1, RED, THICKNESS_RED, 8, 0);

                                //Draw a THICC circle
                                cv::Point midpoint(50, 50 );
                                cv::circle(output1, midpoint, 5, RED, THICKNESS_RED, 8, 0);


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

                        outputStreamStd1.PutFrame(output1);

                        if (debug) 
                                std::cout << "Streaming output1 frame...." << std::endl; 


                        
                        
                } // if(grab_Frame_Status > -1)


        } // while(true)


        return 0;
};