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

        int debug = 1;          // debug flag, set 1 when additional output requested to console output

        grip::GripPipeline pipeline;

        cs::UsbCamera camera = frc::CameraServer::GetInstance()->StartAutomaticCapture();

        int width = 160;
        int height = 120;
        camera.SetResolution(width, height);

        cs::CvSink cvSink = frc::CameraServer::GetInstance()->GetVideo();

        cs::CvSource outputStreamStd = frc::CameraServer::GetInstance()->PutVideo("Floorlines", width, height);

        cv::Mat source;

        cv::Mat output;

        cv::Mat *output_ptr;

        std::vector<std::vector<cv::Point>> *contours_ptr;

        std::vector<std::vector<cv::Point>> contours;

        std::vector<cv::Point> contour1;

        std::vector<cv::Point> contour2;

        cv::Rect r1, r2;
        cv::Point tl2;
        cv::Point tl1;
        cv::Point br2;
        cv::Point br1;

        uint64_t grab_Frame_Status = 0;

        //-------------------------------------------------------------------------------------------------------------
        //      For testing purposes
        //
        // read in image file
        //    Mat image;
        //source = cv::imread("Test_Image", CV_LOAD_IMAGE_COLOR);
        //grab_Frame_Status = 1;

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
                                pipeline.Process(source);
                                contours_ptr = pipeline.GetFilterContoursOutput();
                                contours = *contours_ptr;

                                //contour1 = contours[0];

                                //r1 = cv::boundingRect(contour1);
                                //                r2 = cv::boundingRect(contour2);

                                // tl1 = r1.tl();
                                // tl2 = r2.tl();
                                // br1 = r1.br();
                                // br2 = r2.br();
                                //if (debug)
                                //{
                                //        std::cout << "Top left of rectangle x: (" << tl1.x << ")" << std::endl;
                                //        std::cout << "Top left of rectangle y: (" << tl1.y << ")" << std::endl;
                                //        std::cout << "Bottom right of rectangle x: (" << br1.x << ")" << std::endl;
                                //        std::cout << "Bottom right of rectangle y: (" << br1.y << ")" << std::endl;
                                //};

                                std::vector<cv::Rect> boundingBoxArray;

                                //boundingBoxArray.push_back(r1);

                                //               boundingBoxArray[1] = r1;

                                int num_contours = contours.size();
                                if (debug)
                                        std::cout << "Found # contours (" << num_contours << ")" << std::endl;
                                int midpointBox[num_contours];
                                int minimum = 0;
                                //               boundingBoxArray[0] = cv::boundingRect(contours[0]);

                                if (num_contours>=1)
                                {
                                        for (int count = 0; count < num_contours; count++)
                                        {
                                                boundingBoxArray.push_back(cv::boundingRect(contours[count]));
                                                /// *** NOT YET WORKING MK 2019-02-24  cv::RoatatedRect rotatedRec = cv::minAreaRect(contours[count]));
                                                int midx = ((boundingBoxArray[count].tl()).x + (boundingBoxArray[count].br()).x) / 2;
                                                int midy = ((boundingBoxArray[count].tl()).y + (boundingBoxArray[count].br()).y) / 2;
                                                midpointBox[count] = midx;
                                        }

                                        int differenceMidpoint[num_contours];
                                        for (int count = 0; count < num_contours; count++)
                                        {
                                                differenceMidpoint[count] = abs((width / 2) - midpointBox[count]);
                                        }

                                        for (int count = 1; count < num_contours; count++)
                                        {
                                                if (differenceMidpoint[count] < differenceMidpoint[minimum])
                                                {
                                                        minimum = count;
                                                }
                                        }

                                        if (debug)
                                        {
                                                std::cout << "Midpoint contour #: (" << minimum << ")" << std::endl;
                                                std::cout << "Midpoint x: (" << midpointBox[minimum] << ")" << std::endl;
                                        }
                                        cv::Scalar RED = cv::Scalar(0, 0, 255); //BGR Red Red

                                        int THICKNESS_WHITE     = 1;
                                        int THICKNESS_RED       = 4;
                                        cv::rectangle(output, boundingBoxArray[minimum].tl(), boundingBoxArray[minimum].br(), RED, THICKNESS_RED, 8, 0);

                                        int midxr = ((boundingBoxArray[minimum].tl()).x + (boundingBoxArray[minimum].br()).x) / 2;
                                        //Draw a THICK circle
                                        cv::Point midpoint(midxr, ((boundingBoxArray[minimum].tl()).y));
                                        cv::circle(output, midpoint, 5, RED, THICKNESS_RED, 8, 0);

                                        cv::Scalar WHITE = cv::Scalar(255, 255, 255);
                                        for (int otherBox = 0; otherBox < num_contours; otherBox++)
                                        {
                                                if (otherBox != minimum)
                                                {
                                                        cv::rectangle(output, boundingBoxArray[otherBox].tl(), boundingBoxArray[otherBox].br(), WHITE, THICKNESS_WHITE, 8, 0);
                                                }
                                        }

                                } // if (num_contours>1)
                                //          std::this_thread::sleep_for (std::chrono::milliseconds(100));
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

        //    std::thread visionThread(VisionThread);

        //    visionThread.detach();

        return 0;
};