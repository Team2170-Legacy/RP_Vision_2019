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
        cv::Point tl2;
        cv::Point tl1;
        cv::Point br2;
        cv::Point br1;
// read in image file
//    Mat image; 
    source = cv::imread("Test_Image", CV_LOAD_IMAGE_COLOR);
    
    if(! source.data )
    {
            std::cout <<  "Could not open or find the image" << std::endl ;
            return -1;
    }
  
        while(true) 
        {


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
                tl2 = r2.tl();
                br1 = r1.br();
                br2 = r2.br();
                std::cout << "Top left of rectangle x: (" << tl1.x << ")" << std::endl;
                std::cout << "Top left of rectangle y: (" << tl1.y << ")" << std::endl;
                std::cout << "Bottom right of rectangle x: (" << br1.x << ")" << std::endl;
                std::cout << "Bottom right of rectangle y: (" << br1.y << ")" << std::endl;
                std::vector<cv::Rect> boundingBoxArray;

                //boundingBoxArray.push_back(r1);

 //               boundingBoxArray[1] = r1;

                int num_contours = contours.size();
                int midpointBox [num_contours];
                int minimum = 0;
 //               boundingBoxArray[0] = cv::boundingRect(contours[0]);

                for(int count = 0; count < num_contours; count++){        
                       boundingBoxArray.push_back(cv::boundingRect(contours[count]));
                        int midx = ( (boundingBoxArray[count].tl()).x + (boundingBoxArray[count].br()).x ) / 2;
                        int midy = ( (boundingBoxArray[count].tl()).y + (boundingBoxArray[count].br()).y ) / 2;
                       midpointBox[count] = midx;
                }

                int differenceMidpoint [num_contours];
                for(int count = 0; count < num_contours; count++){
                        differenceMidpoint[count] = abs((width/2) - midpointBox[count]);
                }
                for(int count = 1; count < num_contours; count++){
                        if(differenceMidpoint[count] < differenceMidpoint[minimum]){
                                minimum = count;
                        }
                }

        {
                std::cout <<"Minimum: (" << minimum << ")" << std::endl;
                std::cout <<"Midpoint x: (" << midpointBox[minimum] << ")" <<std::endl; 
        }
             //output_ptr = pipeline.GetCvApplycolormapOutput();
             //output = *output_ptr;
                output = source;
                cv::Scalar color = cv::Scalar(0,0,255); //BGR Red Red

//                tl1.x = 20;
//                tl1.y = 20;
//                br1.x = 60;
//                br1.y = 60;

                int THICKNESS = 1;
                cv::rectangle(output, boundingBoxArray[minimum].tl(),boundingBoxArray[minimum].br(), color, THICKNESS+3, 8, 0);
//                cv::rectangle(output, tl2, br2, color, 4, 8, 0);


                int midxr = ( (boundingBoxArray[minimum].tl()).x + (boundingBoxArray[minimum].br()).x ) / 2;
                //Draw a THICC circle
                cv::Point midpoint(midxr, ((boundingBoxArray[minimum].tl()).y));
                cv::circle(output, midpoint, 5, color, THICKNESS, 8, 0);

                cv::Scalar color2 = cv::Scalar(255,255,255);
                if(num_contours > 1){
                        for(int otherBox = 0; otherBox < num_contours; otherBox++){
                                if(otherBox != minimum){
                                        cv::rectangle(output, boundingBoxArray[otherBox].tl(), boundingBoxArray[otherBox].br(), color2, THICKNESS, 8, 0);
                                }
                        }
                }

                // for(int count = 0; count < num_contours; count++){
                //         cv::circle(output, midpoint, 1, color)
                // }

             outputStreamStd.PutFrame(output);
//MK        outputStreamStd.PutFrame(source);

//          std::this_thread::sleep_for (std::chrono::milliseconds(100));
            }
 //       }


    }

//    std::thread visionThread(VisionThread);

//    visionThread.detach();

        return 0;


};