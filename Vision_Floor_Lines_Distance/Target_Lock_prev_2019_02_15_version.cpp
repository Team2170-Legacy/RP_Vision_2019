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
        grip::GripPipeline pipeline;

        cs::UsbCamera camera = frc::CameraServer::GetInstance()->StartAutomaticCapture();

        int width = 160;
        int height = 120;
        camera.SetResolution(width, height);

        cs::CvSink cvSink = frc::CameraServer::GetInstance()->GetVideo();

        cs::CvSource outputStreamStd = frc::CameraServer::GetInstance()->PutVideo("Olle2", width, height);

        cv::Mat source;

        cv::Mat output;

        cv::Mat *output_ptr;

        std::vector<std::vector<cv::Point>> *contours_ptr;

        std::vector<std::vector<cv::Point>> contours;

        std::vector<cv::Point> contour1;

        std::vector<cv::Point> contour2;

        cv::Rect r1, r2;

        cv::Point tl1;
        cv::Point br1;
        source = cv::imread("2019-02-02_10-30-20-233.png", CV_LOAD_IMAGE_COLOR);

        if (!source.data)
        {
                std::cout << "Could not open or find the image" << std::endl;
                return -1;
        }
        if (source.rows > 0)
        {
                pipeline.Process(source);
                contours_ptr = pipeline.GetFilterContoursOutput();
                contours = *contours_ptr;
                //For Loop test
                int num_contours = contours.size();
                int midpointBox[num_contours];
                std::vector<cv::Rect> boundingBoxArray;
                int minimum = 0;
                int minimum2 = 0;
                if (num_contours >= 2)
                {
                        for (int count = 0; count < num_contours; count++)
                        {
                                boundingBoxArray[count] = cv::boundingRect(contours[count]);
                                int midx = ((boundingBoxArray[count].tl()).x + (boundingBoxArray[count].br()).x) / 2;
                                midpointBox[count] = midx;
                        }
                        int differenceMidpoints[num_contours];
                        for (int count = 0; count < num_contours; count++)
                        {
                                differenceMidpoints[count] = abs((width / 2) - midpointBox[count]);
                        }
                        for (int count = 1; count < num_contours; count++)
                        {
                                if (differenceMidpoints[count] < differenceMidpoints[minimum])
                                {
                                        minimum = count;
                                }
                        }
                        for (int count = 1; count < num_contours; count++)
                        {
                                if (differenceMidpoints[count] > differenceMidpoints[minimum2] && differenceMidpoints[minimum2] > differenceMidpoints[count])
                                {
                                        minimum2 = count;
                                }
                        }
                }
                else
                {
                        // Insufficient number of contours found
                }
        }

        return 0;
};
