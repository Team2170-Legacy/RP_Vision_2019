
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

#include <thread>
#include <chrono>
#include <cmath>
#include "GripPipeline.h"
#include <string>
#include <iostream>
#include <fstream>

float calcTargetHeight(std::vector<std::vector<cv::Point>> contours)
{
    float height = 0;
	if (contours.size() == 2) 
	{
	
    std::vector<std::vector<float>> bounding;

        for (int c = 0; c < 2; c++)
        {
            cv::RotatedRect boundingBox = cv::minAreaRect(contours[c]);

            cv::Point2f corners[4];
            boundingBox.points(corners);
        
            float MinY = fminf(corners[0].y, fminf(corners[1].y, fminf(corners[2].y, corners[3].y)));
            float MaxY = fmaxf(corners[0].y, fmaxf(corners[1].y, fmaxf(corners[2].y, corners[3].y)));
            
            std::vector<float> tmp;
            tmp.push_back(MinY);
            tmp.push_back(MaxY);
            bounding.push_back(tmp);


        }

	float MinY = fminf(bounding.at(0).at(0), bounding.at(1).at(0));
	float MaxY = fmaxf(bounding.at(0).at(1), bounding.at(1).at(1));
    height = MaxY - MinY;
	}

	return height;
}

int main() 
{
bool repeat = true;


grip::GripPipeline pipeline;

    while (repeat)
    {
  //  repeat = true;
    std::string imname;
    std::cout << "Enter Image Name of an Image in ~/Calibration/images/" << std::endl;
    getline(std::cin, imname);
    std::string path = "~/Calibration/images/" + imname;
    cv::Mat source = cv::imread(path);
    std::cout << "Reading Image: " + path << std::endl;
    std::vector<std::vector<cv::Point> >* contours_ptr;
    std::vector<std::vector<cv::Point> > contours;

        if (source.rows > 0) 
        {
        pipeline.Process(source);
        
        contours_ptr = pipeline.GetFilterContoursOutput();
        contours = *contours_ptr;

            if (contours.size() == 2) 
            { 
            std::cout << "The Target Height in this image is " + std::to_string(round(calcTargetHeight(contours))) + " pixels." << std::endl;
            }
            else if (contours.size() < 2) 
            {
            std::cout << "Not enough contours to create a target, this program requires exactly 2 contours to be in the image" << std::endl;
            }
            else
            {
            std::cout << "More than 2 contours found, this program requires exactly 2 contours to be in the image" << std::endl;
            }
        }
        else
        {
        std::cout << "Image Not Found" << std::endl;
        }
        /*
    std::cout << "Press enter to find the height of another image" << std::endl;
        if(std::cin.get() == '\n')
        {
            repeat = true;
        }
        */

    }

}