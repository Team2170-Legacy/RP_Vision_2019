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
#include <string>
#include <iostream>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"

cv::Mat detect_rectangles(cv::Mat source, std::vector<std::vector<cv::Point>> contours)
{
	cv::Mat drawing = cv::Mat::zeros( source.size(), CV_8UC3 );
	if (contours.size() < 2)
	{
		return drawing;
	}	
	//cv::findContours(mask,contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	
	/*
    // compute mask (you could use a simple threshold if the image is always as good as the one you provided)
    cv::Mat mask;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mask,contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    /// Draw contours and find biggest contour (if there are other contours in the image, we assume the biggest one is the desired rect)
    // drawing here is only for demonstration!
    int biggestContourIdx = -1;
    float biggestContourArea = 0;
    cv::Mat drawing = cv::Mat::zeros( mask.size(), CV_8UC3 );
    for( int i = 0; i< contours.size(); i++ )
    {
        cv::Scalar color = cv::Scalar(0, 100, 0);
        drawContours( drawing, contours, i, color, 1, 8, hierarchy, 0, cv::Point() );
    }

    // if no contour found
	/*
    if(biggestContourIdx < 0)
    {
        std::cout << "no contour found" << std::endl;
        return source;
    }
	*/
float max_y , max_x;
float least_y, least_X;
cv::Point2f* topLeft;
cv::Point2f* bottomLeft;
cv::Point2f* topRight;
cv::Point2f* bottomRight;
    // compute the rotated bounding rect of the biggest contour! (this is the part that does what you want/need)
	for ( int c = 0; c < 2; c++)
	{
		cv::RotatedRect boundingBox = cv::minAreaRect(contours[c]);
		// one thing to remark: this will compute the OUTER boundary box, so maybe you have to erode/dilate if you want something between the ragged lines
		// draw the rotated rect
		cv::Point2f corners[4];
		boundingBox.points(corners);
		cv::line(drawing, corners[0], corners[1], cv::Scalar(255,255,255));
		cv::line(drawing, corners[1], corners[2], cv::Scalar(255,255,255));
		cv::line(drawing, corners[2], corners[3], cv::Scalar(255,255,255));
		cv::line(drawing, corners[3], corners[0], cv::Scalar(255,255,255));
		if (c == 0)
		{
			topLeft = new cv::Point2f(corners[1].x, corners[0].y);
			bottomLeft = new cv::Point2f(corners[1].x, corners[2].y);
		}
		else
		{
			topRight = new cv::Point2f(corners[2].x, corners[3].y);
			bottomRight = new cv::Point2f(corners[2].x, corners[1].y);
		}


/*
Left Rectangle
topleft red 0
topright white 3
bottomleft green 1
bottomright blue 2

Right Rectangle
topleft green 1
topright red 0
bottomleft blue 2
bottomright white 3

Left Rectangle GreenX 1.x , RedY 0.y, Topleft Corner
Left Rectangle GreenX 1.x , BlueY 2.y, Bottomleft Corner
Right rectangle WhiteX 3.x, RedY 0.y, Topright corner
Right Rectangle Whitex 3.x, BlueY 2.y Bottomright corner
*/
	cv::circle(drawing, corners[0], 5, cv::Scalar(255,0,0));
	cv::circle(drawing, corners[1], 5, cv::Scalar(0,255,0));
	cv::circle(drawing, corners[2], 5, cv::Scalar(0,0,255));
	cv::circle(drawing, corners[3], 5, cv::Scalar(255,255,255));
	}
		
		cv::line(drawing, *topLeft, *bottomLeft, cv::Scalar(255,255,255));
		cv::line(drawing, *bottomLeft, *bottomRight, cv::Scalar(255,255,255));
		cv::line(drawing, *bottomRight, *topRight, cv::Scalar(255,255,255));
		cv::line(drawing, *topRight, *topLeft, cv::Scalar(255,255,255));

	return drawing;
}

int main(){

		grip::GripPipeline pipeline;
	// camera setup
	int cWidth = 320;
	int cHeight = 240;
	int cExposure = 2;
	int cWhiteBalance = 5100;
	cs::UsbCamera camera = frc::CameraServer::GetInstance()->StartAutomaticCapture();
	camera.SetResolution(cWidth,cHeight);
	camera.SetExposureManual(cExposure);
	camera.SetWhiteBalanceManual(cWhiteBalance);
	cs::CvSink cvSink = frc::CameraServer::GetInstance()->GetVideo();
	cs::CvSource outputStreamStd = frc::CameraServer::GetInstance()->PutVideo("hsvoutput", cWidth,cHeight);
	cs::CvSource outputStreamRectStd = frc::CameraServer::GetInstance()->PutVideo("countouroutput", cWidth,cHeight);
	cv::Mat source;
	cv::Mat output;
	cv::Mat* output_ptr;
	char input_variable;
	bool exit_loop = false;
	cv::Mat* hsv_mat_ptr;
	cv::Mat hsv_mat;
	std::vector<std::vector<cv::Point> >* contours_ptr;
	std::vector<std::vector<cv::Point> > contours;
//network tables setup
	nt::NetworkTableEntry entry;
	auto inst = nt::NetworkTableInstance::GetDefault();
	auto table = inst.GetTable("table");
	while( exit_loop == false )
        {
	std::cout << "Waiting to start camera capture " << std::endl;
	std::cin >>  input_variable;
  int counter = 0;
  while(true) {
  	cvSink.GrabFrame(source);
  	if ( source.rows > 0){
	       if ( input_variable == 's' )
	       {
						pipeline.Process(source);
						hsv_mat_ptr = pipeline.GetHsvThresholdOutput();
						hsv_mat = *hsv_mat_ptr;
						outputStreamStd.PutFrame(hsv_mat);
						std::string path = "~/RP_Vision_2019/VisionTarget/Image_" + std::to_string(counter) + ".png";
						//std::cout <<  cv::imwrite( path , source) << std::endl;
						// DO something with the contours from GRIP Pipeline now.
						contours_ptr = pipeline.GetFilterContoursOutput();
						contours = *contours_ptr;
						//std::cout <<  contours.size() << std::endl;
						std::string path3 = "~/RP_Vision_2019/VisionTarget/Image_HSV" + std::to_string(counter) + ".png"; 
						// DO something with the contours from GRIP Pipeline now.
						//std::cout << cv::imwrite( path3 , hsv_mat) << std::endl;  						
						if(!contours.empty()){
							cv::Mat rect_output = detect_rectangles(source,contours);
							std::string path2 = "~/RP_Vision_2019/VisionTarget/Image_Rect_" + std::to_string(counter) + ".png"; 
							//std::cout << cv::imwrite( path2 , rect_output) << std::endl;
							outputStreamRectStd.PutFrame(rect_output);
						}
						counter++;
						
					}
	    }
        
	}
	}
	return 0;


};
