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
#include <fstream>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
bool save_image;
bool allow_exit;
int counter = 0;

//cv::Mat detect_rectangles(cv::Mat source, std::vector<std::vector<cv::Point>> contours,int counter)
cv::Mat detect_rectangles(cv::Mat source, std::vector<std::vector<cv::Point>> contours)
{
	cv::Mat drawing = cv::Mat::zeros( source.size(), CV_8UC3 );
	//cv::imwrite("/home/pi/RP_Vision_2019/VisionTarget/images/drawing.jpg",drawing);
	drawing = source;
	if (contours.size() < 2)
	{
		return drawing;
	}	
	cv::Point2f* topLeft;
	cv::Point2f* bottomLeft;
	cv::Point2f* topRight;
	cv::Point2f* bottomRight;
	std::vector<std::vector<float>> bounding;
    // compute the rotated bounding rect of the biggest contour! (this is the part that does what you want/need)
	for ( int c = 0; c < 2; c++)
	{
		cv::RotatedRect boundingBox = cv::minAreaRect(contours[c]);
		// one thing to remark: this will compute the OUTER boundary box, so maybe you have to erode/dilate if you want something between the ragged lines
		// draw the rotated rect
		cv::Point2f corners[4];
		boundingBox.points(corners);
		float MinY = fminf(corners[0].y,fminf(corners[1].y,fminf(corners[2].y,corners[3].y)));
		float MaxY = fmaxf(corners[0].y,fmaxf(corners[1].y,fmaxf(corners[2].y,corners[3].y)));
		float MinX = fminf(corners[0].x,fminf(corners[1].x,fminf(corners[2].x,corners[3].x)));
		float MaxX=  fmaxf(corners[0].x,fmaxf(corners[1].x,fmaxf(corners[2].x,corners[3].x)));
		std::vector<float> tmp;
		tmp.push_back(MinX);
		tmp.push_back(MaxX);
		tmp.push_back(MinY);
		tmp.push_back(MaxY);
		bounding.push_back(tmp);
	}
	float MinX = fminf(bounding.at(0).at(0),bounding.at(1).at(0));
	float MaxX = fmaxf(bounding.at(0).at(1),bounding.at(1).at(1));
	float MinY = fminf(bounding.at(0).at(2),bounding.at(1).at(2));
	float MaxY = fmaxf(bounding.at(0).at(3),bounding.at(1).at(3));
	topLeft = new cv::Point2f(MinX, MaxY);
	bottomLeft = new cv::Point2f(MinX, MinY);
	topRight = new cv::Point2f(MaxX, MaxY);
	bottomRight = new cv::Point2f(MaxX, MinY);
	if( save_image == true) 
	{
		std::cout <<  "Saving Image:" << std::endl;
		cv::imwrite("/home/pi/RP_Vision_2019/Calibration/images/"+ std::to_string(counter) + ".jpg",source);
		std::cout << "/home/pi/RP_Vision_2019/Calibration/images/"+ std::to_string(counter) + ".jpg" << std::endl;
		std::ofstream myfile;
		myfile.open ("/home/pi/RP_Vision_2019/Calibration/images/"+ std::to_string(counter) + ".txt");
		myfile << "Width of Target: " + std::to_string(MaxX - MinY) + "\n";
		myfile.close();
		counter++;
	}
	save_image = false;
	
	
	return drawing;
}
void command_input()
{
	std::string input_variable;
	 while(true) {
		//std::cout << "Input S to Save image" << std::endl;
		std::cin >>  input_variable;
		if ( input_variable == "S" || input_variable == "s")
		{
			//std::cout << "Setting Save image to true" << std::endl;
			save_image = true;
		}
	 }
}

int main(){
	std::thread console_thread(command_input);
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
	cv::Mat* hsv_mat_ptr;
	cv::Mat hsv_mat;
	//int counter = 0;
	std::vector<std::vector<cv::Point> >* contours_ptr;
	std::vector<std::vector<cv::Point> > contours;
	//network tables setup
	nt::NetworkTableEntry entry;
	auto inst = nt::NetworkTableInstance::GetDefault();
	auto table = inst.GetTable("table");
  	while(allow_exit == false) {
 	 	cvSink.GrabFrame(source);
		if ( source.rows > 0){
			pipeline.Process(source);
			hsv_mat_ptr = pipeline.GetHsvThresholdOutput();
			hsv_mat = *hsv_mat_ptr;
			outputStreamStd.PutFrame(hsv_mat);
			contours_ptr = pipeline.GetFilterContoursOutput();
			contours = *contours_ptr;
			if(!contours.empty() ){
				cv::Mat rect_output = detect_rectangles(source,contours);
				outputStreamRectStd.PutFrame(rect_output);

			}
			else
			{
				outputStreamRectStd.PutFrame(source);
			}
		}        
	}
	 console_thread.join();
	return 0;


};
