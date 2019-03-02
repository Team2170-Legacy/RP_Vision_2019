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

// data about locked contours, updated whenever a target is locked on too
double targetWidth = 0;
int targetCenterX = 0;
bool targetLocked = false;

/*
void write_log(string message)
{
	// Open and write the message to a file using input output operations 
	std::ofstream myfile;
		fs.open ("/home/pi/RP_Vision_2019/VisionTarget/Log.txt", std::fstream::in | std::fstream::out | std::fstream::app);
		myfile << message + "\n";
		myfile.close();
}
*/

cv::Mat detect_rectangles(cv::Mat source, std::vector<std::vector<cv::Point>> contours, cv::Scalar color, int thickness, bool updateLockedTargetData)
{

	cv::Mat drawing = source;
	if (contours.size() < 2)
	{
		return drawing;
	}
	cv::Point2f* topLeft;
	cv::Point2f* bottomLeft;
	cv::Point2f* topRight;
	cv::Point2f* bottomRight;
	std::vector<std::vector<float>> bounding;

	for (int c = 0; c < 2; c++)
	{
		cv::RotatedRect boundingBox = cv::minAreaRect(contours[c]);
		cv::Point2f corners[4];
		boundingBox.points(corners);
		cv::line(drawing, corners[0], corners[1], cv::Scalar(255, 255, 255));
		cv::line(drawing, corners[1], corners[2], cv::Scalar(255, 255, 255));
		cv::line(drawing, corners[2], corners[3], cv::Scalar(255, 255, 255));
		cv::line(drawing, corners[3], corners[0], cv::Scalar(255, 255, 255));
		float MinY = fminf(corners[0].y, fminf(corners[1].y, fminf(corners[2].y, corners[3].y)));
		float MaxY = fmaxf(corners[0].y, fmaxf(corners[1].y, fmaxf(corners[2].y, corners[3].y)));
		float MinX = fminf(corners[0].x, fminf(corners[1].x, fminf(corners[2].x, corners[3].x)));
		float MaxX = fmaxf(corners[0].x, fmaxf(corners[1].x, fmaxf(corners[2].x, corners[3].x)));
		std::vector<float> tmp;
		tmp.push_back(MinX);
		tmp.push_back(MaxX);
		tmp.push_back(MinY);
		tmp.push_back(MaxY);
		bounding.push_back(tmp);
	}

	float MinX = fminf(bounding.at(0).at(0), bounding.at(1).at(0));
	float MaxX = fmaxf(bounding.at(0).at(1), bounding.at(1).at(1));
	float MinY = fminf(bounding.at(0).at(2), bounding.at(1).at(2));
	float MaxY = fmaxf(bounding.at(0).at(3), bounding.at(1).at(3));
	topLeft = new cv::Point2f(MinX, MaxY);
	bottomLeft = new cv::Point2f(MinX, MinY);
	topRight = new cv::Point2f(MaxX, MaxY);
	bottomRight = new cv::Point2f(MaxX, MinY);
	cv::line(drawing, *topLeft, *bottomLeft, color, thickness);
	cv::line(drawing, *bottomLeft, *bottomRight, color, thickness);
	cv::line(drawing, *bottomRight, *topRight, color, thickness);
	cv::line(drawing, *topRight, *topLeft, color, thickness);
	cv::circle(drawing, cv::Point2f((MinX + MaxX) / 2, (MinY + MaxY) / 2), 5, cv::Scalar(255, 255, 255));
	
	if(updateLockedTargetData) {
	 targetCenterX = ((MinX + MaxX) / 2);
	 targetWidth = (double)(MaxX - MinX);
	targetLocked = true;
	}


	return drawing;
}

cv::Mat lock_target(cv::Mat source, std::vector<std::vector<cv::Point>> contours, int width)
{
	int num_contours = contours.size();
	int midpointBox[num_contours];
	std::vector<cv::Rect> boundingBoxArray;
	int minimum = 0;
	int minimum2 = 0;
	if (num_contours >= 2) 
	{


		for (int count = 0; count < num_contours; count++) {
			cv::RotatedRect boundingBox = cv::minAreaRect(contours[count]);
			cv::Point2f corners[4];
			boundingBox.points(corners);
			float MinX = fminf(corners[0].x, fminf(corners[1].x, fminf(corners[2].x, corners[3].x)));
			float MaxX = fmaxf(corners[0].x, fmaxf(corners[1].x, fmaxf(corners[2].x, corners[3].x)));
			int midx = (MinX + MaxX) / 2;
			midpointBox[count] = midx;
		}
		
		int differenceMidpoints[num_contours];
		for (int count = 0; count < num_contours; count++) {
			differenceMidpoints[count] = abs((width / 2) - midpointBox[count]);
		}

		for (int count = 1; count < num_contours; count++) {
			if (differenceMidpoints[count] < differenceMidpoints[minimum]) 
				minimum = count;
			}

			for (int count = 1; count < num_contours; count++) {
				if (differenceMidpoints[minimum2] > differenceMidpoints[count] && count != minimum)
					minimum2 = count;

		
	}
	}
	else {
		return source;
	}
	if (num_contours > 2) {
		for (int count = 0; count < num_contours - 1; count += 2) {
				std::vector<std::vector<cv::Point>> all_contours;
				all_contours.push_back(contours.at(count));
				all_contours.push_back(contours.at(count + 1));
				source = detect_rectangles(source, all_contours, cv::Scalar(255, 255, 255), 1, false);
			
		}
		std::vector<std::vector<cv::Point>> center_contours;
		center_contours.push_back(contours.at(minimum));
		center_contours.push_back(contours.at(minimum2));
		source = detect_rectangles(source, center_contours, cv::Scalar(0, 0, 255), 4, true);
		
	}
	else if (num_contours == 2)
	{
		std::vector<std::vector<cv::Point>> center_contours;
		center_contours.push_back(contours.at(0));
		center_contours.push_back(contours.at(1));
		source = detect_rectangles(source, center_contours, cv::Scalar(0, 0, 255), 4, true);
	}
	else {

	targetLocked = false;
		return source;
	}

	return source;
}

// given a bounding box's width, calculates the distance from the targets in the bounding box
/*
double calcDistance(double width) {
double estimatedDistance = 10;
// widths[0] is width at 10ft away, widths[1] is 9ft away, widths[2] is 8ft away, ... till width[9] is 1 ft away

	int index;
	for(int i = 9; i>=0; i--) 
	{
		if(width>widths[i]) {
			index = i;
		break;
		}
	}
	write_log("index " + index);
	if(index!=0) {
int difference = widths[index]-widths[index-1];
double decimal = (width-widths[index-1])/(difference);
 estimatedDistance = (10 - (index + decimal));
 	write_log("estimate " + estimatedDistance);
	}
	else {
	return estimatedDistance;
}

	return estimatedDistance;
}
*/

double calcDistance(double width){
		double yCoordArr [11] = {44, 48, 53, 62, 68, 80, 96, 128, 150, 180, 320};
		double distances[] = {10.0, 9.0, 8.0, 7.0, 6.0, 5.0, 4.0, 3.0, 2.0, 1.0, 0.0};
		double yCoord = width;
	int arrSize = 11;
	double distance = 0;
	if( yCoord > yCoordArr[arrSize - 1]){
		//too close
		return 0;
	}
		if( yCoord < yCoordArr[0]){
		return 10;
	}

	for(int i = 0; i < arrSize -1; i++){
		// Search from longest distance (lowest y) to shortest distance (highest y)
		if(yCoord > yCoordArr[i] && yCoord < yCoordArr[i + 1]){ //yCoordArr goes in order, distance is flipped
			distance = ((yCoordArr[i +1] - yCoord)/(yCoordArr[i+1] - yCoordArr[i]))*(distances[i] - distances[i+1]) + distances[i+1];
		}
	}
	return distance;


}

int main() {
	grip::GripPipeline pipeline;
	
	// camera setup
	int cWidth = 320;
    int cHeight = 240;
	int cExposure = 2;
	int cWhiteBalance = 5100;
	cs::UsbCamera camera = frc::CameraServer::GetInstance()->StartAutomaticCapture();
	camera.SetResolution(cWidth, cHeight);
	camera.SetExposureManual(cExposure);
	camera.SetWhiteBalanceManual(cWhiteBalance);
	cs::CvSink cvSink = frc::CameraServer::GetInstance()->GetVideo();
	cs::CvSource outputStreamStd = frc::CameraServer::GetInstance()->PutVideo("hsvoutput", cWidth, cHeight);
	cs::CvSource outputStreamRectStd = frc::CameraServer::GetInstance()->PutVideo("countouroutput", cWidth, cHeight);
	cv::Mat source;
	cv::Mat output;
	cv::Mat* output_ptr;
	cv::Mat* hsv_mat_ptr;
	cv::Mat hsv_mat;
	std::vector<std::vector<cv::Point> >* contours_ptr;
	std::vector<std::vector<cv::Point> > contours;

	//network tables setup
	unsigned int port = 1735;
	auto inst = nt::NetworkTableInstance::GetDefault();
	inst.StartServer("169.254.247.86");
	std::cout << "server started" << std::endl;
	inst.StartClient("169.254.160.228",port);
	auto table = inst.GetTable("VisionTable");
	
	while (true) {
		cvSink.GrabFrame(source);
		if (source.rows > 0) {
		// outputStreamRectStd.PutFrame(source);
			pipeline.Process(source);
			hsv_mat_ptr = pipeline.GetHsvThresholdOutput();
			hsv_mat = *hsv_mat_ptr;
			outputStreamStd.PutFrame(hsv_mat);
			contours_ptr = pipeline.GetFilterContoursOutput();
			contours = *contours_ptr;
			if (!contours.empty()) {
				cv::Mat rect_output = lock_target(source, contours, cWidth);
				outputStreamRectStd.PutFrame(rect_output);

			}
			else
			{
				targetLocked = false;
				outputStreamRectStd.PutFrame(source);
			}
  nt::NetworkTableEntry x_target_error =  table->GetEntry("x_target_error");
 nt::NetworkTableEntry target_locked =  table->GetEntry("target_locked");
 target_locked.SetBoolean(targetLocked);
			if(targetLocked = true) {
    
	x_target_error.SetDouble(targetCenterX - (cWidth/2));
	  nt::NetworkTableEntry distance_to_target =  table->GetEntry("distance_to_target");
	distance_to_target.SetDouble(calcDistance(targetWidth));
	//std::cout << calcDistance(targetWidth) << std::endl;
	
			}
			else {
				x_target_error.SetDouble(0);

			}
		}
	}
	
	return 0;


};