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
#include "networktables/EntryListenerFlags.h"

//----------------------------------------------------------------------------------------------

bool		debug = true; // set to true for debugging: additional video streams etc.

// data about locked contours, updated whenever a target is locked on too
double 	targetWidth = 0; // width of bounding box of locked vision targets
double 	targetHeight = 0; // height of bounding box of locked vision targets
int 		targetCenterX = 0; // x value of center coordinate of bounding box around locked vision targets
bool 		targetLocked = false; // whether or not a target is currently locked
double 	leftTapeHeight = 0; // height of left tape in locked target
double 	rightTapeHeight = 0; // height of right tape in locked target
double leftTapeAngle = 0; // angle of left tape in locked target
double rightTapeAngle = 0; // angle of right tape in locked target
double tapeAngleDifference = 0;

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

//----------------------------------------------------------------------------------------------

double calc_Angle(double xt, double yt, double xb, double yb) {
    double angle = 0;
    double del_x = xt - xb;
    double del_y = yt - yb;
    double pi = 3.14159;

    if(xt > xb) // leaning towards right
    {
        angle = -180.0/pi * ( atan( abs(del_x)/abs(del_y)) );
    } 
    else  // leadning towards the left
    {
        angle = +180.0/pi * ( atan( abs(del_x)/abs(del_y)) );
    }


    return angle;
}

cv::Mat detect_rectangles(cv::Mat source, std::vector<std::vector<cv::Point>> contours, cv::Scalar color, int thickness, bool updateLockedTargetData)
{
    std::vector<std::vector<float>> bounding;

	cv::Mat drawing = source;
	if (contours.size() < 2)
	{
		return drawing;
	}

	cv::Point2f* topLeft;
	cv::Point2f* bottomLeft;
	cv::Point2f* topRight;
	cv::Point2f* bottomRight;
	cv::Point2f* bottomMidpt;
	cv::Point2f* topMidpt;

	

	double contourHeights [2] = {0 , 0};
	double contourAngles [2] = {0, 0};
	float contourMinXs [2] = {0 , 0};
	for (int c = 0; c < 2; c++)
	{
		cv::RotatedRect boundingBox = cv::minAreaRect(contours[c]);
		cv::Point2f corners[4];
		boundingBox.points(corners);
		// 0 is topleft, goes counter - clockwise from there
		cv::line(drawing, corners[0], corners[1], cv::Scalar(255, 255, 255));
		cv::line(drawing, corners[1], corners[2], cv::Scalar(255, 255, 255));
		cv::line(drawing, corners[2], corners[3], cv::Scalar(255, 255, 255));
		cv::line(drawing, corners[3], corners[0], cv::Scalar(255, 255, 255));
		float MinY = fminf(corners[0].y, fminf(corners[1].y, fminf(corners[2].y, corners[3].y)));
		float MaxY = fmaxf(corners[0].y, fmaxf(corners[1].y, fmaxf(corners[2].y, corners[3].y)));
		float MinX = fminf(corners[0].x, fminf(corners[1].x, fminf(corners[2].x, corners[3].x)));
		float MaxX = fmaxf(corners[0].x, fmaxf(corners[1].x, fmaxf(corners[2].x, corners[3].x)));
		contourHeights[c] = (double)(MaxY - MinY);
		contourMinXs [c] = MinX;
		std::vector<float> tmp;
		tmp.push_back(MinX);
		tmp.push_back(MaxX);
		tmp.push_back(MinY);
		tmp.push_back(MaxY);
		bounding.push_back(tmp);


if(updateLockedTargetData) {  
	
	float y_min1 = 100000;
	int ind_min1 = 0;
	float y_min2 = 100000;
	int ind_min2 = 0;
	float y_max1 = 0;
	int ind_max1 = 0;
	float y_max2 = 0;
	int ind_max2 = 0;
	
	for( int j = 0; j < 4; j++ )
	{
			if (corners[j].y < y_min1)
			{
					y_min1          = corners[j].y;
					ind_min1        = j;
			}
	}
	// find NEXT highest y-coord point
	for( int j = 0; j < 4; j++ )
	{
			if ( (corners[j].y < y_min2) && ( j != ind_min1 ) )
			{
					y_min2          = corners[j].y;
					ind_min2        = j;
			}
	}
	for( int j = 0; j < 4; j++ )
	{
			if (corners[j].y > y_max1)
			{
					y_max1          = corners[j].y;
					ind_max1        = j;
			}
	}
	// find NEXT lowest y-coord point
	for( int j = 0; j < 4; j++ )
	{
			if ( (corners[j].y > y_max2) && ( j != ind_max1 ) )
			{
					y_max2          = corners[j].y;
					ind_max2        = j;
			}
	}


	float bx = (corners[ind_max1].x+corners[ind_max2].x)/2;
	float by = (corners[ind_max1].y+corners[ind_max2].y)/2;
	float tx = (corners[ind_min1].x+corners[ind_min2].x)/2;
	float ty = (corners[ind_min1].y+corners[ind_min2].y)/2;
    contourAngles[c] = calc_Angle(tx, ty, bx, by);
}

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
	 targetHeight = (double)(MaxY - MinY);
	 targetLocked = true;
	if(contourMinXs[0]<contourMinXs[1]) {
	leftTapeHeight = contourHeights[0];
	rightTapeHeight = contourHeights[1];
	leftTapeAngle = contourAngles[0];
	rightTapeAngle = contourAngles[1];
	}
	else {
	leftTapeHeight = contourHeights[1];
	rightTapeHeight = contourHeights[0];
	leftTapeAngle = contourAngles[1];
	rightTapeAngle = contourAngles[0];
	}
tapeAngleDifference = leftTapeAngle - rightTapeAngle;
}

	return drawing;
} //cv::Mat detect_rectangles

//----------------------------------------------------------------------------------------------

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
} //cv::Mat lock_target

//----------------------------------------------------------------------------------------------
// given height of the bounding box of a locked target calulates the distance in feet the robot is away from the target
double calcDistance(double height){
	std::cout << "ContourHeight: " + std::to_string(height)  << std::endl;
		double yCoordArr [11] = {8, 9, 10, 11, 13, 15, 18, 22, 31, 50, 160};
		double distances[] = {10.0, 9.0, 8.0, 7.0, 6.0, 5.0, 4.0, 3.0, 2.0, 1.0, 0.0};
		double yCoord = height;
	int arrSize = 11;
	double distance = 0;
	if( yCoord > yCoordArr[arrSize - 1]){
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


} // double calcDistance

//----------------------------------------------------------------------------------------------


int main() {
	grip::GripPipeline pipeline;
	
	// camera setup
	int cWidth = 160;
    int cHeight = 120;

	int cExposure = 2;
	int cWhiteBalance = 5100;

  int exposure_test_counter = 0;
	int cExposure_temp  = 15;

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

	//----------------------------------------------------------------------------------------------

	//network tables setup
	unsigned int port = 1735;
	auto inst = nt::NetworkTableInstance::GetDefault();
	//inst.StartServer("10.21.70.2");
	std::cout << "server started" << std::endl;
	inst.StartClient("10.21.70.2",port);
	auto table = inst.GetTable("VisionTable");

    nt::NetworkTableEntry tape_angle_difference = table->GetEntry("tape_angle_difference");
	nt::NetworkTableEntry left_tape_angle = table->GetEntry("left_tape_angle");
	nt::NetworkTableEntry right_tape_angle = table->GetEntry("right_tape_angle");
	nt::NetworkTableEntry left_tape_height =  table->GetEntry("left_tape_height");
    nt::NetworkTableEntry right_tape_height =  table->GetEntry("right_tape_height");
	nt::NetworkTableEntry distance_to_target =  table->GetEntry("distance_to_target");
	nt::NetworkTableEntry x_target_error =  table->GetEntry("x_target_error");
    nt::NetworkTableEntry target_locked =  table->GetEntry("target_locked");
	nt::NetworkTableEntry exposure =  table->GetEntry("exposure");

	//----------------------------------------------------------------------------------------------
	// camera exposure esttings
	//----------------------------------------------------------------------------------------------

	
	exposure.SetDouble(2);
	//nt::NetworkTableEntry exposure2 =  table->GetEntry("exposure2");

	// the example below had JAVA syntax, wait converting this, use the getvalue differently
	//exposure.addListener(event -> {cexposure_temp = event.value.getValue();},
	//	EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);


	//----------------------------------------------------------------------------------------------
	while (true) {
		cvSink.GrabFrame(source);
		if (source.rows > 0) {
		// outputStreamRectStd.PutFrame(source);
			pipeline.Process(source);
			if ( debug )
			{
				hsv_mat_ptr = pipeline.GetHsvThresholdOutput();
				hsv_mat = *hsv_mat_ptr;
				outputStreamStd.PutFrame(hsv_mat);
			};
			contours_ptr = pipeline.GetFilterContoursOutput();
			contours = *contours_ptr;
			if (contours.size()>=2) {
				cv::Mat rect_output = lock_target(source, contours, cWidth);
				outputStreamRectStd.PutFrame(rect_output);

			}
			else
			{
				targetLocked = false;
				target_locked.SetBoolean(false);
				outputStreamRectStd.PutFrame(source);
				distance_to_target.SetDouble(0);
				x_target_error.SetDouble(0);
				left_tape_height.SetDouble(0);
				right_tape_height.SetDouble(0);
			}
  
 		
 			target_locked.SetBoolean(targetLocked);
			if(targetLocked) {
				x_target_error.SetDouble(targetCenterX - (cWidth/2));
	  		
				distance_to_target.SetDouble(calcDistance(targetHeight));
				
				left_tape_height.SetDouble(leftTapeHeight);
				right_tape_height.SetDouble(rightTapeHeight);
				tape_angle_difference.SetDouble(tapeAngleDifference);
				left_tape_angle.SetDouble(leftTapeAngle);
				right_tape_angle.SetDouble(rightTapeAngle);
				std::cout << "tapeAngleDifference: ";
				std::cout << tapeAngleDifference << std::endl;
				std::cout << "leftTapeAngle: ";
				std::cout << leftTapeAngle << std::endl;
				std::cout << "rightTapeAngle: ";
				std::cout << rightTapeAngle << std::endl;
			}
			else {
				x_target_error.SetDouble(0);
				left_tape_height.SetDouble(0);
				right_tape_height.SetDouble(0);
				tape_angle_difference.SetDouble(0);
			}

			// Expsoure testing
 			

			//if (exposure_test_counter > 50)
			//{
			//	exposure_test_counter = 0;

				// testing before network tables:
				//camera.SetExposureManual(cExposure_temp);
				//cExposure_temp = 8 - cExposure_temp;

				// Use code below for reading the exposure values from network table
			//}
			//else
			//	exposure_test_counter = exposure_test_counter + 1;
				
			cExposure_temp 	= table->GetNumber("exposure",15);
			//std::cout<<cExposure_temp<<std::endl;
			camera.SetExposureManual(cExposure_temp);
			

			
		} 
	} 
	
	return 0;


};