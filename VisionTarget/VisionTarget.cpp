
#include <cscore.h>
#include <cscore_cpp.h>
#include <cameraserver/CameraServer.h>
#include <cameraserver/CameraServerShared.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

#include <thread>
#include <chrono>

#include <cmath>

#include "GripPipeline.h"
#include <string>
#include <iostream>
#include <fstream>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/EntryListenerFlags.h"

#include <time.h>

//----------------------------------------------------------------------------------------------

// start delay in seconds
double start_delay = 60.0;

// constant global color values
const cv::Scalar blue = (255, 0 ,0);
const cv::Scalar green = (0, 255, 0);
const cv::Scalar red = (0, 0, 255);
const cv::Scalar white = (255, 255, 255);
const cv::Scalar lockColor = red;

// constant global exposure values
const int detectionExposure = 1;
const int drivingExposure = 20;

const bool cameraIsUpsideDown = false; // for when someone screws up and the image needs to be rotated
const bool debug = false; // set to true for debugging: additional video streams, couts, etc.

int cWidth = 320; // camera width
int cHeight = 240; // camera height

float contourDetectionX = cWidth/2;; // the contour closest to this x is locked on to

bool automove_flag = false; // whether or not automove is on            

int arrSize = 9;
// double yCoordArr [9] = {6,   7,   8,   10,  12,  15,  20,  29,  45};
double yCoordArr [9] = {12,   14,   16,   20,  24,  30,  40,  58,  90};
double distances[] =   {8.0, 7.0, 6.0, 5.0, 4.0, 3.0, 2.0, 1.0, 0.0};


// data about locked contours, updated whenever a target is locked on too
double 	targetWidth = 0; // width of bounding box of locked vision targets
double 	targetHeight = 0; // height of bounding box of locked vision targets
int targetCenterX = 0; // x value of center coordinate of bounding box around locked vision targets
bool targetLocked = false; // whether or not a target is currently locked
double 	leftTapeHeight = 0; // height of left tape in locked target
double 	rightTapeHeight = 0; // height of right tape in locked target
double leftTapeAngle = 0; // angle of left tape in locked target
double rightTapeAngle = 0; // angle of right tape in locked target
double leftTapeArea = 0; // area of left tape in locked target
double rightTapeArea = 0; // area of right tape in locked target
double distanceToTarget = distances[0]; // distance to locked target

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
    else  // leaning towards the left
    {
        angle = +180.0/pi * ( atan( abs(del_x)/abs(del_y)) );
    }


    return angle;
} // double calc_Angle

//----------------------------------------------------------------------------------------------

// checks if 2 contours are a valid grouping using their angles and aspect ratio
bool contoursAreValid(std::vector<std::vector<cv::Point>> contours) 
{
std::vector<std::vector<float>> bounding;

if(contours.size()!=2)
return false;

double contourAreas [2] = {0, 0};
double contourAngles [2] = {0, 0};
float contourMinXs [2] = {0, 0};
float contourMaxXs [2] = {0, 0};
float contourMinYs [2] = {0, 0};
float contourMaxYs [2] = {0, 0};

for(int c = 0; c < 2; c++) 
{
		cv::RotatedRect boundingBox = cv::minAreaRect(contours[c]);
		contourAreas[c] = cv::contourArea(contours[c]);
		cv::Point2f corners[4];
		boundingBox.points(corners);
		
		float MinX = fminf(corners[0].x, fminf(corners[1].x, fminf(corners[2].x, corners[3].x)));
		float MaxX = fmaxf(corners[0].x, fmaxf(corners[1].x, fmaxf(corners[2].x, corners[3].x)));
		float MinY = fminf(corners[0].y, fminf(corners[1].y, fminf(corners[2].y, corners[3].y)));
		float MaxY = fmaxf(corners[0].y, fmaxf(corners[1].y, fmaxf(corners[2].y, corners[3].y)));
	
	    contourMinXs[c] = MinX;
	    contourMaxXs[c] = MaxX;
		contourMinYs[c] = MinY;
	    contourMaxYs[c] = MaxY;

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

double ltAngle = 0;
double rtAngle = 0;
float leftCenterX = 0;
float rightCenterX = 0;

if(contourMinXs[0]<contourMinXs[1]) {
	ltAngle = contourAngles[0];
	rtAngle = contourAngles[1];
	leftCenterX = (contourMinXs[0] + contourMaxXs[0])/2;
	rightCenterX = (contourMinXs[1] + contourMaxXs[1])/2;
	}
	else 
	{
    ltAngle = contourAngles[1];
	rtAngle = contourAngles[0];
	leftCenterX = (contourMinXs[1] + contourMaxXs[1])/2;
	rightCenterX = (contourMinXs[0] + contourMaxXs[0])/2;
	}

float MinX = fminf(contourMinXs[0], contourMinXs[1]);
float MaxX = fmaxf(contourMaxXs[0], contourMaxXs[1]);
float MinY = fminf(contourMinYs[0], contourMinYs[1]);
float MaxY = fmaxf(contourMaxYs[0], contourMaxYs[1]);

float boundingWidth = MaxX - MinX;
float boundingHeight = MaxY - MinY;
float actualAspectRatio = boundingWidth/boundingHeight;
float optimalAspectRatio = 2.25;
float aspectRatioMargin = 0.75;

if(abs(actualAspectRatio - optimalAspectRatio) > aspectRatioMargin)
{
return false;
}

// only bother checking angles if the pixel area of the contours is above 10	
// if(contourAreas[0] > 8 && contourAreas[1] > 8)
 // {
if(rtAngle<0 || ltAngle>0)
{
return false;
}
 // }

return true;
} //bool contoursAreValid

//----------------------------------------------------------------------------------------------

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
	double contourAreas [2] = {0, 0};
	float contourMinXs [2] = {0 , 0};
	for (int c = 0; c < 2; c++)
	{
		cv::RotatedRect boundingBox = cv::minAreaRect(contours[c]);
		contourAreas[c] = cv::contourArea(contours[c]);
		cv::Point2f corners[4];
		boundingBox.points(corners);
		// 0 is topleft, goes counter - clockwise from there
		cv::line(drawing, corners[0], corners[1], white);
		cv::line(drawing, corners[1], corners[2], white);
		cv::line(drawing, corners[2], corners[3], white);
		cv::line(drawing, corners[3], corners[0], white);
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
	
	
	if(updateLockedTargetData) {
	cv::circle(drawing, cv::Point2f((MinX + MaxX) / 2, (MinY + MaxY) / 2), 5, white);
	 targetCenterX = ((MinX + MaxX) / 2);
	 targetWidth = (double)(MaxX - MinX);
	 targetHeight = (double)(MaxY - MinY);
	 targetLocked = true;
	 contourDetectionX = targetCenterX;
	if(contourMinXs[0]<contourMinXs[1]) {
	leftTapeHeight = contourHeights[0];
	rightTapeHeight = contourHeights[1];
	leftTapeAngle = contourAngles[0];
	rightTapeAngle = contourAngles[1];
	leftTapeArea = contourAreas[0];
	rightTapeArea = contourAreas[1];
	}
	else {
	leftTapeHeight = contourHeights[1];
	rightTapeHeight = contourHeights[0];
	leftTapeAngle = contourAngles[1];
	rightTapeAngle = contourAngles[0];
	leftTapeArea = contourAreas[1];
	rightTapeArea = contourAreas[0];
	}
}

	return drawing;
} //cv::Mat detect_rectangles

//----------------------------------------------------------------------------------------------

// sorts contours left to right based on their center x values
std::vector<std::vector<cv::Point>> sortContours(std::vector<std::vector<cv::Point>> contours) 
{
int num_contours = contours.size();
int contourOrder[num_contours];
    
	// populate contour order array with values 0 to num_contours-1
	for(int i = 0; i<num_contours; i++)
	{
		contourOrder[i] = i;
	}
  
int contourXs[num_contours];

    // populate contourXs array with the contour center x values
	for (int count = 0; count < num_contours; count++) 
	{
		cv::RotatedRect boundingBox = cv::minAreaRect(contours[count]);
		cv::Point2f corners[4];
		boundingBox.points(corners);
		float MinX = fminf(corners[0].x, fminf(corners[1].x, fminf(corners[2].x, corners[3].x)));
		float MaxX = fmaxf(corners[0].x, fmaxf(corners[1].x, fmaxf(corners[2].x, corners[3].x)));
		int midx = (MinX + MaxX) / 2;
		contourXs[count] = midx;
	}

    // selection sort 
    // move boundary of unsorted subarray 
    for (int i = 0; i < num_contours-1; i++) 
    { 
        // find the minimum element in unsorted array 
        int min_idx = i; 
        for (int j = i+1; j < num_contours; j++)
		{ 
          if (contourXs[j] < contourXs[min_idx]) 
		  {
            min_idx = j; 
		  }
		}
  
        // swap the found minimum element with the first element 
		int temp = contourXs[min_idx];
		contourXs[min_idx] = contourXs[i];
        contourXs[i] = temp; 
        
		// make the same swap in the contour order array
		int temp2 = contourOrder[min_idx];
		contourOrder[min_idx] = contourOrder[i];
		contourOrder[i] = temp2;
    }

// populate outputContours array with contours but in correct order
std::vector<std::vector<cv::Point>> outputContours; 
for(int i = 0; i < num_contours; i++)
{
outputContours.push_back(contours.at(contourOrder[i]));
}

return outputContours;
} //cv::Mat sortContours

//----------------------------------------------------------------------------------------------


// identifies the valid target closest to the contour detection point and locks onto it
cv::Mat lock_target(cv::Mat source, std::vector<std::vector<cv::Point>> contours)
{
	// if not moving autonomously use center as the contour detection point
	if(!automove_flag)
	{
	contourDetectionX = cWidth/2;
	}
    else 
	{
    // draws a blue circle where the contour detection point is
	cv::circle(source, cv::Point2f(contourDetectionX, cHeight/2), 5, cv::Scalar(255, 191, 0));
	}

	int num_contours = contours.size();

    if(num_contours < 2)
	{
	// when there are not 2 contours in view nothing is locked onto
	return source; 
	}
	else if(num_contours == 2)
	{
		// when there is only 2 contours in view lock onto them if they are valid
		std::vector<std::vector<cv::Point>> center_contours;
		center_contours.push_back(contours.at(0));
		center_contours.push_back(contours.at(1));
		if(contoursAreValid(center_contours))
		{
		source = detect_rectangles(source, center_contours, lockColor, 2, true);
		}
	}
	else if (num_contours > 2) 
	{
		// sort contours left to right
		contours = sortContours(contours);

		int midpointBox[num_contours];

        // populate midpointBox arr with midpoints of contours
		for (int count = 0; count < num_contours; count++) 
		{
			cv::RotatedRect boundingBox = cv::minAreaRect(contours[count]);
			cv::Point2f corners[4];
			boundingBox.points(corners);
			float MinX = fminf(corners[0].x, fminf(corners[1].x, fminf(corners[2].x, corners[3].x)));
			float MaxX = fmaxf(corners[0].x, fmaxf(corners[1].x, fmaxf(corners[2].x, corners[3].x)));
			int midx = (MinX + MaxX) / 2;
			midpointBox[count] = midx;
		}

        

		// draw over all valid contours in white
		for(int count = 0; count < num_contours - 1; count++)
		{
			std::vector<std::vector<cv::Point>> all_contours;
			all_contours.push_back(contours.at(count));
			all_contours.push_back(contours.at(count + 1));
			if(contoursAreValid(all_contours))
			{
				source = detect_rectangles(source, all_contours, white, 1, false);			 
			}

		}
		
		 int groupContourDistances[num_contours-1];
		 
		 // populate groupContourDistances arr with high values
		 for(int index = 0; index < num_contours-1; index++) 
		 {
		  groupContourDistances[index] = 100000;
		 }
         
         // get distance from contour detection point for each valid target
		 for(int i = 0; i<num_contours-1; i++)
		 {

				std::vector<std::vector<cv::Point>> group_contours;
	            group_contours.push_back(contours.at(i));
		        group_contours.push_back(contours.at(i+1));
				
				 if(contoursAreValid(group_contours))
				 {
					groupContourDistances[i] = round(abs(contourDetectionX - ((midpointBox[i] + midpointBox[i+1])/2)));
				 }
		 }
    
		 // get valid target that has the least distance from contour detection point
		 int minIndex = 0;
		 for(int b = 0; b < num_contours-1; b++) 
		 {
			if(groupContourDistances[b] < groupContourDistances[minIndex])
			{
				minIndex = b;
			}
		 }

		std::vector<std::vector<cv::Point>> locked_contours;
	    locked_contours.push_back(contours.at(minIndex));
		locked_contours.push_back(contours.at(minIndex+1));
		if(contoursAreValid(locked_contours))
		{
        source = detect_rectangles(source, locked_contours, lockColor, 2, true);
		}
	
	}
		 
	return source;

} // cv:Mat lock_target

//----------------------------------------------------------------------------------------------

// given height of the bounding box of a locked target calulates the distance in feet the robot is away from the target
double calcDistance(double height){
	
	if(debug) 
	{
	std::cout << "TargetHeight: " + std::to_string(height);
	}

	    double distance = 0;
      

    double defaultValue = distances[0];
	if(automove_flag)
    {
	defaultValue = distanceToTarget;
	}

	double yCoord = height;
		if( yCoord > yCoordArr[arrSize - 1])
		{
			return 0;
		}
		if(yCoord < 0) 
		{
			return 0;
		}
		if( yCoord < yCoordArr[0])
		{
			return defaultValue;
		}

	for(int i = 0; i < arrSize -1; i++)
	{
		// Search from longest distance (lowest y) to shortest distance (highest y)
		if(yCoord > yCoordArr[i] && yCoord < yCoordArr[i + 1]){ //yCoordArr goes in order, distance is flipped
			distance = ((yCoordArr[i +1] - yCoord)/(yCoordArr[i+1] - yCoordArr[i]))*(distances[i] - distances[i+1]) + distances[i+1];
		}
	}

if(debug) 
{
	std::cout <<" Distance: " + std::to_string(distance) << std::endl;
}
	return distance;
} // double calc_Distance

//----------------------------------------------------------------------------------------------


// calculates tape align error using tape areas
double calcTapeAlignError() 
{

/*
double scaleDown = 10;
double distanceToTapes = calcDistance(targetHeight);
double tapeAreaDiff = leftTapeArea - rightTapeArea;
double tapeAreaDiff = tapeAreaDiff * distanceToTapes * distanceToTapes;
double CTAE = tapeAreaDiff/scaleDown;
return CTAE;
*/

//return (1-(rightTapeArea/leftTapeArea));
//return (leftTapeArea-rightTapeArea)/leftTapeArea;

return (leftTapeArea - rightTapeArea);

} // double getTapeAlignError

//----------------------------------------------------------------------------------------------

int main() {

	std::cout << "Delaying for " + std::to_string(start_delay) << std::endl;
	double dt = 0.0;
	clock_t start_time = clock();

	while(dt<start_delay)
	{
		dt = ((double)clock() - (double)start_time)/(double)CLOCKS_PER_SEC;

	}

	std::cout << "end delay" << std::endl;

	grip::GripPipeline pipeline;
	
	// camera setup
	int camera_dev_number = 0;
	//int cExposure = detectionExposure;
	int cExposure = drivingExposure;
	int cWhiteBalance = 5100;
	int fps = 30;

    cs::UsbCamera *camera_pointer = new cs::UsbCamera("USB Camera 0", camera_dev_number);
    cs::UsbCamera camera = *camera_pointer;
	camera.SetResolution(cWidth, cHeight);
	camera.SetExposureManual(cExposure);
	camera.SetWhiteBalanceManual(cWhiteBalance);

/*
	cs::MjpegServer *mjpegServer1_pointer = new cs::MjpegServer("Forward Camera", 5805);
    cs::MjpegServer mjpegServer1 = *mjpegServer1_pointer;
    mjpegServer1.SetSource(camera);
*/
	cs::CvSink *cvSink_pointer = new cs::CvSink("Vision Target USB Camera");
    cs::CvSink cvSink = *cvSink_pointer;
    cvSink.SetSource(camera);
   
    cs::MjpegServer *mjpegServer2_pointer = new cs::MjpegServer("HSV Stream", 5806);
    cs::MjpegServer mjpegServer2 = *mjpegServer2_pointer;

    cs::MjpegServer *mjpegServer3_pointer = new cs::MjpegServer("Vision Target Output Stream", 5807);
    cs::MjpegServer mjpegServer3 = *mjpegServer3_pointer;

    cs::CvSource *outputStreamStd_pointer = new cs::CvSource("Vision Target", cs::VideoMode::PixelFormat::kMJPEG, cWidth, cHeight, fps);
    cs::CvSource outputStreamStd = *outputStreamStd_pointer;
    mjpegServer3.SetSource(outputStreamStd);

    cs::CvSource *outputStreamHSV_pointer = new cs::CvSource("HSV Target Stream", cs::VideoMode::PixelFormat::kMJPEG, cWidth, cHeight, fps);
    cs::CvSource outputStreamHSV = *outputStreamHSV_pointer;
    mjpegServer2.SetSource(outputStreamHSV);


	cv::Mat source;
	cv::Mat output;
	cv::Mat* output_ptr;
	cv::Mat* hsv_mat_ptr;
	cv::Mat hsv_mat;
	std::vector<std::vector<cv::Point> >* contours_ptr;
	std::vector<std::vector<cv::Point> > contours;

	//---------------------------------------------------------------------------------------------

	//network tables setup
	unsigned int port = 1735;
	auto inst = nt::NetworkTableInstance::GetDefault();
	std::cout << "server started" << std::endl;
	inst.StartClient("10.21.70.2",port);
	auto table = inst.GetTable("VisionTable");

	nt::NetworkTableEntry left_tape_angle = table->GetEntry("left_tape_angle");
	nt::NetworkTableEntry right_tape_angle = table->GetEntry("right_tape_angle");
	nt::NetworkTableEntry left_tape_height =  table->GetEntry("left_tape_height");
    nt::NetworkTableEntry right_tape_height =  table->GetEntry("right_tape_height");
	nt::NetworkTableEntry distance_to_target =  table->GetEntry("distance_to_target");
	nt::NetworkTableEntry x_target_error =  table->GetEntry("x_target_error");
	nt::NetworkTableEntry left_tape_area = table->GetEntry("left_tape_area");
	nt::NetworkTableEntry right_tape_area = table->GetEntry("right_tape_area");
	nt::NetworkTableEntry tape_align_error = table->GetEntry("tape_align_error");
    nt::NetworkTableEntry target_locked =  table->GetEntry("target_locked");
	
	nt::NetworkTableEntry exposure =  table->GetEntry("exposure");
	exposure.SetDouble(cExposure);

    nt::NetworkTableEntry vt_exposure_flag =  table->GetEntry("vt_exposure_flag");
	vt_exposure_flag.SetBoolean(false);
    bool vtExposureFlag = false;

    nt::NetworkTableEntry automove = table->GetEntry("automove");
    automove.SetBoolean(false);
   
   //----------------------------------------------------------------------------------------------
   
	while (true) {
		vtExposureFlag = vt_exposure_flag.GetBoolean("vt_exposure_flag");
		automove_flag = automove.GetBoolean("automove");
		std::cout << "checkpoint 1" << std::endl;
		cvSink.GrabFrame(source);
		std::cout << "checkpoint 2" << std::endl;
		if(cameraIsUpsideDown) 
		{
		 // flips image
         cv::flip(source, source, -1);
		}
		if (source.rows > 0) {
			if(cExposure!=drivingExposure)
			{

			
				std::cout << "checkpoint 3" << std::endl;
			pipeline.Process(source);
				std::cout << "checkpoint 4" << std::endl;
			if ( debug )
			{
				hsv_mat_ptr = pipeline.GetHsvThresholdOutput();
				hsv_mat = *hsv_mat_ptr;
				outputStreamHSV.PutFrame(hsv_mat);
			};
		
			contours_ptr = pipeline.GetFilterContoursOutput();
			contours = *contours_ptr;
			if(debug)
			{
            std::cout << std::to_string(contours.size()) + " Contours Detected" << std::endl;
			}
			if (contours.size()>=2) { 
				cv::Mat rect_output = lock_target(source, contours);
				outputStreamStd.PutFrame(rect_output);
			}
			else
			{
				targetLocked = false;
			}
  
 		
 			target_locked.SetBoolean(targetLocked);

			if(targetLocked) {
                distanceToTarget = calcDistance(targetHeight);
				x_target_error.SetDouble(targetCenterX - (cWidth/2));
				distance_to_target.SetDouble(distanceToTarget);
				left_tape_height.SetDouble(leftTapeHeight);
				right_tape_height.SetDouble(rightTapeHeight);
				left_tape_angle.SetDouble(leftTapeAngle);
				right_tape_angle.SetDouble(rightTapeAngle);
                left_tape_area.SetDouble(leftTapeArea);
				right_tape_area.SetDouble(rightTapeArea);
				tape_align_error.SetDouble(calcTapeAlignError());

                if(debug)
				 {
                /*
				std::cout << "leftTapeAngle: ";
				std::cout << leftTapeAngle << std::endl;
				std::cout << "rightTapeAngle: ";
				std::cout << rightTapeAngle << std::endl;
				*/

			    std::cout << "tapeAreaError: ";
				std::cout << calcTapeAlignError() << std::endl;
				std::cout << "leftTapeArea: ";
				std::cout << leftTapeArea << std::endl;
				std::cout << "rightTapeArea: ";
				std::cout << rightTapeArea << std::endl;
				}

			}
			else {
				outputStreamStd.PutFrame(source);
			    distance_to_target.SetDouble(0);
				x_target_error.SetDouble(0);
				left_tape_height.SetDouble(0);
				right_tape_height.SetDouble(0);
				right_tape_area.SetDouble(0);
				left_tape_area.SetDouble(0);
				left_tape_angle.SetDouble(0);
				right_tape_angle.SetDouble(0);
				tape_align_error.SetDouble(0);
			}
            
			//cExposure	= table->GetNumber("exposure",1);
			if(vtExposureFlag)
			{
			cExposure = detectionExposure;
			camera.SetExposureManual(cExposure);
			std::cout << "         exposure set to " + cExposure << std::endl;
			}
			else
			{
		    cExposure = drivingExposure;
			camera.SetExposureManual(cExposure);
			std::cout << "         exposure set to " + cExposure << std::endl;
			}
			
		}
		else 
		{
			outputStreamStd.PutFrame(source);
	
		}
		} 
	} 
	
	return 0;


};