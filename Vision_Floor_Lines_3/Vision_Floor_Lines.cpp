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
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"


//-----------------------------------------------------------------------------------------------------------
// As index increases, distance increases and y-coordinates decrease
int table_size = 4;
double distances[] = {0, 1, 2, 3};
double small_yCoord[] = {115.0, 69.2, 37.5, 14.2};
double medium_yCoord[] = {230.0, 138.3, 75.0, 28.3};
double big_yCoord[] = {690.0, 415.0, 225.0, 85.0};

//-----------------------------------------------------------------------------------------------------------
double calc_Distance(double yCoord, double yCoordArr[], int hArrSize){
	int arrSize = hArrSize;
	double distance = 0;
	if(yCoord > yCoordArr[0] || yCoord < yCoordArr[arrSize]){
		//too far or too close  
		return -1;
	}
	for(int i = 0; i < arrSize - 1; i++){
		// Search from highest distance to lowest distance
		// As index increases, distance increases and y-coordinates decrease
		if( (yCoord < yCoordArr[i]) && (yCoord > yCoordArr[i + 1]) )
                { 
			distance = ((yCoord - yCoordArr[i+1])/*How much over in percentage*//(yCoordArr[i] - yCoordArr[i+1]))*(distances[i+1] - distances[i])/*finds distance over*/ + distances[i+1]/*distance before*/;
		}
	}
	return distance;


}


//-----------------------------------------------------------------------------------------------------------
// original 2019-03-05 version
//double calc_Angle(int xt, int yt, int xb, int yb){
//    double angle;
//
//
//   if(xt > xb){
//        //leaning towards right
//        angle = 90 - atan((xt-xb)/(yt-yb));
//        angle = angle * -1;
//    } else if(xb > xt){
//        angle = 90 - atan((xb-xt)/(yt-yb));
//    } else{
//        angle = 0;
//    }

    // MK 2019-03-07 version
double calc_Angle(double xt, double yt, double xb, double yb){

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

//-----------------------------------------------------------------------------------------------------------
int main()

{
        //    static void VisionThread()

        //    {

        int     debug = 1;              // debug flag, set 1 when additional output requested to console output
        bool    run_once_flag = false;   // set to true to run main loop only ONCE for debug 

        int width       = /*320;*/       160;
        int height      = /*240;*/       120;
        int targetMidpoint_x            = width / 2;    // middle of the image, replace later with target locking during auto
        int prev_targetMidpoint_x       = targetMidpoint_x;     // previous target x, used to lock on target during automove steering

        grip::GripPipeline pipeline;
        cs::UsbCamera camera = frc::CameraServer::GetInstance()->StartAutomaticCapture();

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

        //#include "Distance.h"
        
        bool            target_lock = false;
        int             target_y = 0;
        int             target_x = 0;
        int             target_error = 0;
        int             target_Distance = 0;
        double          target_angle = 0;
        uint64_t        grab_Frame_Status = 0;

        //-------------------------------------------------------------------------------------------------------------
        //network tables setup

	unsigned int port = 1735;
	auto inst = nt::NetworkTableInstance::GetDefault();
	//inst.StartServer("10.21.70.2");
	std::cout << "server started" << std::endl;
	inst.StartClient("10.21.70.2",port);
	auto table = inst.GetTable("VisionTable");

        nt::NetworkTableEntry fl_target_error           = table->GetEntry("fl_target_error");
        nt::NetworkTableEntry fl_target_Distance_nt     = table->GetEntry("fl_target_Distance");
        nt::NetworkTableEntry fl_target_lock            = table->GetEntry("fl_target_lock");
        nt::NetworkTableEntry fl_target_angle_nt        = table->GetEntry("fl_target_angle");

        nt::NetworkTableEntry automove                  = table->GetEntry("automove");
        automove.SetBoolean(false);
        bool automove_flag = false;                     // true if automove button is being pressed

        fl_target_error.SetDouble(target_error);
        fl_target_Distance_nt.SetDouble(target_Distance);
        fl_target_lock.SetDouble(target_lock);
        fl_target_angle_nt.SetDouble(target_angle);

        //-----------------------------------------------------------------------------------------------------------
        while (!run_once_flag) // loop forever
        {
                bool automove_temp =  table->GetBoolean("automove",false);
                automove_flag = automove_temp;

                target_lock = false;

                grab_Frame_Status = cvSink.GrabFrame(source);
                while (run_once_flag && !grab_Frame_Status)
                {
                        grab_Frame_Status = cvSink.GrabFrame(source);
                }

                if (debug)
                        std::cout << "Grab Frame Status: " << grab_Frame_Status << std::endl;

                if (grab_Frame_Status > 0)
                {
                        output = source;
                        if (source.rows > 0)
                        {
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
                                std::vector<cv::RotatedRect> rotatedRectArray;
                                cv::Point2f rect_points[4]; 
                                cv::Scalar RED          = cv::Scalar(0, 0, 255);        //BGR Red 
                                cv::Scalar WHITE        = cv::Scalar(255, 255, 255);    // BGR White
                                cv::Scalar YELLOW       = cv::Scalar(0, 255, 255);      // BGR Yellow
                                cv::Scalar BLUE         = cv::Scalar(255, 0, 0);        // BGR Blue
                                int THICKNESS_WHITE     = 1;
                                int THICKNESS_RED       = 4;

                                int num_contours = contours.size();
                                if (debug)
                                        std::cout << std::endl;
                                        std::cout << "Found # contours (" << num_contours << ")" << std::endl;
                                int midpointBox[num_contours]; // remove later
                                cv::Point topmidpoint[num_contours];  // array of the mid point of the rotated rectangles top line
                                cv::Point bottommidpoint[num_contours];  // array of the mid point of the rotated rectangles bottom line
                                int minimum_index = 0;

                                if (num_contours>=1)
                                {
                                        target_lock = true;

                                        for (int count = 0; count < num_contours; count++)
                                        {
                                                //boundingBoxArray.push_back(cv::boundingRect(contours[count]));
                                                rotatedRectArray.push_back(cv::minAreaRect(contours[count]));
                                                //
                                                //code below was for boundingBoxArray
                                                //int midx = ((boundingBoxArray[count].tl()).x + (boundingBoxArray[count].br()).x) / 2;
                                                //int midy = ((boundingBoxArray[count].tl()).y + (boundingBoxArray[count].br()).y) / 2;
                                                //midpointBox[count] = midx;
                                                //
                                        
                                                // draw white outlines for all rotated rectangles found

                                                
                                                rotatedRectArray[count].points( rect_points );
                                                for( int j = 0; j < 4; j++ )
                                                {
                                                        cv::line( output, rect_points[j], rect_points[(j+1)%4], YELLOW, THICKNESS_WHITE, 8 );
                                                }

                                                // find the mid points of the top edge of the rotated rectangles
                                                // i.e. this line is defined by the two points with the smallest y-coordinates

                                                int y_min1      = 9999;
                                                int y_min2      = 9999;
                                                int ind_min1    = -1;
                                                int ind_min2    = -1;
                                                // find highest y-coord point
                                                for( int j = 0; j < 4; j++ )
                                                {
                                                        if (rect_points[j].y < y_min1)
                                                        {
                                                                y_min1          = rect_points[j].y;
                                                                ind_min1        = j;
                                                        }
                                                }
                                                // find NEXT highest y-coord point
                                                for( int j = 0; j < 4; j++ )
                                                {
                                                        if ( (rect_points[j].y < y_min2) && ( j != ind_min1 ) )
                                                        {
                                                                y_min2          = rect_points[j].y;
                                                                ind_min2        = j;
                                                        }
                                                }

                                                // not sure if above finding routine works
                                                // debug with overrides
                                                //ind_min1=1;
                                                //ind_min2=2;


                                                if ( debug )
                                                {
                                                        std::cout << "rect_points[0] = " << rect_points[0] << std::endl;
                                                        std::cout << "rect_points[1] = " << rect_points[1] << std::endl;
                                                        std::cout << "rect_points[2] = " << rect_points[2] << std::endl;
                                                        std::cout << "rect_points[3] = " << rect_points[3] << std::endl;
                                                        std::cout << "ymin1 = (" << y_min1 << ")" << std::endl;
                                                        std::cout << "ymin2 = (" << y_min2<< ")" << std::endl;
                                                        std::cout << "ind_min1 = (" << ind_min1 << ")" << std::endl;
                                                        std::cout << "ind_min2 = (" << ind_min2 << ")" << std::endl;
                                                }
                                                // store the top line midpoint
                                                topmidpoint[count].x    = 0.5*(rect_points[ind_min1].x + rect_points[ind_min2].x );
                                                topmidpoint[count].y    = 0.5*(rect_points[ind_min1].y + rect_points[ind_min2].y );
                                                

                                                // calculate and store the bottom line midpoint
                                                // first calculate the indices of the bottom line points
                                                int     ind_min3 = -1;
                                                int     ind_min4 = -1;

                                                for( int j = 0; j < 4; j++ )
                                                {
                                                        if ( (j != ind_min1) && (j != ind_min2) )
                                                                ind_min3 = j;
                                                }

                                                for( int j = 0; j < 4; j++ )
                                                {
                                                        if ( (j != ind_min1) && (j != ind_min2) && (j != ind_min3) )
                                                                ind_min4 = j;
                                                }

                                                bottommidpoint[count].x    = 0.5*(rect_points[ind_min3].x + rect_points[ind_min4].x );
                                                bottommidpoint[count].y    = 0.5*(rect_points[ind_min3].y + rect_points[ind_min4].y );


                                        }  //  for (int count = 0; count < num_contours; count++)



                                        // calculate distances of the found top line midpoints to the middle of the image, 
                                        // replace later with the target locking during auto
                                        int differenceMidpoint[num_contours];

                                        // implement targetlock HOLD when automove BUTTON is pressed,
                                        // i.e. Locked target point is the line CLOSEST to the previous target point,
                                        // i.e. follow the locked line around as it moves around the image
                                        // this is done to be able to handle that line moving AWAY from the image mid point
                                        // when performing wall alignment control
                                        //
                                        if ( automove_flag )
                                        {
                                                targetMidpoint_x = prev_targetMidpoint_x;
                                        }
                                        else
                                        {
                                                targetMidpoint_x = width / 2;
                                        }
                                        


                                        for (int count = 0; count < num_contours; count++)
                                        {
                                                differenceMidpoint[count] = abs(targetMidpoint_x - topmidpoint[count].x);
                                        }

                                        // find which rectangle has it's top midpoint closest to target
                                        for (int count = 1; count < num_contours; count++)
                                        {
                                                if (differenceMidpoint[count] < differenceMidpoint[minimum_index])
                                                {
                                                        minimum_index = count;
                                                }
                                        }

                                        if (debug)
                                        {
                                                std::cout << "Midpoint contour #: (" << minimum_index << ")" << std::endl;
                                                std::cout << "Midpoint x: (" << topmidpoint[minimum_index].x << ")" << std::endl;
                                        }

                                        // MK 2019-03-06        Skip drawing the BIG bounding box when switching to rotated rectangles bounding boxes
                                        //cv::rectangle(output, boundingBoxArray[minimum_index].tl(), boundingBoxArray[minimum_index].br(), RED, THICKNESS_RED, 8, 0);

                                        target_x = topmidpoint[minimum_index].x;
                                        target_y = topmidpoint[minimum_index].y;

                                        //Draw a THICK circles at TOP and BOTTOM
                                        int circle_radius = 5;
                                        cv::circle(output, topmidpoint[minimum_index], circle_radius, RED, THICKNESS_RED, 8, 0);
                                        cv::circle(output, bottommidpoint[minimum_index], circle_radius, BLUE, THICKNESS_RED, 8, 0);

                                        //-----------------------------------------------------------------------------------------------------------
                                        //for( int i = 0; i < num_contours; i++ )
                                        //{
                                        //cv::Scalar color = WHITE;
                                        //if (0 ) // debug )
                                        //{
                                        //        // contour
                                        //        cv::drawContours( output, contours, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point() );
                                        //}


                                        // draw rotated target lock rectangle in RED, other rectangles already drawn in WHITE
                                        cv::Point2f rect_points[4]; 
                                        
                                        rotatedRectArray[minimum_index].points( rect_points );
                                        for( int j = 0; j < 4; j++ )
                                                cv::line( output, rect_points[j], rect_points[(j+1)%4], RED, THICKNESS_RED, 8 );
                                        //}
        //-----------------------------------------------------------------------------------------------------------


                                        prev_targetMidpoint_x   = target_x;
                                         
                                        //for (int otherBox = 0; otherBox < num_contours; otherBox++)
                                        //{
                                        //        if (otherBox != minimum_index)
                                        //        {
                                        //                cv::rectangle(output, boundingBoxArray[otherBox].tl(), boundingBoxArray[otherBox].br(), WHITE, THICKNESS_WHITE, 8, 0);
                                        //        }
                                       // }

                                }  // if (num_contours>=1)

                                if ( target_lock ) 
                                {
                                        // for 160x120 images
                                        //target_Distance = calc_Distance(target_y, small_yCoord,4);

                                        // for 320x240 images
                                        target_Distance = calc_Distance(target_y, medium_yCoord,4);
                                        // MK TO BE UPDATED target_angle = calc_Angle (boundingBoxArray[minimum_index].tl().x, boundingBoxArray[minimum_index].tl().y,boundingBoxArray[minimum_index].br().x, boundingBoxArray[minimum_index].br().y  );
                                        target_angle = calc_Angle (target_x, target_y, bottommidpoint[minimum_index].x , bottommidpoint[minimum_index].y  );
                                        //target_angle = 99.0;
                                }
                                else
                                {
                                        target_Distance = 0;
                                        target_angle    = 0;
                                }
                                
        
                        } // if ( source.rows > 0)

                        outputStreamStd.PutFrame(output);
                        if (debug) 
                                 std::cout << "Streaming output frame...." << std::endl;

                        if(target_lock = true) 
                        {
                                target_error = target_x - targetMidpoint_x;
	                        fl_target_error.SetDouble(target_error);
	                        fl_target_Distance_nt.SetDouble(target_Distance);
                                fl_target_lock.SetDouble(target_lock);
        	                fl_target_angle_nt.SetDouble(target_angle);
			}
                        
                        
                } // if(grab_Frame_Status > -1)
        } // while(true)
        return 0;
};