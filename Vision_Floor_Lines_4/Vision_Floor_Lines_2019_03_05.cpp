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
/*class Distance{
    public:
    float calc_Distance(int y) 
    {
        return 2.33;
    }
};
*/
// As index increases, distance increases and y-coordinates decrease
int table_size = 4;
double distances[] = {0, 1, 2, 3};
double small_yCoord[] = {115.0, 69.2, 37.5, 14.2};
double medium_yCoord[] = {230.0, 138.3, 75.0, 28.3};
double big_yCoord[] = {690.0, 415.0, 225.0, 85.0};

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
		if(yCoord < yCoordArr[i] && yCoord > yCoordArr[i + 1]){ 
			distance = ((yCoord - yCoordArr[i+1])/*How much over in percentage*//(yCoordArr[i] - yCoordArr[i+1]))*(distances[i+1] - distances[i])/*finds distance over*/ + distances[i+1]/*distance before*/;
		}
	}
	return distance;


}



double calc_Angle(int xt, int yt, int xb, int yb){
    double angle;

    if(xt > xb){
        //leaning towards right
        angle = 90 - atan((xt-xb)/(yt-yb));
        angle = angle * -1;
    } else if(xb > xt){
        angle = 90 - atan((xb-xt)/(yt-yb));
    } else{
        angle = 0;
    }

    return angle;
}

int main()
//69
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

        //#include "Distance.h"
        
        bool target_lock;
        int target_y = 0;
        int target_x = 0;
        int target_error = 0;
        int target_Distance = 0;
        double target_angle = 0;
        uint64_t grab_Frame_Status = 0;

        //-------------------------------------------------------------------------------------------------------------
        //      For testing purposes
        //
        // read in image file
        //    Mat image;
        //source = cv::imread("Test_Image", CV_LOAD_IMAGE_COLOR);
        //grab_Frame_Status = 1;
        //
        //     if(! source.data )
        //     {
        //             std::cout <<  "Could not open or find the image" << std::endl ;
        //             return -1;
        //     }
        //-------------------------------------------------------------------------------------------------------------

        while (true) // loop forever
        {
                target_lock = false;
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
                                        target_lock = true;
                                        int midxr = ((boundingBoxArray[minimum].tl()).x + (boundingBoxArray[minimum].br()).x) / 2;
                                        //Draw a THICC circle
                                        cv::Point midpoint(midxr, ((boundingBoxArray[minimum].tl()).y));
                                        cv::circle(output, midpoint, 5, RED, THICKNESS_RED, 8, 0);

                                        target_x = midxr;
                                        target_y = ((boundingBoxArray[minimum].tl()).y);
                                         
                                        cv::Scalar WHITE = cv::Scalar(255, 255, 255);
                                        for (int otherBox = 0; otherBox < num_contours; otherBox++)
                                        {
                                                if (otherBox != minimum)
                                                {
                                                        cv::rectangle(output, boundingBoxArray[otherBox].tl(), boundingBoxArray[otherBox].br(), WHITE, THICKNESS_WHITE, 8, 0);
                                                }
                                        }

                                }
                                
                                 
                        
                        
                                target_Distance = calc_Distance(target_y, small_yCoord,4);
                                target_angle = calc_Angle (boundingBoxArray[minimum].tl().x, boundingBoxArray[minimum].tl().y,boundingBoxArray[minimum].br().x, boundingBoxArray[minimum].br().y  );
        
                                
                                 // if (num_contours>1)
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

                                target_error = target_x-(width/2);
                                if (debug) 
                                        std::cout << "Streaming output frame...." << std::endl;

                        
        //network tables setup
	unsigned int port = 1735;
	auto inst = nt::NetworkTableInstance::GetDefault();
	//inst.StartServer("10.21.70.2");
	std::cout << "server started" << std::endl;
	inst.StartClient("10.21.70.2",port);
	auto table = inst.GetTable("VisionTable");

        if(target_lock = true) {
      nt::NetworkTableEntry fl_target_error =  table->GetEntry("fl_target_error");
	fl_target_error.SetDouble(target_x - (width/2));
	  nt::NetworkTableEntry fl_target_Distance_nt =  table->GetEntry("fl_target_Distance");
	fl_target_Distance_nt.SetDouble(target_Distance);
	
        nt::NetworkTableEntry fl_target_lock = table->GetEntry("fl_target_lock");
        fl_target_lock.SetDouble(target_lock);
			}
	  nt::NetworkTableEntry fl_target_angle_nt =  table->GetEntry("fl_target_angle");
	fl_target_angle_nt.SetDouble(target_angle);
                        
                        
                } // if(grab_Frame_Status > -1)
        } // while(true)

        //    std::thread visionThread(VisionThread);

        //    visionThread.detach();

        return 0;
};
