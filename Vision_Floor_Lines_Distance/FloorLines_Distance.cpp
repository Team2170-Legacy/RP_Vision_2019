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

#include "Distance_Calibration.h"

double calc_Distance(double yCoord, double yCoordArr[], int hArrSize){
	int arrSize = hArrSize;
	double distance = 0;
	if(yCoord > yCoordArr[0] || yCoord < yCoordArr[arrSize]){
		//too far or too close  
		return -1;
	}
	for(int i = 0; i < arrSize; i++){
		// Search from highest distance to lowest distance
		// As index increases, distance increases and y-coordinates decrease
		if(yCoord < yCoordArr[i] && yCoord > yCoordArr[i + 1]){ 
			distance = ((yCoord - yCoordArr[i+1])/*How much over in percentage*//(yCoordArr[i] - yCoordArr[i+1]))*(distances[i+1] - distances[i])/*finds distance over*/ + distances[i+1]/*distance before*/;
		}
	}
	return distance;


}

int main()
{


    std::cout << calc_Distance(60, small_yCoord, table_size) << std::endl;

    //std::cout << calc_Distance(-100, small_yCoord) << std::endl;
    
    //std::cout << calc_Distance(180, small_yCoord) << std::endl;

	return 0;
}
