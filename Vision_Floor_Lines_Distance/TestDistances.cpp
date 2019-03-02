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

//#include <FloorLines_Distance.cpp>
#include <Distance_Calibration.h>

int main()
{

    std::cout << calc_Distance(60, small_yCoord) << std::endl;

    std::cout << calc_Distance(-100, small_yCoord) << std::endl;
    
    std::cout << calc_Distance(180, small_yCoord) << std::endl;

    system("pause");
    return 0;
}