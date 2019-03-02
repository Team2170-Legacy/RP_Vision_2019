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

//160x180 (small)

double distances[] = {3.0, 1.0, 2.0, 0.0};
double small_yCoord[] = {115.0, 69.2, 37.5, 14.2};
double medium_yCoord[] = {230.0, 138.3, 75.0, 28.3};
double big_yCoord[] = {690.0, 415.0, 225.0, 85.0};
