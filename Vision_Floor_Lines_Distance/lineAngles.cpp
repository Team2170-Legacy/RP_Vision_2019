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

#include <math.h> 


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

int main(){
        //std::cout << calc_Angle() << std::endl;

        return 0;
}