//
// Created by steve on 17-1-13.
//



#include <iostream>


#include <opencv2/videoio.hpp>


#include <ArPoseFrame.h>


int main()
{
    cv::VideoCapture cap("/dev/video0");
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(CV_CAP_OPENNI_QVGA_60HZ, 60.0);

    cv::Mat in;
    while(cap.isOpened())
    {
        cap >> in;
        cv::imshow("test",in);
        cv::waitKey(10);
    }

}



