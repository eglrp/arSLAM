//
// Created by steve on 17-1-13.
//



#include <iostream>


#include <opencv2/videoio.hpp>


#include <ArPoseFrame.h>


int main()
{
    cv::VideoCapture cap("/dev/video1");
//    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
//    cap.cap.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
    cap.set(CV_CAP_PROP_FRAME_WIDTH,1280);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,720);
//    cap.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
//    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);
    cap.set(CV_CAP_OPENNI_QVGA_60HZ, 60.0);
//    cap.set(CV_CAP_OPENNI_QVGA_30HZ,30);

    cv::Mat in;

    cv::aruco::DetectorParameters parameters;
    parameters.create();

    cv::aruco::Dictionary dic = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_100);
    ArPoseFrame arPoseFrame(11);
    while(cap.isOpened())
    {
        cap >> in;
//        cv::imshow("test",in);
        arPoseFrame.ProcessImg(in);
        cv::waitKey(10);
    }

}



