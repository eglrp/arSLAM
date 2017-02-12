//
// Created by steve on 17-1-13.
//



#include <iostream>


#include <opencv2/videoio.hpp>


#include <ArPoseFrame.h>



int main()
{
    bool record_video(false);

//    cv::VideoCapture cap("/dev/video1");
//    cap.set(CV_CAP_PROP_FRAME_WIDTH,1280);
//    cap.set(CV_CAP_PROP_FRAME_HEIGHT,720);
//    cap.set(CV_CAP_OPENNI_QVGA_60HZ, 30.0);

    cv::Mat in;

    cv::aruco::DetectorParameters parameters;
    parameters.create();

    cv::aruco::Dictionary dic = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_100);
    ArPoseFrame arPoseFrame(11);


//        cv::VideoWriter videoWriter("log.avi",0,30,cv::Size(1280,720),true);
    std::string log_file_name("./dataset/log4.avi");
    cv::VideoCapture capfile(log_file_name);
    while(capfile.isOpened())
    {
//        cap >> in;
        capfile >>in;


        if(in.empty())
        {
            capfile.release();
            capfile.open(log_file_name);
            capfile >> in;
        }
//   videoWriter << in;
//        cv::imshow("test",in);
        arPoseFrame.ProcessImg(in);
        cv::waitKey(10);
    }

}



