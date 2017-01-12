#include <iostream>

#include <memory>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>


int main() {
    std::cout << "Hello, World!" << std::endl;


    cv::VideoCapture cap("/dev/video0");
    cv::Mat in_img;
    cv::Mat * out_img_ptr;
    std::string win_name("debug");

    cv::namedWindow(const_cast<char *> (win_name.c_str()));

    cv::waitKey(10);


    cv::aruco::Dictionary dic =(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_100)) ;
    cv::Ptr<cv::aruco::Dictionary> dic_ptr(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_100));
//    dic_ptr = &dic;
//    std::shared_ptr<cv::aruco::Dictionary> dic_ptr(dic);



    while (cap.isOpened()) {

        cap >> in_img;
        out_img_ptr = &in_img;
//        ids.clear();
//        corner.clear();
        std::vector<std::vector<cv::Point2f>> corner;
        std::vector<int> ids;
        cv::aruco::detectMarkers(in_img,dic_ptr,corner,ids);
//        cv::aruco::drawAxis(out_img,)

        if(ids.size()>0)
        {
//            for(int i(0);i<ids.size();++i)
            {
//                cv::aruco::drawMarker(dic_ptr,ids[i],10,in_img,1);
                cv::aruco::drawDetectedMarkers(in_img,corner,ids);
            }
//            cv::aruco::drawMarker(dic,ids[0],2,out_img);
        }


//        out_img = in_img;

        cv::imshow(win_name, in_img);
        cvWaitKey(10);
    }


    return 0;
}