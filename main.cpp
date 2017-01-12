#include <iostream>
#include <fstream>

#include <memory>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>


#include <Eigen/Dense>


int main() {
    std::cout.precision(20);
    std::cout << "Hello, World!" << std::endl;


    cv::VideoCapture cap("/dev/video0");
    cv::Mat in_img;
    cv::Mat *out_img_ptr;
    std::string win_name("debug");

    cv::namedWindow(const_cast<char *> (win_name.c_str()));

    cv::waitKey(10);


    cv::aruco::Dictionary dic = (cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_100));
    cv::Ptr<cv::aruco::Dictionary> dic_ptr(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_100));


    cv::Mat3d intrinsic_matrix;
    cv::Mat distortion_matrix;
    distortion_matrix.resize(5);

    std::fstream tfs("./data/distortion_matrix.txt");
    for (int k(0); k < 5; ++k) {
        double ttt;
        tfs >> ttt;
        distortion_matrix.push_back(ttt);
    }

//
    std::fstream ifs;
    ifs.open("./data/intrinsic_matrix.txt");
    std::cout << "isf:" << ifs.is_open() << std::endl;
    for (int ii(0); ii < 3; ++ii) {
        for (int j(0); j < 3; ++j) {
            double d_tmp(0.0);
            ifs >> d_tmp;
            intrinsic_matrix.push_back(d_tmp);
        }
    }

//    for (int x(0); x < intrinsic_matrix.rows; ++x) {
//        for (int y(0); y < intrinsic_matrix.cols; ++y) {
//            std::cout << intrinsic_matrix.at<double>(x, y) << std::endl;
//        }
//    }




//    dic_ptr = &dic;
//    std::shared_ptr<cv::aruco::Dictionary> dic_ptr(dic);



    while (cap.isOpened()) {

        cap >> in_img;
        out_img_ptr = &in_img;
        std::cout <<"here" << std::endl;
        std::vector<std::vector<cv::Point2f>> corner;
        std::vector<int> ids;
        cv::aruco::detectMarkers(in_img, dic_ptr, corner, ids);
//        cv::aruco::drawAxis(out_img,)

        if (ids.size() > 0) {
//            for(int i(0);i<ids.size();++i)
            {
//                cv::aruco::drawMarker(dic_ptr,ids[i],10,in_img,1);
                cv::aruco::drawDetectedMarkers(in_img, corner, ids);
            }

            std::cout << "eee" << std::endl;
            std::vector<cv::Vec3f> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(corner,
                                                 201,
                                                 intrinsic_matrix,
                                                 distortion_matrix,
                                                 rvecs,
                                                 tvecs);
            for (int i(0); i < corner.size(); ++i) {
                //201 mm


                cv::aruco::drawAxis(in_img,intrinsic_matrix,distortion_matrix,
                rvecs[i],tvecs[i],0.1);
            }

//            cv::aruco::drawAxis(in_img, intrinsic_matrix, distortion_matrix,)
//            cv::aruco::drawMarker(dic,ids[0],2,out_img);
        }


//        out_img = in_img;

        cv::imshow(win_name, in_img);
        cvWaitKey(10);
    }


    return 0;
}