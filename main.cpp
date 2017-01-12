#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

int main() {
    std::cout << "Hello, World!" << std::endl;


    cv::VideoCapture cap(1);
    cv::Mat in_img, out_img;
    std::string win_name("debug");

    cv::namedWindow(const_cast<char *> (win_name.c_str()));

    cv::waitKey(10);

    while (cap.isOpened()) {
        cap >> in_img;

        out_img = in_img;

        cv::imshow(win_name, out_img);
        cvWaitKey(10);
    }


    return 0;
}