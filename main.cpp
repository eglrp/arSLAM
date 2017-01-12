#include <iostream>
#include <fstream>

#include <memory>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
//
//#include <opencv2/viz.hpp>




#include <Eigen/Dense>


int main() {
    std::cout.precision(20);
    std::cout << "Hello, World!" << std::endl;


    cv::VideoCapture cap("/dev/video0");
    cv::Mat in_img;
    cv::Mat *out_img_ptr;
    std::string win_name("debug");
    std::string threed_name("3ddebug");

    cv::namedWindow(const_cast<char*>(threed_name.c_str()));

    cv::namedWindow(const_cast<char *> (win_name.c_str()));

    cv::waitKey(10);

//    viz::Viz

    cv::aruco::Dictionary dic = (cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_100));
    cv::Ptr<cv::aruco::Dictionary> dic_ptr(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_100));

/**
 * Read data to set intrinsc_matrix and distortion matrix.
 */
    cv::Mat intrinsic_matrix(3,3,CV_32F);
    cv::Mat distortion_matrix(1,5,CV_32F);

    uchar * dp = distortion_matrix.data;
    float *ddp ;
    ddp = (float *)dp;


    uchar *di = intrinsic_matrix.data;
    float *ddi;
    ddi = (float *)di;



    std::fstream tfs("./data/distortion_matrix.txt");
    for (int k(0); k < 5; ++k) {
        double ttt;
        tfs >> ttt;

        std::cout << "tttt:" << ttt << std::endl;
        ddp[k] = ttt;



    }
    std::cout << distortion_matrix << std::endl;

//
    std::fstream ifs;
    ifs.open("./data/intrinsic_matrix.txt");
    std::cout << "isf:" << ifs.is_open() << std::endl;
    for (int ii(0); ii < 3; ++ii) {
        for (int jj(0); jj < 3; ++jj) {
            double d_tmp(0.0);
            ifs >> d_tmp;
            std::cout << d_tmp << "kkkk" << std::endl;

            ddi[ii*3+jj] = d_tmp;
        }
    }


    std::cout << "-----------------" << std::endl;
    std::cout << intrinsic_matrix << std::endl;

    /*
     * End read matrix.
     */



    while (cap.isOpened()) {

        cap >> in_img;
        out_img_ptr = &in_img;
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

            std::vector<cv::Vec3d> rvecs, tvecs;
//            std::cout << 1.1<<std::endl;
            try{
                cv::aruco::estimatePoseSingleMarkers(corner,
                                                     201,
                                                     intrinsic_matrix,
                                                     distortion_matrix,
                                                     rvecs,
                                                     tvecs);

                for (int i(0); i < rvecs.size(); ++i) {
                    //201 mm
                    if(ids[i] == 11)
                    std::cout << rvecs[i] << "   " << tvecs[i] << std::endl;


                    cv::aruco::drawAxis(*out_img_ptr,intrinsic_matrix,distortion_matrix,
                                        rvecs[i],tvecs[i],100.1);

                }
            }catch(cv::Exception a)
            {
                std::cout << "error in estimate pose:"<<
                                                      a.err << std::endl;
            }

//            std::cout << 1.2 << std::endl;


//            cv::aruco::drawAxis(in_img, intrinsic_matrix, distortion_matrix,)
//            cv::aruco::drawMarker(dic,ids[0],2,out_img);
        }


//        out_img = in_img;

        cv::imshow(win_name, in_img);
        cvWaitKey(10);
    }


    return 0;
}