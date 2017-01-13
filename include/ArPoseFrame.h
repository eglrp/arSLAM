#pragma once
//
// Created by steve on 17-1-13.
//

#ifndef ARSLAM_ARPOSEFRAME_H
#define ARSLAM_ARPOSEFRAME_H

/**
 *                             _ooOoo_
 *                            o8888888o
 *                            88" . "88
 *                            (| -_- |)
 *                            O\  =  /O
 *                         ____/`---'\____
 *                       .'  \\|     |//  `.
 *                      /  \\|||  :  |||//  \
 *                     /  _||||| -:- |||||-  \
 *                     |   | \\\  -  /// |   |
 *                     | \_|  ''\---/''  |   |
 *                     \  .-\__  `-`  ___/-. /
 *                   ___`. .'  /--.--\  `. . __
 *                ."" '<  `.___\_<|>_/___.'  >'"".
 *               | | :  `- \`.;`\ _ /`;.`/ - ` : | |
 *               \  \ `-.   \_ __\ /__ _/   .-` /  /
 *          ======`-.____`-.___\_____/___.-`____.-'======
 *                             `=---='
 *          ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
 *                     佛祖保佑        永无BUG
 *            佛曰:
 *                   写字楼里写字间，写字间里程序员；
 *                   程序人员写程序，又拿程序换酒钱。
 *                   酒醒只在网上坐，酒醉还来网下眠；
 *                   酒醉酒醒日复日，网上网下年复年。
 *                   但愿老死电脑间，不愿鞠躬老板前；
 *                   奔驰宝马贵者趣，公交自行程序员。
 *                   别人笑我忒疯癫，我笑自己命太贱；
 *                   不见满街漂亮妹，哪个归得程序员？
*/

#include <iostream>
#include <fstream>

#include <map>
#include <deque>

#include <memory>
#include <thread>
#include <mutex>

#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/aruco.hpp>

//
#include <opencv2/opencv_modules.hpp>
#include <opencv2/stereo.hpp>
#include <opencv2/calib3d.hpp>


#include <Eigen/Core>
#include <Eigen/Geometry>

/**
 * Input
 */
class ArPoseFrame {
public:

    /**
     *
     * @param initial_id
     * @param dic
     * @param para
     */
    ArPoseFrame(int initial_id = -1,
                cv::aruco::Dictionary dic = cv::aruco::getPredefinedDictionary(cv::aruco::Dict_6x6_100),
                cv::aruco::DetectorParameters para = cv::aruco::DetectorParameters::create())
            : intrinsic_matrix_(3, 3, CV_32F),
              distortion_matrix_(1, 5, CV_32F),
              dic_ptr_(dic),
              para_ptr_(para) {
        initial_id_ = initial_id;

        LoadCameraPara("./data/intrinsic_matrix.txt",
                       "./data/distortion_matrix.txt");

        cv::namedWindow("ArPoseFrame");

    }

    /**
     * Process image from video;
     * @param in
     */
    void ProcessImg(cv::Mat in);


    bool LoadCameraPara(std::string intrinsic_matrix_file,
                        std::string distortion_matrix_file);

    Eigen::Affine3d rt2Matrix(cv::Vec3d rvec,cv::Vec3d tvec);


protected:

    int initial_id_;//the first one,set it's frame as global frame.

    std::map<int, Eigen::Affine3d> transform_map_;//transform_map_ to first

    cv::Ptr<cv::aruco::Dictionary> dic_ptr_;
    cv::Ptr<cv::aruco::DetectorParameters> para_ptr_;

    cv::Mat intrinsic_matrix_, distortion_matrix_;


private:


};

void ArPoseFrame::ProcessImg(cv::Mat in) {


}



bool ArPoseFrame::LoadCameraPara(std::string intrinsic_matrix_file,
                                 std::string distortion_matrix_file) {
    std::fstream cf, df;
    try {
        cf.open(const_cast<char *> (intrinsic_matrix_file.c_str()));
        df.open(const_cast<char *> (distortion_matrix_file.c_str()));

        if (!cf.is_open() && !df.is_open()) {
            std::cout << "Open camera parameters files error." << std::endl;

            std::err << "Open camera parameters files error." << std::endl;

        }


        double t(0.0);
        uchar *tp = intrinsic_matrix_.data;
        float *fp;
        fp = (float *) tp;
        for (int i(0); i < 3; ++i) {
            for (int j(0); j < 3; ++j) {
                cf >> t;
                fp[i * 3 + j] = t;
            }
        }


        tp = distortion_matrix_.data;
        fp = (float *) tp;
        for (int i(0); i < 5; ++i) {
            df >> t;
            fp[i] = t;
        }
    }catch(std::exception &e)
    {
        std::err << e.what() << std::endl;
    }
    return true;

}


#endif //ARSLAM_ARPOSEFRAME_H
