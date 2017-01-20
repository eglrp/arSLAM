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




#include "OwnViewer.h"



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
    ArPoseFrame(int initial_id)
            : intrinsic_matrix_(3, 3, CV_32F),
              distortion_matrix_(1, 5, CV_32F),
              viewer_("ArSlAM Test") {

        dic_ptr_ = new cv::aruco::Dictionary(
                cv::aruco::getPredefinedDictionary(
                        cv::aruco::DICT_6X6_100));


        initial_id_ = initial_id;
        Eigen::Affine3d t3d = Eigen::Affine3d::Identity();
        transform_map_.insert(std::make_pair(initial_id_, t3d));

        LoadCameraPara("./data/intrinsic_matrix.txt",
                       "./data/distortion_matrix.txt");

        cv::namedWindow("ArPoseFrame");

        std::thread btf(&ArPoseFrame::BuildTransform, this);
        btf.detach();

    }

    /**
     * Process image from video;
     * @param in
     */
    void ProcessImg(cv::Mat in);


    bool LoadCameraPara(std::string intrinsic_matrix_file,
                        std::string distortion_matrix_file);

    /**
     * Use rotation vector and tvec from cv compute transform matrix.
     * @param rvec
     * @param tvec
     * @return
     */
    Eigen::Affine3d rt2Matrix(cv::Vec3d rvec, cv::Vec3d tvec);


protected:

    void BuildTransform();

    std::mutex vecs_mutex_;
    std::vector<cv::Vec3d> rvecs_, tvecs_;// temp
    std::vector<int> tids_;   // Id of saved markers.

    int initial_id_;//the first one,set it's frame as global frame.

    std::map<int, Eigen::Affine3d> transform_map_;//transform_map_ to first

    cv::Ptr<cv::aruco::Dictionary> dic_ptr_;
//    cv::Ptr<cv::aruco::DetectorParameters> para_ptr_;

    cv::Mat intrinsic_matrix_, distortion_matrix_;

    std::vector<std::vector<cv::Point2f>> corner_;
    std::vector<int> ids_;


    double real_length_ = 0.201;
    double draw_length_ = 0.3;

    Eigen::Vector3d current_pos_ = Eigen::Vector3d(0, 0, 0);


    /////

    OwnViewer viewer_;


private:


};

void ArPoseFrame::BuildTransform() {

    std::map<int, Eigen::Affine3d> ids_pair;
    std::vector<int> id_list;
    while (1) {
        vecs_mutex_.lock();
        if (tids_.size() > 0) {
            ids_pair.clear();
            id_list.clear();
            for (int i(0); i < tids_.size(); ++i) {
                ids_pair.insert(std::make_pair(tids_[i], rt2Matrix(rvecs_[i], tvecs_[i])));
                id_list.push_back(tids_[i]);
            }
        }
        tids_.clear();
        vecs_mutex_.unlock();

        for (int i(0); i < id_list.size(); ++i) {
            //Updata transform_map_.
            if (id_list[i] != initial_id_) {
                auto search = transform_map_.find(id_list[i]);
                if (search == transform_map_.end()) {


                    //try to build relationship between id_list[i] and initial_id_.

                    for (int j(0); j < id_list.size(); ++j) {
                        auto s = transform_map_.find(id_list[j]);
                        if (s != transform_map_.end()) {
                            Eigen::Affine3d tmp;
                            tmp = ids_pair[id_list[j]].inverse() * ids_pair[id_list[i]];

                            tmp = transform_map_[id_list[j]] * tmp;
                            transform_map_.insert(std::make_pair(id_list[i], tmp));
                            viewer_.addMarker(tmp,id_list[i]);
                            break;
                        }
                    }
                }

            }

        }

        //Get Pose.
        for (int i(0); i < id_list.size(); ++i) {
            Eigen::Vector3d tmp_pose(0, 0, 0);
            auto s = transform_map_.find(id_list[i]);
            Eigen::Vector3d pose(0, 0, 0);
            pose = s->second * ids_pair[id_list[i]] * tmp_pose;
            std::cout << "Pose:" << pose.transpose() << std::endl;
            current_pos_ = pose;

            break;

        }


    }
}


void ArPoseFrame::ProcessImg(cv::Mat in) {

    corner_.clear();
    ids_.clear();
//    std::cout << "IN ProcessImg" << std::endl;


    cv::aruco::detectMarkers(in, dic_ptr_, corner_, ids_);//,para_ptr_);
    if (ids_.size() > 0) {
        if (initial_id_ < 0) {
            initial_id_ = ids_[0];
        }

        std::vector<cv::Vec3d> rvecs, tvecs;


        try {
            cv::aruco::estimatePoseSingleMarkers(corner_,
                                                 real_length_,
                                                 intrinsic_matrix_,
                                                 distortion_matrix_,
                                                 rvecs,
                                                 tvecs);

            //Deep copy?
            vecs_mutex_.lock();
            rvecs_ = rvecs;
            tvecs_ = tvecs;
            tids_ = ids_;
            vecs_mutex_.unlock();

            for (int i(0); i < rvecs.size(); ++i) {

                cv::aruco::drawAxis(in, intrinsic_matrix_, distortion_matrix_,
                                    rvecs[i], tvecs[i], draw_length_);

            }

        } catch (cv::Exception &a) {
            std::cout << a.err << std::endl;
        }


    }

//    cv::addText(in,"pose"+current_pos_,cv::Point2f(100,100),"")
    std::stringstream ss;
    ss << current_pos_;

    cv::putText(in, ss.str(), cv::Point2f(100, 100), CV_FONT_NORMAL, 1, cv::Scalar(20, 200, 20));
    cv::imshow("ArPoseFrame", in);
//        cv::waitKey(10);

    return;

}


Eigen::Affine3d ArPoseFrame::rt2Matrix(cv::Vec3d rvec, cv::Vec3d tvec) {
    cv::Mat cv_rotation_matrix;

    cv::Rodrigues(rvec, cv_rotation_matrix);
//    Eigen::Matrix3d rotation_matrix;
    Eigen::Affine3d transform_matrix(Eigen::Affine3d::Identity());

    for (int i(0); i < 3; ++i) {
        for (int j(0); j < 3; ++j) {
            transform_matrix(i, j) = cv_rotation_matrix.at<double>(i, j);
        }
    }

    for (int i(0); i < 3; ++i) {
        transform_matrix(i, 3) = tvec(i);
    }
    return transform_matrix;

}


bool ArPoseFrame::LoadCameraPara(std::string intrinsic_matrix_file,
                                 std::string distortion_matrix_file) {
    std::fstream cf, df;
    try {
        cf.open(const_cast<char *> (intrinsic_matrix_file.c_str()));
        df.open(const_cast<char *> (distortion_matrix_file.c_str()));

        if (!cf.is_open() && !df.is_open()) {
            std::cout << "Open camera parameters files error." << std::endl;

            std::cerr << "Open camera parameters files error." << std::endl;

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
    } catch (std::exception &e) {
        std::cerr << e.what() << std::endl;
    }
    return true;

}


#endif //ARSLAM_ARPOSEFRAME_H
