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

#include <random>

#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/aruco.hpp>

//
#include <opencv2/opencv_modules.hpp>
#include <opencv2/stereo.hpp>
#include <opencv2/calib3d.hpp>


#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common_headers.h>


#include <boost/thread/thread.hpp>


//#include "MYCH

//#include "OwnViewer.h"



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
              distortion_matrix_(1, 5, CV_32F) {

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

    /**
     * verify
     */
    std::map<int, std::vector<Eigen::Vector3d>> verify_map_; //
    int verify_num_ = 100; // when N(number of similar transform in std::vector) > \
    //verify_num_ ,add this transform to transform_map_;
    double lcverify_dis_ = 0.1; // error is small than verify_dis_ is similar transform.

    cv::Ptr<cv::aruco::Dictionary> dic_ptr_;
//    cv::Ptr<cv::aruco::DetectorParameters> para_ptr_;

    cv::Mat intrinsic_matrix_, distortion_matrix_;

    std::vector<std::vector<cv::Point2f>> corner_;
    std::vector<int> ids_;


//    double real_length_ = 0.201;//

    double real_length_ = 0.199;//
    double draw_length_ = 0.3;

    Eigen::Vector3d current_pos_ = Eigen::Vector3d(0, 0, 0);


    /////

//    OwnViewer viewer_;


private:


};

void ArPoseFrame::BuildTransform() {

    std::map<int, Eigen::Affine3d> ids_pair;
    std::vector<int> id_list;
    pcl::visualization::PCLVisualizer viewer("test");

    viewer.setBackgroundColor(96, 96, 96);


    //TODO: Just for debug,need delete it before use.
    std::default_random_engine e;
    std::uniform_real_distribution<double> urandom(0, 0.99999999);


    viewer.addCoordinateSystem(0.3);
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

                    /**
                     * Search a marker in this frame that position is knowable,
                     * and use the known marker to compute the unknown position of new
                     * markers.(all in world frame)
                     */
                    for (int j(0); j < id_list.size(); ++j) {
                        auto s = transform_map_.find(id_list[j]);
                        if (s != transform_map_.end()) {
                            Eigen::Affine3d tmp;
                            tmp = ids_pair[id_list[j]].inverse() * ids_pair[id_list[i]];

                            tmp = transform_map_[id_list[j]] * tmp;


                            tmp = tmp.inverse();

                            /**
                             * More than N times detected a same position,then insert the position
                             * into transform_map_.
                             *
                             * N control by verify_num_
                             *
                             */

                            Eigen::Vector3d flag_vec(tmp.matrix()(0, 3),
                                                     tmp.matrix()(1, 3),
                                                     tmp.matrix()(2, 3));


                            auto vs = verify_map_.find(id_list[i]);
                            auto tm  = transform_map_.find(id_list[i]);
                            if (vs == verify_map_.end()) {
                                // First time detected a new marker.
                                std::vector<Eigen::Vector3d> t;
                                t.push_back(flag_vec);
                                verify_map_.insert(std::make_pair(id_list[i], t));
                            } else if(tm == transform_map_.end()){//never added to transform map
                                //find similar position.
                                int tmp_num(0);
                                std::vector<Eigen::Vector3d> tv(verify_map_[id_list[i]]);//all position
                                for (int ti(0); ti < tv.size(); ++ti) {
                                    // current position similar to tv[i].
                                    if ((flag_vec - tv[i]).norm() < lcverify_dis_) {
                                        tmp_num++;
                                    }
                                }
                                verify_map_[id_list[i]].push_back(flag_vec);

                                std::cout << "tmp num : " << tmp_num << " in "
                                          << tv.size() << std::endl;

                                /** Extract conditions
                                 * 1.z-axis  vertically relative to horizontal plane
                                 *
                                 * 2.the origin of coordinates is in the x-y plane(z equal to zero).
                                 */

                                bool is_extract_condition_ok(true);

                                /// 1.
                                Eigen::Vector4d src_z(0, 0, 1, 0);
                                Eigen::Vector4d target_z(0, 0, 0, 0);

                                target_z = tmp * src_z;
//                                std::cout << "target z :" << target_z.transpose() << std::endl;
                                if (target_z[2] < 0.97) {
                                    is_extract_condition_ok = false;
                                }

                                /// 2.
                                Eigen::Vector4d src_zero(0, 0, 0, 1);
                                src_zero = tmp * src_zero;

                                std::cout << "zero : " << src_zero.transpose() << std::endl;
                                if (src_zero[2] > 0.03 || src_zero[2] < -0.03) {
                                    is_extract_condition_ok = false;
                                }




                                if (tmp_num > verify_num_ && is_extract_condition_ok) {
                                    transform_map_.insert(std::make_pair(id_list[i], tmp));

//                            viewer_.addMarker(tmp,id_list[i]);

                                    viewer.addCoordinateSystem(0.2, Eigen::Affine3f(tmp.inverse()),
                                                               "id:" + std::to_string(id_list[i]));
                                }
                            }

//                            for(int i(0);i<verify_map_[id_list[i]].size();)
//                            {
//
//                            }

//                            break;
                        }

//                        if (urandom(e) > 0.4) {
//                            break;
//                        }
                    }
                }

            }

        }

        ////Get Pose.

        Eigen::Vector3d pose(0, 0, 0);
        std::vector<Eigen::Vector3d> pose_list;
        std::vector<int> score_list;
        for (int i(0); i < id_list.size(); ++i) {
            Eigen::Vector3d tmp_pose(0, 0, 0);
            auto s = transform_map_.find(id_list[i]);

//            pose =  ids_pair[id_list[i]] * s->second * tmp_pose;
            tmp_pose = s->second.inverse() * ids_pair[id_list[i]].inverse() * tmp_pose;
//            std::cout << "Pose:" << pose.transpose() << std::endl;
            pose_list.push_back(tmp_pose);
            score_list.push_back(0);

        }

        /**
         * vote to similar pose.
         */
        double dist_threold(0.33);
        for (int i(0); i < pose_list.size(); ++i) {
            for (int j(0); j < pose_list.size(); ++j) {
                if ((pose_list.at(i) - pose_list.at(j)).norm() < dist_threold) {
//                    std::cout << "small than threold" << std::endl;
                    score_list[i] += 1;
                }
            }
        }

        int all_num(0);
        for (int i(0); i < pose_list.size(); ++i) {
            if (score_list[i] >= score_list.size() - 2) {
                pose += pose_list[i];
                all_num++;
            }


            /// delete error value in transform_map_.
//            if(score_list[i]==2 && pose_list.size()>4)
//            {
//                int errortag_id = id_list[i];
//                auto tmp_pair = transform_map_.find(errortag_id);
//                if(tmp_pair!=transform_map_.end())
//                {
////                    transform_map_.erase(errortag_id);
////                    viewer.removeCoordinateSystem("id:"+std::to_string(errortag_id),0);
//                }
//
//            }

        }

        if (all_num == 0) {
            pose = Eigen::Vector3d(0, 0, 0);
//            if(pose_list.size()>0)
//            {
//                pose = pose_list[0];
//            }
        } else {
            pose /= double(all_num);
        }
//        std::cout << all_num << "is all num"
//                  << pose_list.size()  <<"is pose list size"
//                  << std::endl;


        if(std::isnan(pose[2]))
        {
//            pose[2] = current_pos_[2];
            pose = Eigen::Vector3d(0,0,0);
        }
        current_pos_ = pose;
//        viewer.removeCoordinateSystem("camera");
//        viewer.addCoordinateSystem(0.10, Eigen::Affine3f(ids_pair[id_list[i]] * s->second).inverse(),
//                                   "camera");


        viewer.removeShape("arrow");
        viewer.addArrow(pcl::PointXYZ(pose(0), pose(1), pose(2)),
                        pcl::PointXYZ(0, 0, 0), 200, 200, 20, "arrow");

        viewer.spinOnce(1);

    }
}


void ArPoseFrame::ProcessImg(cv::Mat in) {

    corner_.clear();
    ids_.clear();
//    std::cout << "IN ProcessImg" << std::endl;


    cv::aruco::detectMarkers(in, dic_ptr_, corner_, ids_);//,para_ptr_);

    cv::aruco::drawDetectedMarkers(in, corner_, ids_);

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
