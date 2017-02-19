//
// Created by steve on 17-2-15.
//

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

#include <opencv2/videoio.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>


#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/types/slam3d_addons/types_slam3d_addons.h"

G2O_USE_TYPE_GROUP(slam3d);


#include "FilterLib/TmpSimpleFilter.h"

/**
 * Useful function....
 *
 */

Eigen::Isometry3d rt2Matrix(cv::Vec3d rvec, cv::Vec3d tvec) {
    cv::Mat cv_rotation_matrix;

    cv::Rodrigues(rvec, cv_rotation_matrix);
//    Eigen::Matrix3d rotation_matrix;
    Eigen::Isometry3d transform_matrix(Eigen::Isometry3d::Identity());

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


/**
 *
 * @return
 */


int main() {
    /**
     * Define
     */
    std::string video_file_name("./locate_dataset_hd/log_test2.mp4");
    std::string win_name("ShowTest");

    std::string intrinsic_matrix_file("./data/intrinsic_matrix.txt");
    std::string distortion_matrix_file("./data/distortion_matrix.txt");

    std::ofstream out_log("./log.txt");
    std::ofstream time_use_log("./time_use_log.txt");

    cv::namedWindow(win_name);

    int initial_marker_id(11);

    std::vector<int> added_id;
    added_id.push_back(initial_marker_id);

    int current_frame_id(1000);
    int plane_id(100);

    std::map<int,std::vector<g2o::EdgeSE3*>> edgen_vec_map;


    /**
     * PF initial
     */

    TmpSimpleFilter tpf(1,
                        Eigen::Vector3d(0,0,0),
                        Eigen::Vector3d(0.05,0.05,0.05),
                        Eigen::Vector3d(3.41,3.41,2.41),
                        5000);

    bool tpf_need_initial(true);



    /**
     * Open video
     */
    cv::VideoCapture cap(video_file_name);

    /**
     * Load Aruco parameter
     */

    double real_length(0.199);
    std::vector<cv::Vec3d> rvecs, tvecs;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corner;
    cv::Ptr<cv::aruco::Dictionary> dic_ptr = new cv::aruco::Dictionary(
            cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_100));


    cv::Mat intrinsic_matrix_(3, 3, CV_32F), distortion_matrix_(1, 5, CV_32F);

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

    /**
     * Initial g2o optimizer
     */
    g2o::SparseOptimizer globalOptimizer;

    g2o::VertexSE3 *v = new g2o::VertexSE3();
    v->setId(initial_marker_id);
    v->setEstimate(Eigen::Isometry3d::Identity());
    v->setFixed(true);
    globalOptimizer.addVertex(v);

    g2o::VertexPlane *p = new g2o::VertexPlane();
    p->setId(plane_id);
    p->setEstimate(g2o::Plane3D(Eigen::Vector4d(0, 0, 0, 1)));

    globalOptimizer.addVertex(p);

    typedef g2o::BlockSolver_6_3 SlamBlockSolver;
    typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

    // 初始化求解器
    SlamLinearSolver *linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering(false);
    SlamBlockSolver *blockSolver = new SlamBlockSolver(linearSolver);
    g2o::OptimizationAlgorithmLevenberg *solver =
            new g2o::OptimizationAlgorithmLevenberg(blockSolver);
    globalOptimizer.setAlgorithm(solver);

//    globalOptimizer.initializeOptimization();
//    g2o::EdgeSE3PlaneSensorCalib *edgePlane = new g2o::EdgeSE3PlaneSensorCalib;
//    edgePlane->vertices()[1] = globalOptimizer.vertex(initial_marker_id);
//    edgePlane->vertices()[0] = globalOptimizer.vertex(plane_id);
//    Eigen::Matrix4d information_plane;
//    information_plane.setIdentity();
//    information_plane *= 10.0;
////    edgePlane->setInformation(information_plane);
////    edgePlane->setInformation()
////    edgePlane->setMeasurement(g2o::Vector4D(0,0,0,1));
//    edgePlane->setMeasurement(g2o::Plane3D(Eigen::Vector4d(0,0,0,1)));
//    globalOptimizer.addEdge(edgePlane);


    /**
     * Main LOOP
     */

    std::cout << "Begin main loop" << std::endl;
    double time_begin = TimeStamp::now();
    while (true) {
        /**
         * Read image
         */
        time_begin = TimeStamp::now();
        cv::Mat img;
//        cap.read(img);
        cap >> img;
//        std::cout << img.size << std::endl;

        current_frame_id++;
//        std::cout << current_frame_id << std::endl;

        if (img.empty()) {
            break;
        }


        /**
         * Compute Transform Matrix && add to g2o
         */

        corner.clear();
        ids.clear();

        cv::aruco::detectMarkers(img,
                                 dic_ptr,
                                 corner,
                                 ids);

        cv::aruco::drawDetectedMarkers(
                img, corner, ids
        );


        /**
         * Detected markers in this frame.
         */
        if (ids.size() > 0) {
            rvecs.clear();
            tvecs.clear();
            /**
             * compute rvec tvec
             */
            cv::aruco::estimatePoseSingleMarkers(corner,
                                                 real_length,
                                                 intrinsic_matrix_,
                                                 distortion_matrix_,
                                                 rvecs, tvecs);

            /**
             * Draw axis
             */
            for (int i(0); i < rvecs.size(); ++i) {
                cv::aruco::drawAxis(img, intrinsic_matrix_,
                                    distortion_matrix_,
                                    rvecs[i], tvecs[i], real_length);
            }

            time_use_log << TimeStamp::now() - time_begin << " ";
            /**
             * Add vertex(markers id）
             */
            for (int i_ids(0); i_ids < ids.size(); ++i_ids) {
                if (std::find(added_id.begin(), added_id.end(), ids[i_ids]) == added_id.end()) {
                    added_id.push_back(ids[i_ids]);
                    g2o::VertexSE3 *v = new g2o::VertexSE3();
                    v->setId(ids[i_ids]);
                    v->setEstimate(Eigen::Isometry3d::Identity());
                    globalOptimizer.addVertex(v);

//                    g2o::EdgeSE3PlaneSensorCalib *ep = new g2o::EdgeSE3PlaneSensorCalib();
//                    ep->vertices()[1] = globalOptimizer.vertex(ids[i_ids]);
//                    ep->vertices()[0] = globalOptimizer.vertex(plane_id);
////                    ep->setInformation(information_plane);
////                    ep->setMeasurement(g2o::Vector4D(0,0,0,1));
//                    ep->setMeasurement(g2o::Plane3D(Eigen::Vector3d(0,0,0,1)));
//                    globalOptimizer.addEdge(ep);

                }
            }


            /**
             * Add vertex (current frame)
             */
            g2o::VertexSE3 *v = new g2o::VertexSE3();
            v->setId(current_frame_id);
            v->setEstimate(Eigen::Isometry3d::Identity());
            globalOptimizer.addVertex(v);

            /**
             * Add edge
             */
            for (int i_ids(0); i_ids < ids.size(); ++i_ids) {
                g2o::EdgeSE3 *edge = new g2o::EdgeSE3();
                edge->vertices()[0] = globalOptimizer.vertex(current_frame_id);
                edge->vertices()[1] = globalOptimizer.vertex(ids[i_ids]);
//                 edge->setRobustKernel()

                Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();
                information(0, 0) = information(1, 1) = information(2, 2) = 100;
                information(3, 3) = information(4, 4) = information(5, 5) = 100;
                edge->setInformation(information);
                Eigen::Isometry3d T = rt2Matrix(rvecs[i_ids], tvecs[i_ids]);
                edge->setMeasurement(T);
                globalOptimizer.addEdge(edge);
            }
            if(current_frame_id-1000 > 30)
            {
//                globalOptimizer.vertex(current_frame_id-20)->setFixed(true);

//                globalOptimizer.removeVertex(globalOptimizer.vertex(current_frame_id-29),
//                                             false);
//                globalOptimizer.removeEdge(globalOptimizer.e)
                g2o::HyperGraph::VertexSet v_set;
                v_set.insert(globalOptimizer.vertex(current_frame_id-29));
                MYCHECK(1);
                globalOptimizer.setFixed(v_set,true);
                MYCHECK(2);
//                globalOptimizer.
            }
            globalOptimizer.initializeOptimization(0);
            time_use_log << TimeStamp::now() - time_begin << " ";
            std::cout << " time use before first time optimize :" << TimeStamp::now() - time_begin << std::endl;
            globalOptimizer.optimize(10);

            if(tpf_need_initial)
            {
                globalOptimizer.optimize(200);
            }



            double * test_output = new double[10];
            globalOptimizer.vertex(current_frame_id)->getEstimateData(test_output);
            for(int i(0);i<10;++i)
            {
                std::cout << test_output[i];
            }
            std::cout << std::endl;
            std::cout << " time use before pf :" << TimeStamp::now() - time_begin << std::endl;
            time_use_log << TimeStamp::now() - time_begin << " ";
            if(tpf_need_initial)
            {

                tpf.InitialState(Eigen::Vector3d(test_output[0],
                test_output[1],
                test_output[2]),
                current_frame_id);
                tpf_need_initial = false;
            }else{
                tpf.StateTransmission(current_frame_id);
                std::vector<Eigen::Vector3d> guess_vec;
                guess_vec.push_back(Eigen::Vector3d(test_output[0],test_output[1],test_output[2]));
                tpf.Evaluation(guess_vec);
                auto after_pf = tpf.GetResult();
                tpf.Resample(1,1);
                std::cout << after_pf.transpose() << std::endl;
                out_log << after_pf.transpose() << std::endl;
            }
            time_use_log << TimeStamp::now() - time_begin << std::endl;

        }

        /**
         * Show image
         */
//        cv::imshow(win_name, img);
//        cv::waitKey(1);
        std::cout << " used time before optimize :" << TimeStamp::now() - time_begin << std::endl;
        globalOptimizer.optimize(30);
        std::cout << "use time :  " << TimeStamp::now() - time_begin << std::endl;
        //time_use_log << TimeStamp::now() - time_begin << std::endl;
    }
    out_log.close();
    std::cout << "final frame id :" << current_frame_id << std::endl;
    /**
     * Save g2o to file
     */
    globalOptimizer.save("./save_graph.g2o");
    return 0;

}