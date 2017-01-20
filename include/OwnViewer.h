#pragma once
//
// Created by steve on 17-1-20.
//

#ifndef ARSLAM_OWNVIEWER_H
#define ARSLAM_OWNVIEWER_H

#include <iostream>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common_headers.h>


#include <boost/thread/thread.hpp>


//#include <thread>
#include <mutex>

#include <Eigen/Core>
#include <Eigen/Geometry>


pcl::visualization::PCLVisualizer vis;
class OwnViewer{
public:
    OwnViewer(std::string windows_name = "Own 3D Viewer"
    ):

            viewer_ptr_(new pcl::visualization::PCLVisualizer(""))
//            viewer_(windows_name)
    {
//        viewer_ = pcl::visualization::PCLVisualizer(windows_name);
//        viewer_ptr_ = boost::shared_ptr<pcl::visualization::PCLVisualizer>(viewer_);
//        vis = viewer_;
//        *viewer_ptr_ = vis;
//        *viewer_ptr_=pcl::visualization::PCLVisualizer(windows_name);

    }


    bool Start()
    {
        viewer_ptr_->addCoordinateSystem(1.0);
        boost::thread t(&OwnViewer::Run,this);
        t.detach();
//        viewer_ptr_->spin();

//        boost::thread
    }


    bool addMarker(Eigen::Affine3d m_t,int marker_id);


    bool addCamera(Eigen::Vector3d srcp,
    Eigen::Vector3d targetp);


    bool Run();


    void test()
    {
        viewer_ptr_->addText("aaa",100,100,"str1",0);
        viewer_ptr_->addCoordinateSystem(20.0);
    }


    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_ptr_ ;//= new pcl::visualization::PCLVisualizer();

    pcl::visualization::PCLVisualizer viewer_;

private:








    std::mutex viewer_mutex_;

    bool is_run_ = true;


    // object string
    std::string marker_str_ = "mark";








};

bool OwnViewer::Run()
{
//    viewer_ptr_->spin();
//    pcl::visualization::PCLVisualizer vis("tst");
//    *viewer_ptr_ = vis;
//    test();
    while(is_run_)
    {
        viewer_mutex_.lock();
//        viewer_ptr_->spinOnce(1,true);
        viewer_ptr_->spinOnce();
        viewer_mutex_.unlock();
//        viewer_ptr_->spin();
//        std::cout << "run " << std::endl;

        boost::this_thread::sleep(boost::posix_time::microseconds(10000));
    }
    return true;

}



bool OwnViewer::addMarker(Eigen::Affine3d m_t, int marker_id) {
//    viewer_mutex_.lock();
//    viewer_ptr_->addCoordinateSystem(1.0,Eigen::Affine3f(m_t),marker_str_+std::to_string(marker_id),0);
    Eigen::Affine3f t(m_t);
    std::cout << " t :\n " << t.matrix() << std::endl;


    viewer_ptr_->addCoordinateSystem(1.0,t,0);
//    viewer_mutex_.unlock();

}


bool OwnViewer::addCamera(Eigen::Vector3d srcp, Eigen::Vector3d targetp) {
    viewer_ptr_->addArrow(pcl::PointXYZ(srcp(0),srcp(1),srcp(2)),
    pcl::PointXYZ(targetp(0),targetp(1),targetp(2)),
    200,0,200,
    "camera"
    );
}
#endif //ARSLAM_OWNVIEWER_H
