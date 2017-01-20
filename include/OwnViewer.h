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


#include <thread>
#include <mutex>

#include <Eigen/Core>
#include <Eigen/Geometry>

class OwnViewer{
public:
    OwnViewer(std::string windows_name = "Own 3D Viewer"
    ):
            viewer_ptr_ (new pcl::visualization::PCLVisualizer(windows_name))
    {
        std::cout << "first" << std::endl;
        std::thread t1(&OwnViewer::Run,this);
        std::cout << "second" << std::endl;
        t1.detach();
        std::cout << "third " << std::endl;

    }


    bool addMarker(Eigen::Affine3d m_t,int marker_id);







private:

    bool Run();




    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_ptr_;

    std::mutex viewer_mutex_;

    bool is_run_ = true;


    // object string
    std::string marker_str_ = "mark";








};

bool OwnViewer::Run()
{
    while(is_run_)
    {
        viewer_mutex_.lock();
        viewer_ptr_->spinOnce();
        viewer_mutex_.unlock();

        boost::this_thread::sleep(boost::posix_time::microseconds(10000));
    }
    return true;

}


bool OwnViewer::addMarker(Eigen::Affine3d m_t, int marker_id) {
    viewer_mutex_.lock();
//    viewer_ptr_->addCoordinateSystem(1.0,Eigen::Affine3f(m_t),marker_str_+std::to_string(marker_id),0);
    Eigen::Affine3f t(m_t);
    std::cout << " t :\n " << t.matrix() << std::endl;


    viewer_ptr_->addCoordinateSystem(1.0,t,0);
    viewer_mutex_.unlock();

}
#endif //ARSLAM_OWNVIEWER_H
