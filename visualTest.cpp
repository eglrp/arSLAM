//
// Created by steve on 17-1-20.
//


#include <iostream>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common_headers.h>

#include <boost/thread/thread.hpp>


int main()
{
   boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_ptr(
           new pcl::visualization::PCLVisualizer("3dviewer")
   );
   viewer_ptr->addText("test information",10,10,"string1",0);

//   viewer_ptr->addArrow()

//   viewer_ptr->spin();
    viewer_ptr->addArrow(pcl::PointXYZ(0,0,0),
    pcl::PointXYZ(10,0,10),20,20,200,"arrow 1");
    viewer_ptr->addCoordinateSystem(1.0);
    double px(0.0),py(0.0),pz(0.0);
    int arrowid(10);
    while(true)
    {

//        viewer_ptr->removePolygonMesh("arrow 1",0);
        viewer_ptr->addArrow(pcl::PointXYZ(px,py,pz),
        pcl::PointXYZ(px+10,py,pz+10),
        20,20,200,
        "arrow " + std::to_string(arrowid));
        px += 0.1;
        py += 0.1;
        pz += 0.1;
        arrowid ++;
        viewer_ptr->removeShape("arrow " + std::to_string(arrowid-10));
       viewer_ptr->spinOnce();
        boost::this_thread::sleep(boost::posix_time::microseconds(10000));



    }
}