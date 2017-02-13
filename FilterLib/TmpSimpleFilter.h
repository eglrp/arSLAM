//
// Created by steve on 17-2-13.
//

#ifndef ARSLAM_TMPSIMPLEFILTER_H
#define ARSLAM_TMPSIMPLEFILTER_H


#include <Eigen/Dense>

#include <iostream>


class TmpSimpleFilter {

public:

    /**
     *
     * @param filter_type
     * @param initial_x
     * @param noise_sigma
     * @param evaluation_sigma
     * @param particle_number
     */
    TmpSimpleFilter(int filter_type, Eigen::Vector3d initial_x, Eigen::Vector3d noise_sigma,
                    Eigen::Vector3d evaluation_sigma, int particle_number = 1000);

protected:
    Eigen::MatrixXd particles_;// N*9(x,y,z,v_x,v_y,v_z,a_x,a_y,a_z) matrix,N represent the number of particle.

    Eigen::MatrixXd probability_; // N*1,N represent the number of particle.



private:
    int filter_type_ = 0;// 0:kf  , 1:pf
};


#endif //ARSLAM_TMPSIMPLEFILTER_H
