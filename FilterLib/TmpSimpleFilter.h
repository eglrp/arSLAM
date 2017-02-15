//
// Created by steve on 17-2-13.
//
#pragma once
#ifndef ARSLAM_TMPSIMPLEFILTER_H
#define ARSLAM_TMPSIMPLEFILTER_H


#include <Eigen/Dense>

#include <iostream>
#include <random>

#include "time_stamp.h"
#include "MyError.h"

#define TmpSimpleFilterDEBUG false

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


    /**
     *  initial all particle to same state.
     * @param initial_state
     * @return
     */
    bool InitialState(Eigen::Vector3d initial_state,double time = -1.0);


    /**
     * State transmission.
     * @return
     */
    bool StateTransmission(double time = -1.0);


    /**
     * Evaluation every particle according to guess_pose
     * @param guess_pose_list
     * @return
     */
    bool Evaluation(std::vector<Eigen::Vector3d> guess_pose_list);

    /**
     * Resample
     * @param resample_method  0: normal method
     * @param particle_filter -1:do not change the particle_num ,N(>0):change the particle_num to N.
     * @return
     */
    bool Resample(int resample_method, int particle_num);


    /**
     * Get average result for particles.
     * @return
     */
    Eigen::Vector3d GetResult();

protected:
    Eigen::MatrixXd particles_;// N*9(x,y,z,v_x,v_y,v_z,a_x,a_y,a_z) matrix,N represent the number of particle.

    Eigen::MatrixXd probability_; // N*1,N represent the number of particle.

    int particle_num_; // particle number

    Eigen::Vector3d noise_sigma_;//noise sigma

    Eigen::Vector3d evaluation_sigma_; // evaluation sigma

    std::default_random_engine e_; // random engine

    double last_time_ = -10.0 ; //

    /*
     * Single value Normal probability distribution function.
     */
    double ScalarNormalPdf(double x, double miu, double sigma) {
        try {
            double para1 = (x - miu) * (x - miu) / 2 / sigma / sigma;
            double para2 = (1 / std::sqrt(2 * M_PI) / sigma);

            return para2 * std::exp(-para1);
        } catch (...) {
            //Some error when compute the Single value .

            return 0.000000001;
        }

    }

private:
    int filter_type_ = 0;// 0:kf  , 1:pf
};

TmpSimpleFilter::TmpSimpleFilter(int filter_type, Eigen::Vector3d initial_x, Eigen::Vector3d noise_sigma,
                                 Eigen::Vector3d evaluation_sigma, int particle_number) :
        particles_(particle_number, 9),
        probability_(particle_number, 1) {
    MYCHECK(TmpSimpleFilterDEBUG);


    filter_type_ = filter_type;
    if (filter_type_ == 0) {
        /**
         * EKF....
         */
    } else if (filter_type_ == 1) {
        /**
         * Particle Filter...
         */
        particle_num_ = particle_number;

        particles_.resize(particle_num_, 9);
        probability_.resize(particle_num_, 1);

        noise_sigma_ = noise_sigma;
        evaluation_sigma_ = evaluation_sigma;

    }

    MYCHECK(TmpSimpleFilterDEBUG);
}


bool TmpSimpleFilter::InitialState(Eigen::Vector3d initial_state,
                                   double time = -1) {
    MYCHECK(TmpSimpleFilterDEBUG);
   particles_.setZero();
   MYCHECK(TmpSimpleFilterDEBUG);
    probability_.setOnes();
    probability_ /= probability_.sum();

   if(time < 0.0)
   {
       last_time_ = TimeStamp::now();
   }else{
       last_time_ = time;
   }
    for(int i(0);i<particles_.rows();++i)
    {
        for(int j(0);j<3;++j)
        {
            particles_(i,j) = initial_state(j);
        }
    }
    MYCHECK(TmpSimpleFilterDEBUG);
    return true;
}

bool TmpSimpleFilter::StateTransmission(double time = -1) {
    MYCHECK(TmpSimpleFilterDEBUG);
    double time_now;// = TimeStamp::now();
    if(time < -1)
    {
        time_now = TimeStamp::now();
    }else{
        time_now = time;
    }
    double dt = time_now-last_time_;
    std::normal_distribution<> normal_distribution(0,noise_sigma_.mean());

    for(int i(0);i<particles_.rows();++i)
    {
        /**
         * 1. a_x a_y a_z add random noise
         * 2. v_x v_y v_z update according to (v = a * t)
         * 3. x y z update according to (x = v_x * t + 0.5 * a_x * t * t)
         */


        ////1.
        for(int j(6);j<9;++j)
        {
            particles_(i,j) += normal_distribution(e_);
        }


        ////2.
        for(int j(3);j<6;++j)
        {
            particles_(i,j) += dt * particles_(i,j+3);
        }

        ////3.
        for(int j(0);j<3;++j)
        {
            particles_(i,j) += dt * particles_(i,j+3) + 0.5 * dt * dt * particles_(i,j+6);
        }
    }
    last_time_ = time_now;
    MYCHECK(TmpSimpleFilterDEBUG);

}

bool TmpSimpleFilter::Evaluation(std::vector<Eigen::Vector3d> guess_pose_list) {
    MYCHECK(TmpSimpleFilterDEBUG);
    for(int i(0);i<particles_.rows();++i)
    {
        double score(0.0000001);
        Eigen::Vector3d tmp_pose(particles_(i,0),particles_(i,1),particles_(i,2));

        for(int index(0);index < guess_pose_list.size();++index)
        {
            score += ScalarNormalPdf((tmp_pose-guess_pose_list[index]).norm(),
                                     0,evaluation_sigma_.mean());
        }
        score /= double(guess_pose_list.size());

        probability_(i,0) *= score;
    }
    probability_ /= probability_.sum();
    MYCHECK(TmpSimpleFilterDEBUG);
    return true;
}


bool TmpSimpleFilter::Resample(int resample_method, int particle_num) {
    MYCHECK(TmpSimpleFilterDEBUG);
    Eigen::MatrixXd tmp_particles;//(particles_);
    Eigen::MatrixXd tmp_probability;//(probability_);

    tmp_particles.resize(particles_.rows(),particles_.cols());
    tmp_probability.resize(probability_.rows(),probability_.cols());

    tmp_particles = particles_;
    tmp_probability = probability_;

    std::uniform_real_distribution<double> real_distribution(0,0.999999);
    for(int index(0);index<particles_.rows();++index)
    {
        double score = real_distribution(e_);
        int i(-1);

        while(score>0)
        {
            ++i;
            score -= tmp_probability(i);
        }

        if(i>=tmp_particles.rows())
        {
            i = tmp_particles.rows()-1;

            MYERROR(" over range in resample stage");

        }


        for(int j(0);j<particles_.cols();++j)
        {
            particles_(index,j) = tmp_particles(i,j);
        }
        probability_(index,0) = tmp_probability(i,0);


    }

    if(std::isnan(probability_.sum()))
    {
        MYERROR("probability has some nan value.");
        probability_.setOnes();
    }
    probability_ /= probability_.sum();
    MYCHECK(TmpSimpleFilterDEBUG);
    return true;

}

Eigen::Vector3d TmpSimpleFilter::GetResult() {
    MYCHECK(TmpSimpleFilterDEBUG);
    Eigen::Vector3d pose(0,0,0);
    probability_ /= probability_.sum();

    for(int i(0);i<particles_.rows();++i)
    {
        for(int j(0);j<3;++j)
        {
            pose(j) += probability_(i,0) * particles_(i,j);
        }
    }
    MYCHECK(TmpSimpleFilterDEBUG);
    return pose;
}

#endif //ARSLAM_TMPSIMPLEFILTER_H
