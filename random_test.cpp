//
// Created by steve on 17-3-4.
//


#include <iostream>

#include "omp.h"

#include <random>

#include <boost/asio.hpp>


int main()
{
    std::default_random_engine e(0);
    std::uniform_real_distribution<> rd(0.0,1.0);

#pragma omp parallel for
    for( int i = 0;i<10000;++i)
    {
        printf("rnd : %f\n",rd(e));
    }

//    boost::asio::
}

