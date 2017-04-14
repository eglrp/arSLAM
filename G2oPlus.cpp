//
// Created by steve on 4/13/17.
//

#include <iostream>
#include <map>
#include <deque>

#include <memory>
#include <thread>
#include <mutex>


#include <random>

#include "time_stamp.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/types/slam3d_addons/types_slam3d_addons.h"

#include "OwnEdge/Z_zero_edge.h"

#include "OwnEdge/Z_zero_edge.cpp"
//G2O_USE_TYPE_GROUP(slam3d);

int main(int argc,char *argv[])
{
    //Initial g2o optimizer
    g2o::SparseOptimizer globalOptimizer;


    typedef g2o::BlockSolver_6_3 SlamBlockSolver;
    typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

    // 初始化求解器
    SlamLinearSolver *linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering(false);
    SlamBlockSolver *blockSolver = new SlamBlockSolver(linearSolver);
    g2o::OptimizationAlgorithmLevenberg *solver =
            new g2o::OptimizationAlgorithmLevenberg(blockSolver);
    globalOptimizer.setAlgorithm(solver);

    globalOptimizer.load("/home/steve/Data/save_graph.g2o");

    for(int i(10);i<100;++i)
    {
        auto the_vertex = globalOptimizer.vertex(i);
        std::cout << "Get vertex ok." << std::endl;
        if(the_vertex>0&&the_vertex->id()!=11)
        {          double data[10]={100};
            the_vertex->getEstimateData(data);
            std::cout << "id :" << the_vertex->id() << " ";
           //add new edge
            auto *v = new ZzeroEdge();
            v->vertices()[0]=the_vertex;
            v->vertices()[1]=globalOptimizer.vertex(11);
            Eigen::Matrix<double,1,1> info= Eigen::Matrix<double,1,1>::Identity();
            info(0, 0) = 1000;
//            v->setInformation(info);
//            std::cout << "after set information " << std::endl;
            v->setMeasurement(0.0);
            std::cout << "after set Measurement" << std::endl;
            globalOptimizer.addEdge(v);
            std::cout << "add to graph ok " << std::endl;

        }else{
            std::cout << "break" << std::endl;
            continue;
        }
    }


    globalOptimizer.initializeOptimization(0);
    std::cout << "initial g2o ok" << std::endl;

    auto start_time = TimeStamp::now();
    std::cout << "start optimize :" << start_time << std::endl;
    globalOptimizer.optimize(30);
    std::cout << " Optimize time :" << TimeStamp::now()-start_time << std::endl;
    std::cout << "Optimizer ok:" << std::endl;
    for(int i(10);i<100;++i)
    {
        auto the_vertex = globalOptimizer.vertex(i);
        std::cout << "Get vertex ok." << std::endl;
        if(the_vertex>0)
        {
            //add new edge
            double data[10]={100};
            the_vertex->getEstimateData(data);
            std::cout << "id :" << the_vertex->id() << " ";
            for(int kk(0);kk<8;++kk)
            {
                std::cout << data[kk]<<" ";
            }
            std::cout << std::endl;


        }else{
            std::cout << "break" << std::endl;
            continue;
        }
    }
    globalOptimizer.save("/home/steve/test.g2o");


}