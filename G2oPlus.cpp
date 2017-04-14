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

G2O_USE_TYPE_GROUP(slam3d);

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

    globalOptimizer.load("/home/steve/Data/save_graph_opt.g2o");

    for(int i(10);i<100;++i)
    {
        auto the_vertex = globalOptimizer.vertex(i);
        if(the_vertex->id()==i)
        {
           //add new edge

        }else{
            break;
        }
    }







    globalOptimizer.initializeOptimization(0);

    auto start_time = TimeStamp::now();
    globalOptimizer.optimize(1000);
    std::cout << " Optimize time :" << TimeStamp::now()-start_time << std::endl;
    std::cout << "Optimizer ok:" << std::endl;


}