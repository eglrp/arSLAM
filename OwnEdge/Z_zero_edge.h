//
// Created by steve on 17-3-4.
//

#ifndef ARSLAM_Z_ZERO_EDGE_H
#define ARSLAM_Z_ZERO_EDGE_H

//#include "g2o_type_slam3d_addons_api.h"
#include "g2o/config.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/base_edge.h"
#include "g2o/core/hyper_graph_action.h"
#include "g2o/types/slam3d/vertex_se3.h"



class ZzeroEdge :public g2o::BaseEdge<1,g2o::VertexSE3>{
public:




};


#endif //ARSLAM_Z_ZERO_EDGE_H