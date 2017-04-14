//
// Created by steve on 17-3-4.
//

#include "Z_zero_edge.h"

ZzeroEdge::ZzeroEdge() :BaseBinaryEdge<1,double,g2o::VertexSE3,g2o::VertexSE3>()
{
    information().setIdentity();
    _information(0,0) = 100.0;
}

bool ZzeroEdge::read(std::istream &is) {
    /**
     * Achive it !!!
     */

    return true;
}

bool ZzeroEdge::write(std::ostream &os) const {
    os << "ss" << std::endl;
    return os.good();
}


void ZzeroEdge::computeError() {
   g2o::VertexSE3 *from = static_cast<g2o::VertexSE3*>(_vertices[0]);
    g2o::VertexSE3 *to = static_cast<g2o::VertexSE3*>(_vertices[1]);
    _error(0, 0) = (from->estimate().matrix()(2, 3) - to->estimate().matrix()(2, 3));
}

bool ZzeroEdge::setMeasurementFromState() {
    setMeasurement(0);
    return true;
}

void ZzeroEdge::linearizeOplus() {
//    std::cout << "linearizeOplus" << std::endl;
    g2o::VertexSE3 *from = static_cast<g2o::VertexSE3*>(_vertices[0]);
    g2o::VertexSE3 *to   = static_cast<g2o::VertexSE3*>(_vertices[1]);
    _jacobianOplusXi(0, 2) = -1.;//*(_error(0,0));
    _jacobianOplusXj(0, 2) = -1.;//* (_error(0,0));
//    _jacobianOplusXi.setZero();
//    _jacobianOplusXj.setZero();
//    std::cout << " after linearizeOplus" << std::endl;
}


void ZzeroEdge::initialEstimate(const g2o::OptimizableGraph::VertexSet &from,
                                g2o::OptimizableGraph::Vertex *to) {
    /**
     * Do nothing
     */
}


