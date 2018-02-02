//
//  CartoPath.hpp
//  carto
//
//  Created by Matthew Thomas on 1/20/18.
//  Copyright © 2018 Matthew Thomas. All rights reserved.
//

#ifndef CartoPath_hpp
#define CartoPath_hpp
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "ortools/base/integral_types.h"
#include "ortools/constraint_solver/routing.h"



namespace Carto {

    class CartoNode;
    
class CartoPath {
public:
    cv::Mat detected_edges;
    
    CartoPath();
    CartoPath(int size);

    ~CartoPath();
    void buildANNPath(std::vector<Carto::CartoNode> *path);
    void buildTSP(std::vector<cv::Point> *path);
    int distance(cv::Point p1, cv::Point p2);
private:
    int64 tspIndex(operations_research::RoutingModel::NodeIndex from, operations_research::RoutingModel::NodeIndex to);
    int64 tspDistance(operations_research::RoutingModel::NodeIndex from, operations_research::RoutingModel::NodeIndex to) const;
    std::unique_ptr<int64[]> matrix_;
    const int size_;
};
}

#endif /* CartoPath_hpp */
