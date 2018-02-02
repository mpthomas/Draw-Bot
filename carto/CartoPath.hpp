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

namespace Carto {

    class CartoNode;
    
class CartoPath {
public:
    cv::Mat detected_edges;
    
    CartoPath();
    ~CartoPath();
    void buildANNPath(std::vector<Carto::CartoNode> *path);
    void buildTSP(std::vector<Carto::CartoNode> *path);
    int distance(cv::Point p1, cv::Point p2);
};
}

#endif /* CartoPath_hpp */
