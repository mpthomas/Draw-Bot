//
//  CartoPath.hpp
//  carto
//
//  Created by Matthew Thomas on 1/20/18.
//  Copyright © 2018 Matthew Thomas. All rights reserved.
//

#ifndef CartoPath_hpp
#define CartoPath_hpp

namespace Carto {

    class CartoNode;
    
class CartoPath {
public:
    Mat detected_edges;
    
    CartoPath();
    ~CartoPath();
    void buildANNPath(std::vector<Carto::CartoNode> *path);
    int distance(cv::Point p1, cv::Point p2);
};
}

#endif /* CartoPath_hpp */
