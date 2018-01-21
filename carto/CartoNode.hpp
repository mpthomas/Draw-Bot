//
//  CartoNode.hpp
//  carto
//
//  Created by Matthew Thomas on 1/20/18.
//  Copyright © 2018 Matthew Thomas. All rights reserved.
//

#ifndef CartoNode_hpp
#define CartoNode_hpp

#include "carto_includes.h"

using namespace cv;
namespace Carto {
class CartoNode {
public:
    CartoNode();
    ~CartoNode();
    std::vector<CartoNode>neighbors;
    Point point;
};
}
#endif /* CartoNode_hpp */
