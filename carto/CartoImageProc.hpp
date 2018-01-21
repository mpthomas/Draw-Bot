//
//  CartoImageProc.hpp
//  carto
//
//  Created by Matthew Thomas on 1/21/18.
//  Copyright © 2018 Matthew Thomas. All rights reserved.
//

#ifndef CartoImageProc_hpp
#define CartoImageProc_hpp

#include <stdio.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

namespace Carto {

class CartoImageProc {
public:
    char *image_name;
    cv::Mat mat;

    // Constructors
    CartoImageProc();
    CartoImageProc(char *filename);

    // Destructors
    ~CartoImageProc();
    
    void setMat(cv::Mat mat);
    
    cv::Mat getMat();
    
};
}

#endif /* CartoImageProc_hpp */
