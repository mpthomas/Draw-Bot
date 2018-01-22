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
#include <string>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

namespace Carto {

class CartoImageProc {
public:
    std::string image_name;
    int window_ctr;
    cv::Mat mat;

    // Constructors
    CartoImageProc();
    CartoImageProc(std::string filename);

    // Destructors
    ~CartoImageProc();
    
    void setMat(cv::Mat mat);
    
    cv::Mat getMat();
    
    cv::Mat filterGrayscale(int start, int end);
    void toGrayscale();
    void show();
    
};
}

#endif /* CartoImageProc_hpp */
