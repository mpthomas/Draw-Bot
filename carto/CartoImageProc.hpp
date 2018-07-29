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
#include "CartoPath.hpp"
#include "CartoNode.hpp"
#include "CartoSimulator.hpp"

namespace Carto {

class CartoImageProc {
public:
    std::string image_name;
    int window_ctr,id, line_counter;
    cv::Mat mat;
    CartoSimulator *sim;

    // Constructors
    CartoImageProc();
    CartoImageProc(std::string filename, int id=0);

    // Destructors
    ~CartoImageProc();
    
    void setMat(cv::Mat mat);
    
    cv::Mat getMat();
    
    cv::Mat filterGrayscale(int start, int end);
    void filterGrayscale(cv::Mat *inmat, int start, int end);
    void filterPerlin(cv::Mat *inmat, double scale);
    void createMask(cv::Mat *inmat, cv::Mat *mask, int start_x, int len_x, int start_y, int len_y);
    void buildTSPath(cv::Mat *inmat);
    void buildPath(cv::Mat *inmat);
    void renderPath(std::vector<CartoNode::CartoNode>, cv::Mat *inmat, cv::Point start_point);
    void toGrayscale();
    void show();
    void show(cv::Mat mat, std::string name);
    
};
}

#endif /* CartoImageProc_hpp */
