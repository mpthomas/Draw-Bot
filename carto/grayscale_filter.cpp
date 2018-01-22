//
//  grayscale_filter.cpp
//  carto
//
//  Created by Matthew Thomas on 1/21/18.
//  Copyright © 2018 Matthew Thomas. All rights reserved.
//

#include <stdlib.h>
#include <string>
#include "CartoImageProc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace Carto;
using namespace cv;

int start = 0, end = 150;
CartoImageProc *img;

void refresh(int pos, void *userData);
int main( int argc, char** argv ){
    img=new CartoImageProc("/Users/matt/xcode/Cartogrifer/carto/carto/img.jpg");
    
    img->toGrayscale();
    
    Mat newmat = img->filterGrayscale(0,150);
    img->show(newmat,"0 to 150");
    
    newmat = img->filterGrayscale(151,200);
    img->show(newmat,"151 to 200");
    
    newmat = img->filterGrayscale(200,255);
    img->show(newmat,"200 to 255");
    
    newmat = img->filterGrayscale(151,255);
    img->show(newmat,"151 to 255");
    
    createTrackbar("Start","0 to 150",&start, 50, refresh);
    createTrackbar("End","0 to 150",&end, 200, refresh);
    
    waitKey(0);
}

void refresh(int pos, void *userData) {
    Mat newmat = img->filterGrayscale(start,end);
    img->show(newmat,"0 to 150");
    
}
