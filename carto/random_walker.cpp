//
//  random_walker.cpp
//  carto
//
//  Created by Matthew Thomas on 1/21/18.
//  Copyright © 2018 Matthew Thomas. All rights reserved.
//

#include "random_walker.hpp"

#include <stdlib.h>
#include <string>
#include "CartoImageProc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace Carto;
using namespace cv;

int main( int argc, char** argv ){
    CartoImageProc *img=new CartoImageProc("/Users/matt/xcode/Cartogrifer/carto/carto/img.jpg");
    
    Mat newmat = img->filterGrayscale(200,255);
    
    img->setMat(newmat);
    img->show();
    
    newmat = img->filterGrayscale(200,255);
    
    img->setMat(newmat);
    img->show();
}
