//
//  CartoImageProc.cpp
//  carto
//
//  Created by Matthew Thomas on 1/21/18.
//  Copyright © 2018 Matthew Thomas. All rights reserved.
//

#include "CartoImageProc.hpp"

using namespace cv;

namespace Carto {

    CartoImageProc::CartoImageProc() {
        
    }
    
    CartoImageProc::CartoImageProc(char *filename) {
        this->image_name=filename;
        this->mat = imread(filename,1);
    }
    
    CartoImageProc::~CartoImageProc() {}
    
    void CartoImageProc::setMat(Mat mat){
        this->mat=mat;
    }
}
