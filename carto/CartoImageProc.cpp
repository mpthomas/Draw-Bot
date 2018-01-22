//
//  CartoImageProc.cpp
//  carto
//
//  Created by Matthew Thomas on 1/21/18.
//  Copyright © 2018 Matthew Thomas. All rights reserved.
//

#include "CartoImageProc.hpp"
#include <iostream>
#include <sys/stat.h>

namespace Carto {

    using namespace cv;
    using namespace std;
    
    CartoImageProc::CartoImageProc() {
        this->image_name.assign("CartoImageProc");
    }
    
    CartoImageProc::CartoImageProc(std::string filename) {
        struct stat buffer;

        if(stat(filename.c_str(), &buffer) == 0) {
            this->image_name.assign(filename);
            this->mat = imread(filename,1);
        }else{
            cout << "File not found: " << filename << endl;
            exit(1);
        }
    }
    
    CartoImageProc::~CartoImageProc() {}
    
    void CartoImageProc::setMat(Mat mat){
        this->mat=mat.clone();
    }
    
    cv::Mat CartoImageProc::filterGrayscale(int start, int end){
        Mat filtered;
        
        this->toGrayscale();
        filtered=this->mat.clone();
        filtered=Scalar::all(255);
        
        for(int x=0; x<this->mat.cols; x++){
            for(int y=0; y<this->mat.rows; y++) {
                uchar val=this->mat.at<uchar>(Point(x,y));

                if(val > (uchar)start && val < (uchar)end) {
                    filtered.at<uchar>(Point(x,y))=val;
                }
            }
        }
        return filtered;
    }
    
    void CartoImageProc::toGrayscale() {
        cvtColor(this->mat,this->mat,CV_BGR2GRAY);
    }
    
    void CartoImageProc::show() {
        string window_name=this->image_name;
        window_name.append(std::to_string(this->window_ctr));
        
        this->window_ctr++;
        
        namedWindow(this->image_name,CV_WINDOW_AUTOSIZE);
        imshow(this->image_name,this->mat);
        waitKey(0);
    }
}
