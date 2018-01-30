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
#include "PerlinNoise.hpp"
#include "CartoPath.hpp"
#include "CartoNode.hpp"
#include "CartoSimulator.hpp"

namespace Carto {

    using namespace cv;
    using namespace std;
    
    CartoImageProc::CartoImageProc() {
        this->image_name.assign("CartoImageProc");
    }
    
    CartoImageProc::CartoImageProc(std::string filename, int id) {
        struct stat buffer;

        if(stat(filename.c_str(), &buffer) == 0) {
            this->image_name.assign(filename);
            this->mat = imread(filename,1);
        }else{
            cout << "File not found: " << filename << endl;
            exit(1);
        }
        
        this->id=id;
    }
    
    CartoImageProc::~CartoImageProc() {}
    
    void CartoImageProc::setMat(Mat mat){
        this->mat=mat.clone();
    }
    
    void CartoImageProc::filterGrayscale(cv::Mat *inmat, int start, int end){
        Mat tmp = Mat::zeros(inmat->rows, inmat->cols, CV_8UC3);
        tmp=Scalar::all(255);
        
        for(int x=0; x<inmat->cols; x++){
            for(int y=0; y<inmat->rows; y++) {
                uchar val=inmat->at<uchar>(Point(x,y));

                if(val > (uchar)start && val < (uchar)end) {
                    inmat->at<uchar>(Point(x,y))=val;
                }else{
                    inmat->at<uchar>(Point(x,y))=255;
                }
            }
        }
    }

    void CartoImageProc::filterPerlin(cv::Mat *inmat, double scale){
        //Mat tmp = Mat::zeros(inmat->rows, inmat->cols, CV_8UC3);
        //tmp=Scalar::all(255);
        
        Mat perlin = CreatePerlinNoiseImage(Size(inmat->cols,inmat->rows),scale);
        //imshow("151 to 200",perlin);
        uchar inmat_val, perlin_val;
        
        for(int x=0; x<inmat->cols; x++){
            for(int y=0; y<inmat->rows; y++) {
                inmat_val=inmat->at<uchar>(Point(x,y));
                perlin_val=perlin.at<uchar>(Point(x,y));
                
                if(inmat_val < 255 && perlin_val < 125) {
                    inmat->at<uchar>(Point(x,y))=255;
                }else if (inmat_val < 255){
                    inmat->at<uchar>(Point(x,y))=0;
                }
            }
        }
    }
    
    void CartoImageProc::toGrayscale() {
        cvtColor(this->mat,this->mat,CV_BGR2GRAY);
    }
    
    void CartoImageProc::show() {
        string window_name=this->image_name;
        window_name.append(std::to_string(this->window_ctr));
        
        this->window_ctr++;
        
        namedWindow(window_name,CV_WINDOW_AUTOSIZE);
        imshow(window_name,this->mat);
    }
    
    void CartoImageProc::show(Mat mat, std::string name) {
        namedWindow(name);
        imshow(name,mat);
    }
    
    void CartoImageProc::buildPath(Mat *inmat) {
        CartoPath *path = new CartoPath();
        std::vector<CartoNode> annPath;
        
        path->detected_edges=inmat->clone();
        *inmat=Scalar::all(255);
        
        path->buildANNPath(&annPath);
        
        std::cout << "Found" << path->detected_edges.rows << " edges" << std::endl;
        
        if(path->detected_edges.rows == 0) {
            std::cout << "No rows found" << std::endl;
            return;
        }
        
        this->sim=new CartoSimulator::CartoSimulator(inmat);
        this->renderPath(annPath, inmat, Point(0,0));
    }
    
    void CartoImageProc::renderPath(std::vector<CartoNode::CartoNode> annNode, Mat *inmat, Point start_point){
        if(annNode.size() == 0) {
            return;
        }
        
        //std::cout << "CartoImageProc::renderPath point: " << start_point << std::endl;
        
        for(int i=0;i<annNode.size();i++) {
            this->sim->MoveToPoint(annNode[i].point,1);
            
            line(*inmat,start_point,annNode[i].point,Scalar(200,200,200),1,8);
            
            if(annNode[i].neighbors.size() > 0) {
                this->renderPath(annNode[i].neighbors,inmat,annNode[i].point);
            }
            
            start_point=annNode[i].point;
        }
    }
}
