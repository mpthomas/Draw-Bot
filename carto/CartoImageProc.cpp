
#include "CartoImageProc.hpp"
#include <iostream>
#include <sys/stat.h>
#include "PerlinNoise.hpp"
#include "CartoPath.hpp"
#include "CartoNode.hpp"
#include "CartoSimulator.hpp"
#include <gsl/gsl_errno.h>
#include <gsl/gsl_spline.h>
#include <cmath>

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
        TickMeter tm;
        tm.start();
     
        Mat tmp = Mat::zeros(inmat->rows, inmat->cols, CV_8UC3);
        tmp=Scalar::all(255);
        
        for(int x=0; x<inmat->cols; x++){
            for(int y=0; y<inmat->rows; y++) {
                uchar val=inmat->at<uchar>(Point(x,y));

                if(val >= (uchar)start && val <= (uchar)end) {
                    inmat->at<uchar>(Point(x,y))=val;
                }else{
                    inmat->at<uchar>(Point(x,y))=255;
                }
            }
        }
        tm.stop();
        std::cout << "filterGrayscale: " << tm.getTimeSec() << std::endl;
    }

    void CartoImageProc::filterPerlin(cv::Mat *inmat, double scale){
        TickMeter tm;
        tm.start();
        Mat perlin = CreatePerlinNoiseImage(Size(inmat->cols,inmat->rows),scale);
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
        tm.stop();
        std::cout << "filterPerlin: " << tm.getTimeSec() << std::endl;
    }
    
    void CartoImageProc::autoFilterPerlin(cv::Mat *inmat, double scale) {
        int max_x = inmat->cols;
        int max_y = inmat->rows;
        int block_x_size=100, block_y_size=100;
        Mat tmp_mat = inmat->clone();
        vector<Mat> channels;
        double mean, s;
        uchar inmat_val, perlin_val;
        
        for(int start_x=0; start_x< max_x; start_x+=block_x_size) {
            for(int start_y=0; start_y < max_y; start_y+=block_y_size){
                
                if(start_x+block_x_size > max_x) {
                    start_x = max_x - (start_x - block_x_size);
                    block_x_size = max_x - start_x;
                }
                
                if(start_y+block_y_size > max_y) {
                    start_y = max_x - (start_y - block_y_size);
                    block_y_size = max_y - start_y;
                }
                
                this->createMask(inmat, &tmp_mat, start_x, block_x_size, start_y, block_y_size);
                mean = cv::mean(tmp_mat)[0]/1000*-1; // Average of first and only channel
                s = .255 - mean + scale; // create a perlin scale filter relative to the mean. Offset by scale arg
                
                Mat perlin = CreatePerlinNoiseImage(Size(tmp_mat.cols, tmp_mat.rows), s);
                
                std::cout << "Mean: " << mean << " Scale: " << s << std::endl;
                
                for(int x = 0; x < perlin.cols; x++) {
                    for(int y = 0; y < perlin.rows; y++) {
                        inmat_val=inmat->at<uchar>(Point(x + start_x,y + start_y));
                        perlin_val=perlin.at<uchar>(Point(x,y));
                        
                        if(inmat_val < 255 && perlin_val < 125) {
                            inmat->at<uchar>(Point(x + start_x,y + start_y))=255;
                        }else if (inmat_val < 255){
                            inmat->at<uchar>(Point(x + start_x,y + start_y))=0;
                        }
                    }
                }
            }
        }
    }
    
    void CartoImageProc::toGrayscale() {
        cvtColor(this->mat,this->mat,CV_BGR2GRAY);
    }
    
    void CartoImageProc::show() {
        TickMeter tm;
        tm.start();
        string window_name=this->image_name;
        window_name.append(std::to_string(this->window_ctr));
        
        this->window_ctr++;
        
        namedWindow(window_name,CV_WINDOW_AUTOSIZE);
        imshow(window_name,this->mat);
        
        tm.stop();
        std::cout << "show: " << tm.getTimeSec() << std::endl;
    }
    
    void CartoImageProc::show(Mat mat, std::string name) {
        TickMeter tm;
        tm.start();
        namedWindow(name,CV_GUI_EXPANDED | WINDOW_NORMAL);
        imshow(name,mat);
        tm.stop();
        std::cout << "show: " << tm.getTimeSec() << std::endl;
    }
    
    void CartoImageProc::createMask(Mat *inmat, Mat *mask, int start_x, int len_x, int start_y, int len_y) {
        TickMeter tm;
        tm.start();
        *mask=Scalar::all(0);
        
        Rect roi = Rect(start_x, start_y, len_x, len_y);
        Mat crop(*inmat, roi);
        crop.copyTo(*mask);
        
        tm.stop();
        std::cout << "createMask: " << tm.getTimeSec() << std::endl;
    }
    
    void CartoImageProc::buildTSPath(Mat *inmat) {

    }
    
    void CartoImageProc::getPath(cv::Mat *inmat, std::vector<Carto::CartoNode> *nodes, cv::Point start_point) {
        //int target_x=1634, target_y=729;
        int target_x=1736, target_y=2052;
        Mat drawingMat = imread("/Users/matt/xcode/track_bmw.jpeg");
        CartoPath *path = new CartoPath();
        
        if(drawingMat.empty()) {
            std::cout << "Unable to find blank drawing mat";
            return;
        }
        
        cvtColor(drawingMat,drawingMat,COLOR_BGR2GRAY);
        drawingMat=Scalar::all(255);
        
        if(inmat->rows <= drawingMat.rows and inmat->cols <= drawingMat.cols) {
            inmat->copyTo(drawingMat(Rect(target_x, target_y,inmat->cols,inmat->rows)));
        }else{
            std::cout << "Mismatch for source image and drawing mat\n";
            return;
        }
        
        path->detected_edges = drawingMat.clone();
        //*inmat=drawingMat.clone();
        
        //path->buildANNPath(nodes);
        path->buildANNPath(nodes,Point(1736,2052));
        
        return;
    }
    
    void CartoImageProc::processPath(cv::Mat *inmat, std::vector<Carto::CartoNode> *nodes) {
        
        Mat drawingMat = imread("/Users/matt/xcode/track_bmw.jpeg");
        CartoPath *path = new CartoPath();
        //int target_x=1634, target_y=729;
        int target_x=1736, target_y=2052;
        if(drawingMat.empty()) {
            std::cout << "Unable to find blank drawing mat";
            return;
        }
        
        cvtColor(drawingMat,drawingMat,COLOR_BGR2GRAY);
        drawingMat=Scalar::all(255);
        
        if(inmat->rows <= drawingMat.rows and inmat->cols <= drawingMat.cols) {
            inmat->copyTo(drawingMat(Rect(target_x, target_y,inmat->cols,inmat->rows)));
        }else{
            std::cout << "Mismatch for source image and drawing mat\n";
            return;
        }
        
        path->detected_edges = drawingMat.clone();
        this->sim=new CartoSimulator(&path->detected_edges);
        
        /* Reset it. No longer need old image */
        drawingMat=Scalar::all(255);
        this->line_counter=0;

        /* Finally Render it */
        std::cout << "Start Point is: " << Point(this->sim->prev_point) << std::endl;
        std::cout << "First Node is: " << Point(nodes->at(0).point) << std::endl;
        
        this->renderPath(*nodes, &drawingMat, this->sim->prev_point);
        
        /* Reset our inmat so it can be displayed by caller */
        *inmat=drawingMat;
        
        this->sim->arduino->close();
    }
    
    void CartoImageProc::renderPath(std::vector<CartoNode> annNode, Mat *inmat, Point start_point){
        if(annNode.size() == 0) {
            return;
        }

        for(int i=0;i<annNode.size();i++) {
            this->sim->MoveToPoint(annNode[i].point,1);
            //Point simp=Point(this->sim->prev_point.x/5, this->sim->prev_point.y/5);
            Point simp=this->sim->prev_point;
            
            if(this->sim->draw_line) {
 
                line(*inmat,start_point,simp,Scalar(0,0,0),2,8);
                
                //if(this->sim->line1->color[0] < 100) {
                 //   circle(*inmat, simp, 2, Scalar(200,200,200),1);
                //}
                //std::cout << this->sim->prev_point.x << " " << this->sim->prev_point.y;
                //std::cout << " Len1: " << this->sim->line1->length << " Len2: " << this->sim->line2->length << std::endl;
            }
            
            if(annNode[i].neighbors.size() > 0) {
                //this->renderPath(annNode[i].neighbors,inmat,annNode[i].point);
                this->renderPath(annNode[i].neighbors,inmat,simp);
            }
            //start_point=annNode[i].point;
            start_point=simp;
        }
    }
    
    void CartoImageProc::reloadImage(std::string filename) {
        struct stat buffer;
        
        if(stat(filename.c_str(), &buffer) == 0) {
            this->mat = imread(filename,1);
        }else{
            cout << "CartoImageProc::reloadImage(): File not found: " << filename << endl;
        }
    }
}
