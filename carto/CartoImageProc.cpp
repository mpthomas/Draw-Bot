
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
        string window_name=this->image_name;
        window_name.append(std::to_string(this->window_ctr));
        
        this->window_ctr++;
        
        namedWindow(window_name,CV_WINDOW_AUTOSIZE);
        imshow(window_name,this->mat);
    }
    
    void CartoImageProc::show(Mat mat, std::string name) {
        namedWindow(name,CV_GUI_EXPANDED | WINDOW_NORMAL);
        imshow(name,mat);
    }
    
    void CartoImageProc::createMask(Mat *inmat, Mat *mask, int start_x, int len_x, int start_y, int len_y) {
        *mask=Scalar::all(0);
        
        Rect roi = Rect(start_x, start_y, len_x, len_y);
        Mat crop(*inmat, roi);
        crop.copyTo(*mask);
    }
    
    void CartoImageProc::buildTSPath(Mat *inmat) {
        int size=0;
        int base_block_size=25;
        int x_block_size=0;
        int y_block_size=0;
        int inmat_xmax=0;
        int inmat_ymax=0;
        int xctr=0;
        int yctr=0;
        CartoPath *cp;
        
        Mat mask=inmat->clone();
        Mat tmp;
        mask=Scalar::all(255);
        
        inmat_xmax=inmat->cols; inmat_ymax=inmat->rows;
        
        std::vector<Point> path, finalpath;
        Point from=Point(0,0);
        // go top left to bottom right
        // TODO: clean up
        do {
            xctr=0;
            
            if(yctr+y_block_size > inmat_ymax)
                y_block_size=inmat_ymax-yctr;
            else
                y_block_size=base_block_size;
            
            do{
                if(xctr+x_block_size > inmat_xmax)
                    x_block_size=inmat_xmax-xctr;
                else
                    x_block_size=base_block_size;
                
                this->createMask(inmat, &mask, xctr, x_block_size, yctr, y_block_size);

                for(int i=0; i<mask.cols; i++) {
                    for(int j=0; j < mask.rows; j++){
                        if(mask.at<uchar>(Point(i,j)) < 255) {
                            size++;
                        }
                    }
                }
                
                if(size > 1 && size < base_block_size*base_block_size) {
                    std::cout << "Start x: " << xctr << " Start y: " << yctr << " Size: " << size << std::endl;
                    
                    cp = new CartoPath(size);
                    cp->detected_edges=mask;
                    cp->buildTSP(&path);
                    
                    for(int i=0;i<path.size();i++) {
                        path[i]=path[i]+Point(xctr,yctr);
                        line(*inmat,from,path[i],Scalar(200,200,200),1,8);
                        from=path[i];
                    }
                    this->show(*inmat,"TSP Path");
                    finalpath.insert(finalpath.end(), path.begin(), path.end());
                    path.clear();
                    return;
                }
                xctr+=x_block_size;
                size=0;
            } while(xctr != inmat_xmax);
            yctr+=y_block_size;
        } while(yctr != inmat_ymax);
        
        for(int i=0;i<finalpath.size();i++) {
           // line(*inmat,from,finalpath[i],Scalar(200,200,200),1,8);
            from=finalpath[i];
        }
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
        
        this->sim=new CartoSimulator(inmat);
        this->line_counter=0;
        this->renderPath(annPath, inmat, this->sim->prev_point);
    }
    
    void CartoImageProc::renderPath(std::vector<CartoNode> annNode, Mat *inmat, Point start_point){
        if(annNode.size() == 0) {
            return;
        }
        
        //std::cout << "CartoImageProc::renderPath point: " << start_point << std::endl;
        
        for(int i=0;i<annNode.size();i++) {
            this->sim->MoveToPoint(annNode[i].point,1);
            //Point simp=Point(this->sim->prev_point.x/5, this->sim->prev_point.y/5);
            Point simp=this->sim->prev_point;
            
            if(this->sim->draw_line) {
                //line(*inmat,start_point,annNode[i].point,Scalar(200,200,200),1,8);
                //start_point=annNode[i].point;
                line(*inmat,start_point,simp,Scalar(200,200,200),1,8);
                start_point=simp;
                
                std::cout << this->sim->prev_point.x << " " << this->sim->prev_point.y;
                std::cout << " Len1: " << this->sim->line1->length << " Len2: " << this->sim->line2->length << std::endl;
            }
            
            if(annNode[i].neighbors.size() > 0) {
                //this->renderPath(annNode[i].neighbors,inmat,annNode[i].point);
                this->renderPath(annNode[i].neighbors,inmat,simp);
            }
            //start_point=annNode[i].point;
            start_point=simp;
        }
    }
}
