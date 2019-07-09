//
//  CartoPath.cpp
//  carto
//
//  Created by Matthew Thomas on 1/20/18.
//  Copyright © 2018 Matthew Thomas. All rights reserved.
//

#include "CartoPath.hpp"
#include "CartoNode.hpp"
#include <memory>
#include <stdio.h>

#include "ortools/base/callback.h"
#include "ortools/base/commandlineflags.h"
#include "ortools/base/commandlineflags.h"
#include "ortools/base/integral_types.h"
#include "google/protobuf/text_format.h"
#include "ortools/base/join.h"
#include "ortools/base/join.h"
#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_enums.pb.h"
#include "ortools/constraint_solver/routing_flags.h"
#include "ortools/base/random.h"

DEFINE_bool(tsp_use_random_matrix, false, "Use random cost matrix.");
DEFINE_int32(tsp_random_forbidden_connections, 0,
             "Number of random forbidden connections.");
DEFINE_bool(tsp_use_deterministic_random_seed, false,
            "Use deterministic random seeds.");

using namespace operations_research;
using namespace cv;

namespace Carto {
    CartoPath::CartoPath() : size_(0) {}
    CartoPath::CartoPath(int size) : size_(size) {}
    CartoPath::~CartoPath() {}
    Mat img;
    
    void CartoPath::buildTSP(std::vector<Point> *path) {
        std::vector<Point> parr;
        path->clear();
        parr.clear();
        
        this->matrix_.reset(new int64[size_ * size_]);
        
        for(int i=0; i < this->detected_edges.cols; i++){
            for(int j=0; j < this->detected_edges.rows; j++){
                int pixel = this->detected_edges.at<uchar>(Point(i,j));
                if(pixel < 255) {
                    parr.push_back(Point(i,j));
                    std::cout << "Point: " << Point(i,j) << "Pixel: " << pixel << std::endl;
                }
            }
        }
        
        //Use ints for index to Mat
        int i=0, j=0;
        
        for (RoutingModel::NodeIndex from = RoutingModel::kFirstNode; from < size_;
             ++from) {
            j=0;
            
            for (RoutingModel::NodeIndex to = RoutingModel::kFirstNode; to < size_;
                 ++to) {
                if (to != from) {
                    matrix_[tspIndex(from, to)] = (int64)this->distance(parr[i],parr[j]);
                } else {
                    matrix_[tspIndex(from, to)] = 0LL;
                }
                j++;
            }
            i++;
        }
        
        RoutingModel routing(this->size_, 1, RoutingModel::NodeIndex(0));
        RoutingSearchParameters parameters = BuildSearchParametersFromFlags();
        // Setting first solution heuristic (cheapest addition).
        parameters.set_first_solution_strategy(
                                               FirstSolutionStrategy::PATH_CHEAPEST_ARC);
        parameters.set_solution_limit(5);
        
        routing.SetArcCostEvaluatorOfAllVehicles(NewPermanentCallback(this, &CartoPath::tspDistance));
        
        const Assignment* solution = routing.SolveWithParameters(parameters);
        
        if ( solution != nullptr) {
            const int route_number = 0;
            std::string route;
            
            for(int64 node = routing.Start(route_number); !routing.IsEnd(node);
                node = solution->Value(routing.NextVar(node))) {
                
                //std::cout << parr[node];
                path->push_back(parr[node]);
            }
            const int64 end = routing.End(route_number);
            //std::cout << parr[end];
            path->push_back(parr[end]);
            
        }else{
            std::cout << "No solution found\n";
        }

    }
    
    int64 CartoPath::tspDistance(RoutingModel::NodeIndex from, RoutingModel::NodeIndex to) const {
        return (from * size_ + to).value();
    }
    
    int64 CartoPath::tspIndex(RoutingModel::NodeIndex from, RoutingModel::NodeIndex to) {
        return (from * this->size_ + to).value();
    }
    
    void CartoPath::buildANNPath(std::vector<Carto::CartoNode> *path, CvPoint start_point){
        TickMeter tm;
        tm.start();
        int i=0, j=0, acc=0, dim = 2, k=2, eps=0;
        int nPts=0;              // actual number of data points
        ANNpointArray dataPts; // data points
        ANNpoint queryPt;      // query point
        ANNidxArray nnIdx;     // near neighbor indices
        ANNdistArray dists;    // near neighbor distances
        ANNkd_tree* kdTree;    // search structure
        std::vector<Point> point_long_path, point_short_path;
        
        for(i=0; i != this->detected_edges.rows; i++){
            for(j=0; j != this->detected_edges.cols; j++){
                int pixel = this->detected_edges.at<uchar>(i,j);
                if(pixel < 255) {
                    nPts++;
                }
            }
        }
        tm.stop();
        std::cout << "buildANNPath (nPts loop) took: " << tm.getTimeSec() << std::endl;
        tm.reset();
        
        tm.start();
        queryPt=annAllocPt(dim);
        dataPts=annAllocPts(nPts, dim);
        tm.stop();
        std::cout << "buildANNPath (annAllocPts allocations) took: " << tm.getTimeSec() << std::endl;
        tm.reset();
        
        nnIdx=new ANNidx[k];
        dists = new ANNdist[k];
        
        tm.start();
        
        for(i=0; i != this->detected_edges.rows; i++){
            for(j=0; j != this->detected_edges.cols; j++){
                int pixel = this->detected_edges.at<uchar>(i,j);
                if(pixel < 255) {
                    dataPts[acc][0] = (double)j;
                    dataPts[acc][1] = (double)i;
                    
                    // std::cout << "Set Data points: " << dataPts[acc][0] << std::endl;
                    acc++;
                }
            }
        }
        
        tm.stop();
        std::cout << "buildANNPath (set dataPts loop) took: " << tm.getTimeSec() << std::endl;
        tm.reset();
        
        tm.start();
        kdTree = new ANNkd_tree(                    // build search structure
                                dataPts,                    // the data points
                                nPts,                       // number of points
                                dim);                       // dimension of space
        
        tm.stop();
        std::cout << "buildANNPath (new ANNkd_tree) took: " << tm.getTimeSec() << std::endl;
        tm.reset();
        
        queryPt[0]=start_point.x;
        queryPt[1]=start_point.y;
        
        tm.start();
        for(i=0;i<nPts;i++){
            kdTree->annkPriSearch(                     // search
                                  queryPt,                        // query point
                                  k,                              // number of near neighbors
                                  nnIdx,                          // nearest neighbors (returned)
                                  dists,                          // distance (returned)
                                  eps);                           // error bound
            Carto::CartoNode n;
            n.point=Point(dataPts[nnIdx[0]][0], dataPts[nnIdx[0]][1]);
            
            path->push_back(n);
            point_long_path.push_back(Point(n.point));
            
            //annDeallocPt(*dataPts[nnIdx[0]]);
            queryPt[0]=n.point.x;
            queryPt[1]=n.point.y;
            
            dataPts[nnIdx[0]][0]=NULL;
            dataPts[nnIdx[0]][1]=NULL;
        }
        
        tm.stop();
        std::cout << "buildANNPath (annkPriSearch loop) took: " << tm.getTimeSec() << std::endl;
        tm.reset();
        
        tm.start();
        delete [] nnIdx;
        delete []  dists;
        delete kdTree;
        annClose();
        tm.stop();
        std::cout << "buildANNPath (cleanup) took: " << tm.getTimeSec() << std::endl;
        tm.reset();
        
        /* Finally try to reduce the path length */
        point_short_path.resize(point_long_path.size());
        //for(int i = 0; i < point_long_path.size()-4; i+=4) {
            approxPolyDP(point_long_path, point_short_path, 3, false);
         //   i=i;
        //}
        
        if(img.rows == 0) {
            img = Mat::zeros(this->detected_edges.cols-1634, this->detected_edges.rows-729, CV_8UC1);
            img=Scalar::all(255);
        }
        
        for(int i =0 ; i < point_short_path.size()-1;i++) {
            line(img,point_short_path[i],point_short_path[i+1],Scalar(0,0,0),1,8);
            //circle(img, point_short_path[i], 2, Scalar(0,0,0),-1);
        }
        
        std::cout << "Short Path: " << point_short_path.size() << " Long Path: " << point_long_path.size() << std::endl;
        
        namedWindow("Short Path",CV_GUI_EXPANDED | WINDOW_NORMAL);
        //imshow("Short Path",img);
        
        // Create CartoNode path using shorter path
        path->clear();
        
        // change loop to either point_short_path or point_long_path to determine which is returned
        for(int i =0; i < point_long_path.size(); i++) {
            Carto::CartoNode node;
            node.point=Point(point_long_path[i]);
            path->push_back(node);
        }
}

int CartoPath::distance(Point p1, Point p2) {
    double x = (double)p1.x - (double)p2.x; //calculating number to square in next step
    double y = (double)p1.y - (double)p2.y;
    double dist;
    
    dist = pow(x, 2) + pow(y, 2);       //calculating Euclidean distance
    dist = sqrt(dist);
    
    return (int)dist;
}

}

