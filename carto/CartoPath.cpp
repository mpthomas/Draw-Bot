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

//DEFINE_int32(tsp_size, 3, "Size of Traveling Salesman Problem instance.");
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

    void CartoPath::buildTSP(std::vector<Point> *path) {
        std::vector<Point> parr;
        path->clear();
        
        this->matrix_.reset(new int64[size_ * size_]);
        
        for(int i=0; i != this->detected_edges.rows; i++){
            for(int j=0; j != this->detected_edges.cols; j++){
                int pixel = this->detected_edges.at<uchar>(i,j);
                if(pixel < 255) {
                    parr.push_back(Point(i,j));
                }
            }
        }
        
        //Use ints for index to Mat
        int i=0, j=0;
        
        for (RoutingModel::NodeIndex from = RoutingModel::kFirstNode; from < size_;
             ++from) {
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
    
    void CartoPath::buildANNPath(std::vector<Carto::CartoNode> *path){
    int i=0, j=0, acc=0, dim = 2, k=1, eps=0;
    int nPts=0;              // actual number of data points
    ANNpointArray dataPts; // data points
    ANNpoint queryPt;      // query point
    ANNidxArray nnIdx;     // near neighbor indices
    ANNdistArray dists;    // near neighbor distances
    ANNkd_tree* kdTree;    // search structure
    
    for(i=0; i != this->detected_edges.rows; i++){
        for(j=0; j != this->detected_edges.cols; j++){
            int pixel = this->detected_edges.at<uchar>(i,j);
            if(pixel < 255) {
                nPts++;
            }
        }
    }
    
    queryPt=annAllocPt(dim);
    dataPts=annAllocPts(nPts, dim);
    nnIdx=new ANNidx[k];
    dists = new ANNdist[k];

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
    kdTree = new ANNkd_tree(                    // build search structure
                            dataPts,                    // the data points
                            nPts,                       // number of points
                            dim);                       // dimension of space
    
    queryPt[0]=0;
    queryPt[1]=0;
    
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
        //annDeallocPt(*dataPts[nnIdx[0]]);
        queryPt[0]=n.point.x;
        queryPt[1]=n.point.y;
        
        dataPts[nnIdx[0]][0]=NULL;
        dataPts[nnIdx[0]][1]=NULL;
    }
    
    delete [] nnIdx;
    delete []  dists;
    delete kdTree;
    annClose();
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

