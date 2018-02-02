#include <memory>

/*
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
*/
//#include "CartoImageProc.hpp"
#include "CartoPath.hpp"
#include "CartoNode.hpp"

using namespace cv;
using namespace Carto;

std::vector< std::vector<int64> > dist_matrix;
Mat *image=new Mat(3,100,CV_8UC1);
int size;

int main(int argc, char** argv) {
    *image=Scalar::all(255);
    image->at<uchar>(0,5)=1;
    image->at<uchar>(0,1)=1;
    image->at<uchar>(0,99)=1;
    
    CartoPath *cp= new CartoPath(3);
    cp->detected_edges=*image;
    
    std::cout << "Starting" << std::endl;
//    gflags::ParseCommandLineFlags( &argc, &argv, true);
    //operations_research::Tsp();
    cp->buildTSP(new std::vector<Point>);
   
    return 0;
}
