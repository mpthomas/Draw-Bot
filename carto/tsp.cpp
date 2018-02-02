// Copyright 2010-2017 Google
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//
// Traveling Salesman Sample.
//
// This is a sample using the routing library to solve a Traveling Salesman
// Problem.
// The description of the problem can be found here:
// http://en.wikipedia.org/wiki/Travelling_salesman_problem.
// For small problems one can use the hamiltonian path library directly (cf
// graph/hamiltonian_path.h).
// The optimization engine uses local search to improve solutions, first
// solutions being generated using a cheapest addition heuristic.
// Optionally one can randomly forbid a set of random connections between nodes
// (forbidden arcs).

#include <memory>

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

#include "CartoImageProc.hpp"
#include "CartoPath.hpp"

using namespace cv;
using namespace Carto;

std::vector< std::vector<int64> > dist_matrix;
Mat *image=new Mat(3,3,CV_8UC1);
int size;

DEFINE_int32(tsp_size, 3, "Size of Traveling Salesman Problem instance.");
DEFINE_bool(tsp_use_random_matrix, false, "Use random cost matrix.");
DEFINE_int32(tsp_random_forbidden_connections, 0,
             "Number of random forbidden connections.");
DEFINE_bool(tsp_use_deterministic_random_seed, false,
            "Use deterministic random seeds.");

namespace operations_research {
    
    // Random seed generator.
    int32 GetSeed() {
        if (FLAGS_tsp_use_deterministic_random_seed) {
            return ACMRandom::DeterministicSeed();
        } else {
            return ACMRandom::HostnamePidTimeSeed();
        }
    }
    
    void init() {
        CartoPath cp=CartoPath();
        std::vector<Point> parr;
        
        image->at<uchar>(0,0)=1;
        image->at<uchar>(0,5)=1;
        image->at<uchar>(0,99)=1;
        
        for(int i=0;i<image->cols;i++){
            for( int j=0;j<image->rows;j++)
                if(image->at<uchar>(i,j) > 0 ) {
                    parr.push_back(Point(i,j));
                }
        }
        
       // dist_matrix=new Mat((int)parr.size(),(int)parr.size() ,CV_32FC1);
        //*dist_matrix=Scalar::all(0);
        size=(int)parr.size();
        
        for (int i = 0; i < size; i++) {
            vector<int> row; // Create an empty row
            for (int j = 0; j < parr.size(); j++) {
                row.push_back(0); // Add an element (column) to the row
            }
            dist_matrix.push_back(row); // Add the row to the main vector
            dist_matrix.
        }
        
        for(int64 from=0;from<parr.size();from++) {
            for(int64 to=0;to<parr.size();to++) {
                dist_matrix[from,to]=cp.distance(parr.at(from), parr.at(to));
            }
        }
        
    }
    
    int64 getIndex(RoutingModel::NodeIndex from,
                      RoutingModel::NodeIndex to) const {
        return (from * size_ + to).value();
    }
    std::unique_ptr<int64[]> matrix_;
    const int size_;
    
    // Cost/distance functions.
    
    // Sample function.
    int64 MyDistance(RoutingModel::NodeIndex from, RoutingModel::NodeIndex to) {
        return (dist_matrix[from,to]);
    }
    
    // Random matrix.
    class RandomMatrix {
    public:
        explicit RandomMatrix(int size) : size_(size) {}
        void Initialize() {
            matrix_.reset(new int64[size_ * size_]);
            const int64 kDistanceMax = 100;
            ACMRandom randomizer(GetSeed());
            for (RoutingModel::NodeIndex from = RoutingModel::kFirstNode; from < size_;
                 ++from) {
                for (RoutingModel::NodeIndex to = RoutingModel::kFirstNode; to < size_;
                     ++to) {
                    if (to != from) {
                        matrix_[MatrixIndex(from, to)] = randomizer.Uniform(kDistanceMax);
                    } else {
                        matrix_[MatrixIndex(from, to)] = 0LL;
                    }
                }
            }
        }
        int64 Distance(RoutingModel::NodeIndex from,
                       RoutingModel::NodeIndex to) const {
            return matrix_[MatrixIndex(from, to)];
        }
        
    private:
        int64 MatrixIndex(RoutingModel::NodeIndex from,
                          RoutingModel::NodeIndex to) const {
            return (from * size_ + to).value();
        }
        std::unique_ptr<int64[]> matrix_;
        const int size_;
    };
    
    void Tsp() {
        //initPoints();
        init();
        if (FLAGS_tsp_size > 0) {
            // TSP of size FLAGS_tsp_size.
            // Second argument = 1 to build a single tour (it's a TSP).
            // Nodes are indexed from 0 to FLAGS_tsp_size - 1, by default the start of
            // the route is node 0.
            RoutingModel routing(FLAGS_tsp_size, 1, RoutingModel::NodeIndex(0));
            RoutingSearchParameters parameters = BuildSearchParametersFromFlags();
            // Setting first solution heuristic (cheapest addition).
            parameters.set_first_solution_strategy(
                                                   FirstSolutionStrategy::PATH_CHEAPEST_ARC);
            
            // Setting the cost function.
            // Put a permanent callback to the distance accessor here. The callback
            // has the following signature: ResultCallback2<int64, int64, int64>.
            // The two arguments are the from and to node inidices.
            RandomMatrix matrix(FLAGS_tsp_size);
            if (FLAGS_tsp_use_random_matrix) {
                matrix.Initialize();
                routing.SetArcCostEvaluatorOfAllVehicles(
                                                         NewPermanentCallback(&matrix, &RandomMatrix::Distance));
            } else {
                routing.SetArcCostEvaluatorOfAllVehicles(
                                                         NewPermanentCallback(MyDistance));
            }
            // Forbid node connections (randomly).
            /*
            ACMRandom randomizer(GetSeed());
            int64 forbidden_connections = 0;
            while (forbidden_connections < FLAGS_tsp_random_forbidden_connections) {
                const int64 from = randomizer.Uniform(FLAGS_tsp_size - 1);
                const int64 to = randomizer.Uniform(FLAGS_tsp_size - 1) + 1;
                if (routing.NextVar(from)->Contains(to)) {
                    LOG(INFO) << "Forbidding connection " << from << " -> " << to;
                    routing.NextVar(from)->RemoveValue(to);
                    ++forbidden_connections;
                }
            }
            */
            // Solve, returns a solution if any (owned by RoutingModel).
            const Assignment* solution = routing.SolveWithParameters(parameters);
            if (solution != nullptr) {
                // Solution cost.
                LOG(INFO) << "Cost " << solution->ObjectiveValue();
                // Inspect solution.
                // Only one route here; otherwise iterate from 0 to routing.vehicles() - 1
                const int route_number = 0;
                std::string route;
                for (int64 node = routing.Start(route_number); !routing.IsEnd(node);
                     node = solution->Value(routing.NextVar(node))) {
                    StrAppend(&route, routing.IndexToNode(node).value(), " (", node,
                              ") -> ");
                }
                const int64 end = routing.End(route_number);
                StrAppend(&route, routing.IndexToNode(end).value(), " (", end, ")");
                LOG(INFO) << route;
                std::cout << route;
            } else {
                LOG(INFO) << "No solution found.";
                std::cout << "No solution found";
            }
        } else {
            LOG(INFO) << "Specify an instance size greater than 0.";
        }
    }
}  // namespace operations_research

int main(int argc, char** argv) {
    std::cout << "Starting" << std::endl;
    gflags::ParseCommandLineFlags( &argc, &argv, true);
    operations_research::Tsp();
    return 0;
}
