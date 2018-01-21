#include "carto_includes.h"

struct Node {
    std::vector<Node>neighbors;
    cv::Point point;
};
using namespace cv;
using namespace Carto;


/// Global variables
///
Mat src, src_gray;
Mat dst, detected_edges, cart_img, acc_img;
//VideoCapture cap(0); // open the default camera

int edgeThresh = 1;
int lowThreshold=95;
int distanceMultiplier=1;
int const max_distanceMultiplier = 10;
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;

CartoSimulator sim;
bool windowInit=false;

void CannyThreshold(int pos,void *userData);
void RenderGraph(std::vector<CartoNode::CartoNode> nodes, Mat* img, Point start_point);
int EDistance(Point p1, Point p2);
void ProcessPic(int event,int,int,int,void*);
    
/**
 * @function CannyThreshold
 * @brief Trackbar callback - Canny thresholds input with a ratio 1:3
 */
void CannyThreshold(int pos, void *userData)
{
    /// Reduce noise with a kernel 3x3
    blur( src_gray, detected_edges, cv::Size(1.5,1.5) );
   
    
    /// Canny detector
    Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );
    imshow("Edge", detected_edges );
    /// Using Canny's output as a mask, we display our result
    dst = Scalar::all(255);
    
   // std::cout << "M = \n" << detected_edges << "\n\n";
    //src_gray.copyTo( dst, detected_edges);
    ProcessPic(EVENT_LBUTTONDOWN, 0, 0, 0, nullptr); 
}

void ProcessPic(int event,int,int,int,void*){
    Point prev_point;
    Carto::CartoPath *path = new CartoPath();
    std::vector<Carto::CartoNode> annPath;
    
    if (event != EVENT_LBUTTONDOWN)
        return;
    
    cart_img = Mat::zeros(detected_edges.rows,detected_edges.cols,CV_8UC3);
    cart_img = Scalar::all(255);
    acc_img = src.clone();
    acc_img = Scalar::all(255);
    
    path->buildANNPath(&annPath);
    
    if(path->detected_edges.rows > 0) {
        
        sim = CartoSimulator(&cart_img);
        RenderGraph(annPath, &cart_img, Point(0,0));
    }
    
    acc_img+=cart_img;
    
    imshow("Cart",cart_img);
    imshow("Accumulator",acc_img);
    
    return;
}

void RenderGraph(std::vector<CartoNode::CartoNode> nodes, Mat* img, Point start_point) {
    //Mat cart_img = Mat::zeros(detected_edges.rows,detected_edges.cols,CV_8UC3);
    int thickness = 1;
    int lineType=8;
    int colornum=0;
    int i=0,edist=0;
    Point prev_point;
    
    if(nodes.size() == 0) {
        circle(*img,start_point,1,Scalar(255,0,255),-1,8);
        return;
    }
    
    for(i=0;i<nodes.size();i++) {
        
        edist=EDistance(nodes[i].point, start_point);
        
       // std::cout << "Edistance is :" << edist << std::endl;
        
        colornum=(edist*distanceMultiplier);
        
        if(colornum > 255) {
            colornum=255;
        }
        
        //std::cout << "\n== Moving to point ==\n";
        sim.MoveToPoint(nodes[i].point,1);
        
        line(*img,start_point,nodes[i].point,Scalar(200,200,200),thickness,lineType);
        
        if(nodes[i].neighbors.size() > 0) {
            RenderGraph(nodes[i].neighbors, img, nodes[i].point);
        }
        
        start_point=nodes[i].point;
    }
}

int EDistance(Point p1, Point p2) {
    double x = (double)p1.x - (double)p2.x; //calculating number to square in next step
    double y = (double)p1.y - (double)p2.y;
    double dist;
    
    dist = pow(x, 2) + pow(y, 2);       //calculating Euclidean distance
    dist = sqrt(dist);
    
    return (int)dist;
}

void getFrame() {
    
    src=imread("/Users/matt/xcode/Cartogrifer/carto/carto/img.jpg", 1);
    // For capturing via camera
    //cap >> src;
    
    /// Convert the image to grayscale
    cvtColor( src, src_gray, CV_BGR2GRAY );
    
    if(!windowInit) {
        /// Create a matrix of the same type and size as src (for dst)
        dst.create( src.size(), src.type() );
        /// Create a window
        namedWindow("Edge", CV_WINDOW_AUTOSIZE );
        namedWindow("Cart",CV_WINDOW_AUTOSIZE);
        namedWindow("Accumulator",CV_WINDOW_AUTOSIZE);
        
        /// Create a Trackba r for user to enter threshold
        createTrackbar( "Min Threshold:", "Edge", &lowThreshold, max_lowThreshold, CannyThreshold );
        createTrackbar( "Ratio:", "Edge", &ratio, 10, CannyThreshold );
        createTrackbar( "Dst Threshold:", "Edge", &distanceMultiplier, max_distanceMultiplier, CannyThreshold );
        setMouseCallback("Edge", ProcessPic);
        waitKey(0);
    }
    
     CannyThreshold(0, 0);
}

/** @function main */
int main( int argc, char** argv )
{
    /// Load an image
    
    while(1){
        getFrame();
    }

    /// Wait until user exit program by pressing a key
    //waitKey(0);
    
    return 0;
}
