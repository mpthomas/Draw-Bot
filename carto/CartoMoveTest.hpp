//
//  CartoMoveTest.hpp
//  carto
//
//  Created by Matthew Thomas on 10/11/17.
//  Copyright © 2017 Matthew Thomas. All rights reserved.
//

#ifndef CartoMoveTest_h
#define CartoMoveTest_h

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;

class Circle {
public:
    Circle(){}
    double a, b, r;
};

class CartoMoveTest {
private:
    double x1, y1, x2, y2;
    
public:
    Circle circle1, circle2;
    
    CartoMoveTest();
    ~CartoMoveTest();
    cv::Point getIntersection(cv::Point p1, double p1_radius, cv::Point p2, double p2_radius);
    bool twoCirclesIntersection(Circle c1, Circle c2);
    
};


#endif /* CartoMoveTest_h */
