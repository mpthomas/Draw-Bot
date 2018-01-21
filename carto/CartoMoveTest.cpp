//
//  CartoMoveTest.cpp
//  carto
//
//  Created by Matthew Thomas on 10/10/17.
//  Copyright © 2017 Matthew Thomas. All rights reserved.
//

#include <stdio.h>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "CartoMoveTest.hpp"

using namespace cv;

CartoMoveTest::CartoMoveTest() {}
CartoMoveTest::~CartoMoveTest() {}

cv::Point CartoMoveTest::getIntersection(Point p1, double p1_radius, Point p2, double p2_radius){
    this->circle1.a=p1.x;
    this->circle1.b=p1.y;
    this->circle1.r=p1_radius;
    
    this->circle2.a=p2.x;
    this->circle2.b=p2.y;
    this->circle2.r=p2_radius;
    
    std::cout << p1 << p1_radius << p2 << p2_radius << std::endl;
    
    if (this->twoCirclesIntersection(circle1, circle2)) {
        // If true - then the circles intersects
        // the intersection points are given by (x1, y1) and (x2, y2)
        if(this->y1 < 0)
            return Point((int)this->x1,abs((int)this->y1));
        else if(this->y2 < 0)
            return Point((int)this->x2,abs((int)this->y2));
        else {
            std::cout << "Both intersections had positive y!" << std::endl;
            return Point(0,0);
        }
    }else{
        std::cout << "No intersection found!" << std::endl;
        return Point(0,0);
        
    }
}

bool CartoMoveTest::twoCirclesIntersection(Circle c1, Circle c2) {
    //**************************************************************
    //Calculating intersection coordinates (x1, y1) and (x2, y2) of
    //two circles of the form (x - c1.a)^2 + (y - c1.b)^2 = c1.r^2
    //                        (x - c2.a)^2 + (y - c2.b)^2 = c2.r^2
    //
    // Return value:   true if the two circles intersects
    //                 false if the two circles do not intersects
    //**************************************************************
    double val1, val2, test,D;
    
    // Calculating distance between circles centers
    D = sqrt((c1.a - c2.a) * (c1.a - c2.a) + (c1.b - c2.b) * (c1.b - c2.b));
    
    if (((c1.r + c2.r) >= D) && (D >= fabs(c1.r - c2.r))) {
        // Two circles intersects or tangent
        // Area according to Heron's formula
        //----------------------------------
        double a1 = D + c1.r + c2.r;
        double a2 = D + c1.r - c2.r;
        double a3 = D - c1.r + c2.r;
        double a4 = -D + c1.r + c2.r;
        double area = sqrt(a1 * a2 * a3 * a4) / 4;
        
        // Calculating x axis intersection values
        //---------------------------------------
        val1 = (c1.a + c2.a) / 2 + (c2.a - c1.a) * (c1.r * c1.r - c2.r * c2.r) / (2 * D * D);
        val2 = 2 * (c1.b - c2.b) * area / (D * D);
        this->x1 = val1 + val2;
        this->x2 = val1 - val2;
        
        // Calculating y axis intersection values
        //---------------------------------------
        val1 = (c1.b + c2.b) / 2 + (c2.b - c1.b) * (c1.r * c1.r - c2.r * c2.r) / (2 * D * D);
        val2 = 2 * (c1.a - c2.a) * area / (D * D);
        this->y1 = val1 - val2;
        this->y2 = val1 + val2;
        
        // Intersection pointsare (x1, y1) and (x2, y2)
        // Because for every x we have two values of y, and the same thing for y,
        // we have to verify that the intersection points as chose are on the
        // circle otherwise we have to swap between the points
        test = fabs((x1 - c1.a) * (x1 - c1.a) + (y1 - c1.b) * (y1 - c1.b) - c1.r * c1.r);
        
        if (test > 0.0000001) {
            // point is not on the circle, swap between y1 and y2
            // the value of 0.0000001 is arbitrary chose, smaller values are also OK
            // do not use the value 0 because of computer rounding problems
            double tmp = y1;
            y1 = y2;
            y2 = tmp;
        }
        return true;
    }
    else {
        // circles are not intersecting each other
        return false;
    }
    
    return false;
}
