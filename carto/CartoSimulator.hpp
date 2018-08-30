//
//  CartoSimulator.hpp
//  carto
//
//  Created by Matthew Thomas on 10/8/17.
//  Copyright © 2017 Matthew Thomas. All rights reserved.
//

#ifndef CartoSimulator_hpp
#define CartoSimulator_hpp

#include <stdio.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#ifndef ANN_H
#include "ANN/ANN.h"
#endif

using namespace cv;

/*
class CartoSimulatorMotor {

private:
    std::ofstream arduino;
    
public:
    float speed, distance, target_distance;
    int motor_number;
    CartoSimulatorMotor();
    
    ~CartoSimulatorMotor();
    
    void setSpeed(float speed);
    int getSpeed();
 
    void setTargetDistance(float distance);
    
    void go(int time_target);
};
*/
class CartoSimulatorLine {
private:
    double x;
    double y;
    
    float _CalcEDistance(Point p1, Point p2);
    
public:
    Point origin, end;
    int steps_left = 0, motor_number =0;
    float length, real_length, target_distance;
    std::ofstream *arduino;
    
    TickMeter tick_meter;
    CartoSimulatorLine(Point origin, Point end = Point(0,0));
    
    ~CartoSimulatorLine();
    
    void SetTarget(Point p, int steps);
    void goFromTo(CvPoint2D32f from, Point to);
    void setMotorNumber(int number);
    void Step(int steps);

    float getEdist(Point p);
    int getStepsLeft();
};


class CartoSimulator {
    
private:
    float _CalcEDistance(Point p1, Point p2);
    
public:
    
    int current_x = 0;
    int current_y = 0;
    Mat *canvas;
    CvPoint2D32f prev_point;
    bool draw_line = false;
    std::ofstream *arduino;
    TickMeter tick_meter;
    
    Point origin;
    CartoSimulatorLine *line1;
    CartoSimulatorLine *line2;
    
    CartoSimulator();
    CartoSimulator(Mat *cvMat);
    
   ~CartoSimulator();
    
    void MoveToPoint(Point p, int steps);
    CvPoint2D32f findPoint(int side_a, int side_b, int side_c);
};

#endif /* CartoSimulator_hpp */
