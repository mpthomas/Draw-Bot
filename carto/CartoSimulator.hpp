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
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#ifndef ANN_H
#include "ANN/ANN.h"
#endif

using namespace cv;

class CartoSimulatorMotor {

private:
    std::ofstream arduino;
    
public:
    float speed, distance, target_distance;
    int motor_number;
    CartoSimulatorMotor();
    
    ~CartoSimulatorMotor();
    
    void setMotorNumber(int number);
    void setSpeed(float speed);
    int getSpeed();
    
    void setTargetDistance(int distance);
    
    void go(int time_target);
};

class CartoSimulatorLine: public CartoSimulatorMotor {
private:
    double x;
    double y;
    
    int _CalcEDistance(Point p1, Point p2);
    
public:
    Point origin, end;
    int length = 0, steps_left = 0;
    float real_length;
    
    CartoSimulatorLine(Point origin, Point end = Point(0,0));
    
    ~CartoSimulatorLine();
    
    void SetTarget(Point p, int steps);
    void Step(int steps);

    int getEdist(Point p);
    int getStepsLeft();
};


class CartoSimulator {
    
private:
    int _CalcEDistance(Point p1, Point p2);
    
public:
    
    int current_x = 0;
    int current_y = 0;
    Mat *canvas;
    Point prev_point;
    
    Point origin;
    CartoSimulatorLine *line1;
    CartoSimulatorLine *line2;
    
    CartoSimulator();
    CartoSimulator(Mat *cvMat);
    
   ~CartoSimulator();
    
    void MoveToPoint(Point p, int steps);
};

#endif /* CartoSimulator_hpp */
