//
//  CartoSimulator.cpp
//  carto
//
//  Created by Matthew Thomas on 10/8/17.
//  Copyright © 2017 Matthew Thomas. All rights reserved.
//

#include "CartoSimulator.hpp"
#include "CartoMoveTest.hpp"

using namespace cv;

/************************/
/* CartoSimulator Motor */
/************************/
CartoSimulatorMotor::CartoSimulatorMotor() {
}

CartoSimulatorMotor::~CartoSimulatorMotor() {}

void CartoSimulatorMotor::setSpeed(float speed) {
   // std::cout << "Speed set to: " << speed << std::endl;
    this->speed=speed;
}

int CartoSimulatorMotor::getSpeed(){
    return this->speed;
}

void CartoSimulatorMotor::setTargetDistance(int distance) {
    this->target_distance = distance;
}

void CartoSimulatorMotor::setMotorNumber(int number) {
    this->motor_number = number;
}

void CartoSimulatorMotor::go(int time_target) {
    this->arduino.open("/tmp/arduino.txt", std::ofstream::app);
    
    int s=0;
    
    if(this->speed > 0) {
        s=(int)std::ceil(this->speed);
    }else if(this->speed < 0){
        s=(int)std::floor(this->speed);
    }
    
    s=std::abs(s);
    
    arduino << this->motor_number << "," << s << "," << this->target_distance << std::endl;
    arduino.flush();
    this->arduino.close();
}

/************************/
/* CartoSimulator Line */
/************************/

CartoSimulatorLine::CartoSimulatorLine(Point origin, Point end):CartoSimulatorMotor() {
    this->origin=origin;
    this->end=end;
    
    this->length = this->_CalcEDistance(this->origin, this->end);
    this->real_length=this->length;
}

CartoSimulatorLine::~CartoSimulatorLine() {}

int CartoSimulatorLine::_CalcEDistance(Point p1, Point p2)
{
    double x = (double)p1.x - (double)p2.x; //calculating number to square in next step
    double y = (double)p1.y - (double)p2.y;
    double dist;
        
    dist = pow(x, 2) + pow(y, 2);       //calculating Euclidean distance
    dist = sqrt(dist);
        
    return (int)dist;
}

void CartoSimulatorLine::SetTarget(Point p, int steps){
    // This may not work. Distance between points is not necessarily related to line length
    float line_len = this->getEdist(p);
    float distance = 0;
    
    //std::cout << "Line len for motor " << this->motor_number << ": " << line_len << std::endl;
    
    distance=line_len-this->real_length;
    
    this->real_length=line_len;
    
    int time_target = 5;
    this->steps_left = steps;
    
    // We want the movement to finish at the same time, so the rates for each will need
    // to be distinct. Measurements will be in points-per-second
    // d = r * t or for us r = d / t
    // t = 5 seconds (target time)
    //
    
    float speed = (float)distance/time_target;

    this->setSpeed(speed);
    this->setTargetDistance(distance);
    this->go(time_target);
    
    this->end=p;
    
    //for(int i=0;i<time_target;i++){
        // Each step is a "second"
        
      //  this->length+=this->speed;
    //}
}

int CartoSimulatorLine::getStepsLeft() {
    return this->steps_left;
}

void CartoSimulatorLine::Step(int steps = 1) {
    this->steps_left-=steps;
    
    this->length+=this->getSpeed() * steps;
}

    /* Utility to find what the travel length would be for a point change */
int CartoSimulatorLine::getEdist(Point p) {
    return this->_CalcEDistance(this->origin, p);
}

/************************/
/* CartoSimulator */
/************************/

CartoSimulator::CartoSimulator(Mat *cvMat) {
    int max_x = 125 * 40;
    
    this->canvas=cvMat;
    this->prev_point=Point(12*125, 6*125);
    
    std::ofstream arduino;
    if(!arduino.is_open()) {
        arduino.open("/tmp/arduino.txt", std::ofstream::trunc);
    }
    arduino.close();
    
    this->line1=new CartoSimulatorLine(Point(0,0), this->prev_point);
    this->line1->setMotorNumber(0);
    
    this->line2=new CartoSimulatorLine(Point(max_x,0), this->prev_point);
    this->line2->setMotorNumber(1);
}

CartoSimulator::CartoSimulator() {}
CartoSimulator::~CartoSimulator() {}

void CartoSimulator::MoveToPoint(Point p, int steps=5){
    Point intersection;
    p.x = p.x + (12*125);
    p.y = p.y + (6*125);
    this->line1->SetTarget(p,steps);
    this->line2->SetTarget(p,steps);
    CartoMoveTest test;
    
    for(int i=0;i<steps;i++) {
        this->line1->Step();
        this->line2->Step();
        intersection=test.getIntersection(
                                        this->line1->origin,
                                        this->line1->length,
                                        this->line2->origin,
                                        this->line2->length
                                           );
        //std::cout << "Target\t\t: " << p << std::endl;
        //std::cout << "Intersection\t: " << intersection << std::endl;
        //std::cout << "Prev Intersection\t: " << this->prev_point << std::endl;
        
        line(*this->canvas,this->prev_point,intersection,Scalar(255,255,255),1,8);
        this->prev_point=intersection;
    }
}




