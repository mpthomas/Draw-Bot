//
//  CartoSimulator.cpp
//  carto
//
//  Created by Matthew Thomas on 10/8/17.
//  Copyright © 2017 Matthew Thomas. All rights reserved.
//

#include "CartoSimulator.hpp"
#include "CartoMoveTest.hpp"

#define HEAD_WIDTH_MM   155
#define STEPS_PER_MM    5.8

using namespace cv;

/************************/
/* CartoSimulator Line */
/************************/

CartoSimulatorLine::CartoSimulatorLine(Point origin, Point end) {
    float a,b,c;
    this->origin=origin;
    this->end=end;
    
    //this->length = this->_CalcEDistance(this->origin, this->end);
    this->real_length=this->length;
    
    if(this->origin.x == 0) {
        a=end.x;
    }else{
        a=abs(this->origin.x - end.x);
    }
    
    b = end.y;
    c = (a * a) + (b * b);
    c=sqrt(c);
    
   this->real_length = (int)c;
    this->length = (int)c;
    
    /* +++ Temporary calculation of steps taking the head into account +++ */
    /*
    if(this->origin.x == 0) {
        a=end.x - ( (HEAD_WIDTH_MM/2) * STEPS_PER_MM );
    }else{
        a=abs(this->origin.x - (end.x + ( (HEAD_WIDTH_MM/2) * STEPS_PER_MM)));
    }
    
    b=end.y;
    c = (a * a) + (b * b);
    this->real_length = (int)sqrt(c);
     */
}

CartoSimulatorLine::~CartoSimulatorLine() {}

float CartoSimulatorLine::_CalcEDistance(Point p1, Point p2)
{
    float x = (float)p1.x - (float)p2.x; //calculating number to square in next step
    float y = (float)p1.y - (float)p2.y;
    float dist;
        
    dist = pow(x, 2) + pow(y, 2);       //calculating Euclidean distance
    dist = sqrt(dist);
        
    //return (int)dist;
    return dist;
}

void CartoSimulatorLine::SetTarget(Point p, int steps){
    // This may not work. Distance between points is not necessarily related to line length
    float line_len = this->getEdist(p);
    
    //std::cout << "Line len for motor " << this->motor_number << ": " << line_len << std::endl;
    
    this->target_distance=line_len-this->real_length;
    this->real_length=line_len;
    this->length=line_len;
    
    //this->arduino.open("/Users/matt/xcode/arduino.txt", std::ofstream::app);
    int s=0;
    
    *this->arduino << this->motor_number << "," << s << "," << std::setprecision(3) << this->target_distance << std::endl;
    this->arduino->flush();
    //this->arduino.close();
    //this->go(0);
    
    this->end=p;
}

void CartoSimulatorLine::goFromTo(CvPoint2D32f from, Point to){
    // This may not work. Distance between points is not necessarily related to line length
    float a, b, c;
    
    // Figure out the vertices length given x, y
    if(this->origin.x == 0) {
        a=to.x;
    }else{
        a=abs(this->origin.x - to.x);
    }
    
    b=to.y;
    c = (a * a) + (b * b);
    c=(int)sqrt(c);
    
    // rounding issue so do nothing
    if(c == this->length) {
        this->target_distance = 0;
    }else{
        this->target_distance = c - this->real_length;
        this->real_length = c;
        this->length = c;
    }
    
    //std::cout << "Line len for motor " << this->motor_number << ": " << line_len << std::endl;
    //this->target_distance=line_len-this->real_length;
    //this->real_length=line_len;
    //this->length=line_len;
    
    //this->arduino.open("/Users/matt/xcode/arduino.txt", std::ofstream::app);
    int s=0;
    
    /* +++ Temporary calculation of steps taking the head into account +++ */
    /*
    if(this->origin.x == 0) {
        a=to.x - ( (HEAD_WIDTH_MM/2) * STEPS_PER_MM );
    }else{
        a=abs(this->origin.x - (to.x + ( (HEAD_WIDTH_MM/2) * STEPS_PER_MM)));
    }
    
    b=to.y;
    c = (a * a) + (b * b);
    c = sqrt(c);
    if(c == this->real_length) {
        this->target_distance = 0;
    }else{
        this->target_distance = c - this->real_length;
        this->real_length = c;
    }
    */
    
    *this->arduino << this->motor_number << "," << s << "," << std::setprecision(4) << this->target_distance << "," << this->color[0] << std::endl;
    this->arduino->flush();
    //this->arduino.close();
    //this->go(0);
    
    this->end=to;
}

void CartoSimulatorLine::setMotorNumber(int number){
    this->motor_number = number;
}

/* Utility to find what the travel length would be for a point change */
float CartoSimulatorLine::getEdist(Point p) {
    return this->_CalcEDistance(this->origin, p);
}

/************************/
/* CartoSimulator */
/************************/

CartoSimulator::CartoSimulator(Mat *cvMat) {
    int max_x = 980 * STEPS_PER_MM;  // 980mm / 5 steps per mm
    this->draw_line=true;
    //int max_x=cvMat->cols;
    
    this->canvas=cvMat;
    //this->prev_point=this->findPoint(440*5, 736*5, 980*5);
    //this->prev_point=this->findPoint(320*STEPS_PER_MM, 700*STEPS_PER_MM, 980*STEPS_PER_MM);
    this->prev_point=this->findPoint(465*STEPS_PER_MM, 767*STEPS_PER_MM, 980*STEPS_PER_MM);
    //this->prev_point=Point(0,0);

    this->arduino = new std::ofstream;
    this->arduino->open("/Users/matt/xcode/arduino.txt", std::ofstream::trunc);
    
    this->line1=new CartoSimulatorLine(Point(0,0), this->prev_point);
    this->line1->setMotorNumber(0);
    this->line1->arduino = this->arduino;
    
    this->line2=new CartoSimulatorLine(Point(max_x,0), this->prev_point);
    this->line2->setMotorNumber(1);
    this->line2->arduino=this->arduino;
    
    std::cout << "CartoSimulator(): Constructor prev_point: " << Point(this->prev_point) << std::endl;
    std::cout << "CartoSimulator(): Line 1 len: " << this->line1->length << std::endl;
    std::cout << "CartoSimulator(): Line 2 len: " << this->line2->length << std::endl;
    Point p = (Point)this->findPoint(this->line1->length,this->line2->length, this->line2->origin.x);
    std::cout << "CartoSimulator(): findPoint(line1,line2): " << p << std::endl;

    //std::cout << this->prev_point.x << " " << this->prev_point.y;
    //std::cout << " Len1: " << this->line1->length << " Len2: " << this->line2->length << " START" << std::endl;
}

CartoSimulator::CartoSimulator() {}
CartoSimulator::~CartoSimulator() {}

CvPoint2D32f CartoSimulator::findPoint(int side_a, int side_b, int side_c) // left, right, top
{
    double angle_a, angle_b, angle_c;       //radians
    double degrees_a, degrees_b, degrees_c;
    double numerator, denominator;

    numerator = (side_b * side_b) + (side_c * side_c) - (side_a * side_a);
    denominator = 2.0 * side_b * side_c;
    angle_a = acos(numerator / denominator);
    
    numerator = (side_a * side_a) + (side_c * side_c) - (side_b * side_b);
    denominator = 2.0 * side_a * side_c;
    angle_b = acos(numerator / denominator);
    
    numerator = (side_a * side_a) + (side_b * side_b) - (side_c * side_c);
    denominator = 2.0 * side_a * side_b;
    angle_c = acos(numerator / denominator);
    
    // Convert to degrees
    degrees_a = angle_a * 180 / M_PI;
    degrees_b = angle_b * 180 / M_PI;
    degrees_c = angle_c * 180 / M_PI;
    
    //std::cout << "DegreeA: " << degrees_a << " DegreeB: " << degrees_b << " DegreeC: " << degrees_c << std::endl;
    
    // Figure out the x,y (adjacent/opposite) lengths
    double l1_adj_degrees = 90 - degrees_b;
    double l1_adj_angle = l1_adj_degrees * (M_PI /180);

    // side_a / sin(90) = x / sin(l1_adj_degrees)
    double x = sin(l1_adj_angle) * (side_a / sin(90));
    
    double c_adj_degrees = 180 - l1_adj_degrees - 90;
    double c_adj_angle = c_adj_degrees * (M_PI / 180);
    
    // side_a / sin(90) = y / sin(c_adj_degrees)
    double y = sin(c_adj_angle) * (side_a / sin(90));
    
    //return CvPoint2D32f(x-(x*.109),y-(y*.109));
    return CvPoint2D32f(x,y);
}

void CartoSimulator::MoveToPoint(Point p, int steps=5){
    Point intersection;
    this->tick_meter.start();
    // M was slanted. Trying without this.
    //p.x = p.x + (11.5*50); //12.5
    //p.y = p.y + (5.25*50);  //27.75
    ////this->line1->SetTarget(p,steps);
    ////this->line2->SetTarget(p,steps);
    
    ////p.x=(p.x + 220) * 5;
    ////p.y=(p.y + 131) * 5;
    
    this->line1->color=this->canvas->at<uchar>(p);
    this->line2->color=this->line1->color;
    
    this->line1->goFromTo(this->prev_point,p);
    this->line2->goFromTo(this->prev_point,p);
    this->tick_meter.stop();
    
    CartoMoveTest test;

    this->prev_point=this->findPoint(this->line1->length, this->line2->length, this->line2->origin.x);
    //circle(*this->canvas,this->prev_point, 2, Scalar(0,0,0),1,8);
}

int CartoSimulator::distance(Point p1, Point p2) {
    double x = (double)p1.x - (double)p2.x; //calculating number to square in next step
    double y = (double)p1.y - (double)p2.y;
    double dist;
    
    dist = pow(x, 2) + pow(y, 2);       //calculating Euclidean distance
    dist = sqrt(dist);
    
    return (int)dist;
}




