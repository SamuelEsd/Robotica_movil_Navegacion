#include "Vector2d.hpp" 
#include <cmath>
#include "iostream"

double computeAngle(double x, double y){
    double angle = 0;
    if(y == -0){
        y = 0;
    }

    if (x == 0){
        if (y >= 0){
            angle = M_PI/2;
        }
        else{
            angle = M_PI*(3/4);
        }
    }
    else{
        angle = atan(y/x);
    }

    // Second and third quadrant
    if(x < 0 && y >= 0){
        angle += M_PI;
    }
    else if (x < 0 && y < 0){
        angle += M_PI;
    }
    // fourth quadrant
    else if (y < 0){
        angle = 2*M_PI + angle;
    }
    return angle;
}

double computeMagnitude(double x, double y){
    return sqrt(pow(x,2)+pow(y,2));
}

Vector2d::Vector2d(double m, double a){ 
    magnitude = m;
    angle = a;
    this->getComponents();
}

Vector2d::Vector2d(double x, double y, bool b){ 
    this->x = x;
    this->y = y;
    this->angle = computeAngle(x,y);
    this->magnitude = computeMagnitude(x,y);
}

void Vector2d::setMagnitude(double magnitude){
    this->magnitude = magnitude;
    this->getComponents();
}

double Vector2d::getMagnitude(){
    return this->magnitude;
}

double Vector2d::getAngle(){
    return this->angle;
}

Vector2d Vector2d::sum(Vector2d v){
    double x = this->x + v.x;
    double y = this->y + v.y;
    this->magnitude = computeMagnitude(x,y);
    this->angle = computeAngle(x,y);

    // this->getComponents();
    this->x = x;
    this->y = y;
}

void Vector2d::getComponents(){
    double degrees = (angle*180)/M_PI;
    x = cos(this->angle)*magnitude;
    y = sin(this->angle)*magnitude;
}


void Vector2d::print(){
  std::cout << "Angulo: " << this->angle << " Magnitud: " << this->magnitude << std::endl;
  std::cout << "x: " << this->x << " y: " << this->y << std::endl;
}
