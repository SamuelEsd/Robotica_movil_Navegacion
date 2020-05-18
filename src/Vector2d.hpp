//Markers.h

#ifndef __VECTOR2D_HPP_INCLUDED__   
#define __VECTOR2D_HPP_INCLUDED__   

#include <visualization_msgs/Marker.h>

class Vector2d{
     // private data member 
    private:
        double magnitude;
        double angle;      // In radians
        double x;
        double y;

        void getComponents();
       
    // public member function     
    public:     
        Vector2d(double m, double a);
        Vector2d(double x, double y, bool b);
        Vector2d sum(Vector2d v);
        double getMagnitude();
        void setMagnitude(double magnitude);
        double getAngle();
        void print();
};


#endif 