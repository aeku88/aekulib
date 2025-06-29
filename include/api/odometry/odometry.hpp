#pragma once
#include <cmath>
namespace aekulib 
{
    class Odometry
    {

    void y_position(double angle, double arcY){
        double Tr;

        double y_coord = 2 * std::sin(angle / 2) * (((arcY)/angle) + Tr);

    }

    void x_position(double angle, double arcX){
        double Ts; 

        double x_coord = 2 * std::sin(angle / 2) * (((arcX)/angle) + Ts);

    }


    };
}