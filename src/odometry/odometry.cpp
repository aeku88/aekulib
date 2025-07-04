#include "api/odometry/odometry.hpp"

void Odometry::x_position(double angle, double arcX){
        double Ts; 

        double x_change = 2 * std::sin(angle / 2) * (((arcX)/angle) + Ts);

    }

void Odometry::y_position(double angle, double arcY){
    double Tr;

        double y_change = 2 * std::sin(angle / 2) * (((arcY)/angle) + Tr);

}


