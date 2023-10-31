#include "../include/utils.h"


using namespace std;


double rational_function(double x) {
    return 1 / (1.0 + x);
}


tuple<double, double, double> localization(Coordinate car, Coordinate goal) {
    double tx = car.x - goal.x;
    double ty = car.y - goal.y;
    double x = tx * cos(goal.th) + ty * sin(goal.th);
    double y = -tx * sin(goal.th) + ty * cos(goal.th);
    double th = car.th - goal.th;
    return {x, y, th};
}

