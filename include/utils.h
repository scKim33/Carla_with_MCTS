#ifndef UTILS_H_
#define UTILS_H_


// ROS Headers
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


// Headers
#include <math.h>
#include <cmath>
#include <random>
#include <algorithm>
#include <time.h>
#include <vector>
#include <stdlib.h>


// Math Utils
#define DEG2RAD (M_PI / 180)
#define RAD2DEG 1 / DEG2RAD
#define MPS2KMPH 3.6
#define KMPH2MPS 1 / MPS2KMPH
#define DIST(x1, y1, x2, y2) sqrt((x1 - x2)*(x1 - x2) +(y1 - y2)*(y1 - y2))


// Coordinate of the car and goal
struct Coordinate {
    double x; // m
    double y; // m
    double th; // rad

    Coordinate() {} 
    Coordinate(double x_, double y_, double th_) : x(x_), y(y_), th(th_) {} 
    Coordinate operator-(const Coordinate &c) const {
        return Coordinate(x - c.x, y - c.y, th - c.th);
    }
};


// Car state
struct State {
    Coordinate pos;
    double v; // m/s
    double s; // rad

    friend ostream& operator<<(ostream &o, const State &s) {
        o << "x: " << s.pos.x << ", y: " << s.pos.y << ", th: " << s.pos.th * RAD2DEG << ", v: " << s.v << ", s: " << s.s * RAD2DEG;
        return o;
    }
};


double rational_function(double x);
/**
 * @param x is the norm of distance or angle
 * @return the rational fucntion value
*/


tuple<double, double, double> localization(Coordinate car, Coordinate goal);
/**
 * @brief calculate the local coordinate
 * @return coordinates of the car when the origin is goal pose
*/


#endif