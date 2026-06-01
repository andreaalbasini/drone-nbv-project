/*
 * GHKFilter.h
 * Implements a very simple constant acceleration and constant rate ghk filter.
 * From: "TRACKING AND KALMAN FILTERING MADE EASY" by Eli Brookner -- pages 51-55
 *
 *  Created on: Sep 23, 2013
 *      Author: mark
 */

#ifndef GHKFILTER_H_
#define GHKFILTER_H_

#include "mocap/defines.h"
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point.hpp>

class GHKFilter {
public:
    GHKFilter();
    void propUpdate(double dt);
    void measUpdate(double dt, geometry_msgs::msg::Point measure_pos);
    geometry_msgs::msg::Point getPos() {return pos;}
    geometry_msgs::msg::Vector3 getVel() {return vel;}
    geometry_msgs::msg::Vector3 getAcc() {return acc;}
    void reset();
private:
    bool initialized_;
    geometry_msgs::msg::Point pos;
    geometry_msgs::msg::Vector3 vel;
    geometry_msgs::msg::Vector3 acc;

    void propUpdate_single(double dt, double &xk,
        double &vk, double &ak);
    void measUpdate_single(double dt, double xm, double &xk,
        double &vk, double &ak);

    double g_gain, h_gain, k_gain;

};

#endif /* GHKFILTER_H_ */
