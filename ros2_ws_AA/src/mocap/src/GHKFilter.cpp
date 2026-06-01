/*
 * GHKFilter.cpp
 *
 *  Created on: Sep 23, 2013
 *      Author: mark
 */

#include "mocap/GHKFilter.h"

GHKFilter::GHKFilter()
{
    // Set g,h,k gains based on theta
    // Comes from page 52 of "TRACKING AND KALMAN FILTERING MADE EASY" by
    // Eli Brookner
    g_gain = 1 - POSITION_THETA * POSITION_THETA * POSITION_THETA;
    h_gain = 1.5 * (1 - POSITION_THETA * POSITION_THETA) * (1 - POSITION_THETA);
    k_gain = 0.5 * (1 - POSITION_THETA) * (1 - POSITION_THETA)
            * (1 - POSITION_THETA);
}

void GHKFilter::reset()
{
    initialized_ = false;
    this->vel.x = 0.0;
    this->vel.y = 0.0;
    this->vel.z = 0.0;
    this->acc.x = 0.0;
    this->acc.y = 0.0;
    this->acc.z = 0.0;
}

void GHKFilter::propUpdate(double dt)
{
    if (not initialized_){
        return;
    }
    this->propUpdate_single(dt,this->pos.x,this->vel.x,this->acc.x);
    this->propUpdate_single(dt,this->pos.y,this->vel.y,this->acc.y);
    this->propUpdate_single(dt,this->pos.z,this->vel.z,this->acc.z);
}

void GHKFilter::measUpdate(double dt, geometry_msgs::msg::Point measure_pos)
{
    if (not initialized_){  
        this->pos = measure_pos;
        initialized_ = true;
    }
    else{
        this->measUpdate_single(dt, measure_pos.x, this->pos.x, this->vel.x, this->acc.x);
        this->measUpdate_single(dt, measure_pos.y, this->pos.y, this->vel.y, this->acc.y);
        this->measUpdate_single(dt, measure_pos.z, this->pos.z, this->vel.z, this->acc.z);
    }
}

void GHKFilter::propUpdate_single(double dt, double &xk, double &vk, double &ak)
{

    // Propogation step
    double xkp = xk + dt * vk + 1. / 2. * dt * dt * ak; // update estimated position
    double vkp = vk + dt * ak; // update estimated velocity
    double akp = ak; // update estimated acceleration

    // make assignments
    xk = xkp;
    vk = vkp;
    ak = akp;
}

void GHKFilter::measUpdate_single(double dt, double xm, double &xk, double &vk,
        double &ak)
{

    // Measurement Update
    double rk = xm - xk; // residual error (innovation signal)
    xk = xk + g_gain * rk; // update pos,vel,acc given residual error
    vk = vk + h_gain / dt * rk;
    ak = ak + 2 * k_gain / (dt * dt) * rk;
}
