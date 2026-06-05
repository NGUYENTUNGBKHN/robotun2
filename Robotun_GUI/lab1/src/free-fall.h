/******************************************************************************/
/*! @addtogroup Group2
    @file       free-fall.h
    @brief      
    @date       2026/06/02
    @author     Development Dept at Tokyo (nguyen-thanh-tung@jcm-hq.co.jp)
    @par        Revision
    $Id$
    @par        Copyright (C)
    Japan CashMachine Co, Limited. All rights reserved.
******************************************************************************/

#ifndef FREE_FALL_H
#define FREE_FALL_H

#define PI 3.14159265358979323846

struct FreeFallState {
    double dth;   // Angular velocity of the pendulum (rad/s)
    double th;    // Pendulum angle (rad, 0 is the vertical upright position)
    double dphi;  // Angular velocity of the rolling wheel (rad/s)
    double phi;   // Wheel rotation angle (rad)
};

struct FreeFallParams {
    double M = 1.5;     // Wheel mass (kg)
    double R = 0.3;     // Wheel radius (m)
    double m = 0.5;     // Pendulum mass (kg)
    double l = 0.8;     // Distance from wheel center to pendulum COM (m)
    double g = 9.81;    // Gravity (m/s^2)
    double cw = 0.1;    // Wheel damping coefficient
    double cp = 0.05;   // Pendulum joint damping coefficient
};


#endif // FREE_FALL_H

