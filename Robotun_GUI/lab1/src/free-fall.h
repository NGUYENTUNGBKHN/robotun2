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
    double M;     // Wheel mass (kg)
    double R;     // Wheel radius (m)
    double m;     // Pendulum mass (kg)
    double l;     // Distance from wheel center to pendulum Center of Mass (m)
    double g;     // Gravity acceleration (m/s^2)
    double cw;    // Wheel damping/friction coefficient
    double cp;    // Pendulum joint damping/friction coefficient
};


#endif // FREE_FALL_H

