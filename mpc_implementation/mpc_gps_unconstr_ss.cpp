#include <dlib/control.h>
#include <math.h>

using namespace dlib;

const int NR_OF_STATES = 4;
const int NR_OF_CONTROLS = 3;

matrix<double, NR_OF_STATES, NR_OF_STATES> ss_A[3];
matrix<double, NR_OF_STATES, NR_OF_CONTROLS> ss_B[3];

/* 
STATE: 
    Yaw angle
    Yaw rate
    Schwimm Angle
    Velocity

CONTROLS:
    Steering Angle
    Acceleration
    Breaks

Linearization for sample time 0.1s

Corresponding MATLAB/SIMULINK Model: mpc_gps_obst
*/

void init_ss()
{
    matrix<double, NR_OF_STATES, NR_OF_STATES> A0;

    A0 = 1, 0.0701198468049256, 0.00779874007178866, 0,
    0, 0.475743473586823, 0.0615641544802408, 0,
    0, 0.0564338082735540, 0.00734286491632463, 0,
    0, 0, 0, 0.990049833749168;
    ss_A[0] = A0;

    matrix<double, NR_OF_STATES, NR_OF_STATES> A1; // TODO: Fake
    A1 = 1, 0.0701198468049256, 0.00779874007178866, 0,
    0, 0.475743473586823, 0.0615641544802408, 0,
    0, 0.0564338082735540, 0.00734286491632463, 0,
    0, 0, 0, 0.990049833749168;
    ss_A[1] = A1;

    matrix<double, NR_OF_STATES, NR_OF_STATES> A2; // Fake
    A2 = 1, 0.0701198468049256, 0.00779874007178866, 0,
    0, 0.475743473586823, 0.0615641544802408, 0,
    0, 0.0564338082735540, 0.00734286491632463, 0,
    0, 0, 0, 0.990049833749168;
    ss_A[2] = A2;

    matrix<double, NR_OF_STATES, NR_OF_CONTROLS> B0;
    B0 = 0.0628877679773124, 0, 0,
    1.08257977931400, 0, 0,
    -0.389095891554744, 0, 0,
    0, 0.0995016625083195, -0.199003325016639;
    ss_B[0] = B0;

    matrix<double, NR_OF_STATES, NR_OF_CONTROLS> B1; //Fake
    B1 = 0.0628877679773124, 0, 0,
    1.08257977931400, 0, 0,
    -0.389095891554744, 0, 0,
    0, 0.0995016625083195, -0.199003325016639;
    ss_B[1] = B1;

    matrix<double, NR_OF_STATES, NR_OF_CONTROLS> B2; // Fake
    B2 = 0.0628877679773124, 0, 0,
    1.08257977931400, 0, 0,
    -0.389095891554744, 0, 0,
    0, 0.0995016625083195, -0.199003325016639;
    ss_B[2] = B2;
}