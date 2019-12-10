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
    Side slip
    Velocity

CONTROLS:
    Steering Angle
    Throttle
    Breaks

Linearization for sample time 0.1s

Corresponding MATLAB/SIMULINK Model: mpc_gps_unconstr
*/

void init_ss()
{
    matrix<double, NR_OF_STATES, NR_OF_STATES> A0;

    A0 = 1, 0.00521573236775262, 0.000140339553996367, 0,
    0, 0.227356975419193, 0.0199799069155128, 0,
    0, 0.00517705168705985, 0.597114155981438, 0,
    0, 0, 0, 0.999000499833375;
    ss_A[0] = A0;

    matrix<double, NR_OF_STATES, NR_OF_STATES> A1;
    A1 = 1, 0.00706395910842331, 0.000189238551391244, 0,
    0, 0.476860482790080, 0.0319842854081146, 0,
    0, 0.00666833660091353, 0.772818297929357, 0,
    0, 0, 0, 0.999000499833375;
    ss_A[1] = A1;

    matrix<double, NR_OF_STATES, NR_OF_STATES> A2;
    A2 = 1, 0.00706395910842331, 0.000189238551391244, 0,
    0, 0.476860482790080, 0.0319842854081146, 0,
    0, 0.00666833660091353, 0.772818297929357, 0,
    0, 0, 0, 0.999000499833375;
    ss_A[2] = A2;

    matrix<double, NR_OF_STATES, NR_OF_CONTROLS> B0;
    B0 = 0.00537103664061578, 0, 0,
    0.866418146172481, 0, 0,
    -0.153749086178215, 0, 0,
    0, 0.03, -0.03;
    ss_B[0] = B0;

    matrix<double, NR_OF_STATES, NR_OF_CONTROLS> B1;
    B1 = 0.00659964738959933, 0, 0,
    1.17539112379496, 0, 0,
    -0.0835780184612082, 0, 0,
    0, 0.03, -0.03;
    ss_B[1] = B1;

    matrix<double, NR_OF_STATES, NR_OF_CONTROLS> B2;
    B2 = 0.00659964738959933, 0, 0,
    1.17539112379496, 0, 0,
    -0.0835780184612082, 0, 0,
    0, 0.03, -0.03;
    ss_B[2] = B2;
}