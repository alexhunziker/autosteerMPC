#include <dlib/control.h>
#include <math.h>

using namespace dlib;

const int NR_OF_STATES = 5;
const int NR_OF_CONTROLS = 3;

matrix<double, NR_OF_STATES, NR_OF_STATES> ss_A[3];
matrix<double, NR_OF_STATES, NR_OF_CONTROLS> ss_B[3];

/* 
STATE: 
    Yaw angle
    Yaw rate
    Side slip
    Velocity
    Time to Collision

CONTROLS:
    Steering Angle
    Throttle
    Breaks

Linearization for sample time 0.1s

Corresponding MATLAB/SIMULINK Model: mpc_gps_obst
*/

void init_ss()
{
    matrix<double, NR_OF_STATES, NR_OF_STATES> A0;

    A0 = 1, 0.00521573236775262, 0.000140339553996367, 0, 0,
    0, 0.227356975419193, 0.0199799069155128, 0, 0,
    0, 0.00517705168705985, 0.597114155981437, 0, 0,
    0, 0, 0, 0.999000499833375, 0,
    0, 0, 0, -0.00499750083312504, 1;
    ss_A[0] = A0;

    matrix<double, NR_OF_STATES, NR_OF_STATES> A1;
    A1 = 1, 0.00521573236775262, 0.000140339553996367, 0, 0,
    0, 0.227356975419193, 0.0199799069155128, 0, 0,
    0, 0.00517705168705985, 0.597114155981437, 0, 0,
    0, 0, 0, 0.999000499833375, 0,
    0, 0, 0, -0.00499750083312504, 1;
    ss_A[1] = A1;

    matrix<double, NR_OF_STATES, NR_OF_STATES> A2;
    A2 = 1, 0.00521573236775262, 0.000140339553996367, 0, 0,
    0, 0.227356975419193, 0.0199799069155128, 0, 0,
    0, 0.00517705168705985, 0.597114155981437, 0, 0,
    0, 0, 0, 0.999000499833375, 0,
    0, 0, 0, -0.00499750083312504, 1;
    ss_A[2] = A2;

    matrix<double, NR_OF_STATES, NR_OF_CONTROLS> B0;
    B0 = 0.00537103664061578, 0, 0,
    0.866418146172481, 0, 0,
    -0.153749086178215, 0, 0,
    0, 0.03, -0.03,
    0, -2.49916687495834e-05, 4.99833374991668e-05;
    ss_B[0] = B0;

    matrix<double, NR_OF_STATES, NR_OF_CONTROLS> B1;
    B1 = 0.00537103664061578, 0, 0,
    0.866418146172481, 0, 0,
    -0.153749086178215, 0, 0,
    0, 0.03, -0.03,
    0, -2.49916687495834e-05, 4.99833374991668e-05;
    ss_B[1] = B1;

    matrix<double, NR_OF_STATES, NR_OF_CONTROLS> B2;
    B2 = 0.00537103664061578, 0, 0,
    0.866418146172481, 0, 0,
    -0.153749086178215, 0, 0,
    0, 0.03, -0.03,
    0, -2.49916687495834e-05, 4.99833374991668e-05;
    ss_B[2] = B2;
}