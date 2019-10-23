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

Corresponding MATLAB/SIMULINK Model: mpc_gps_unconstr
*/

void init_ss()
{
    matrix<double, NR_OF_STATES, NR_OF_STATES> A0;

    A0 = 1, 0.000104778166629036, 0.000155226913524498, 0,
    0, 3.58147397036724e-131, 7.90895075243695e-130, 0,
    0, 5.35532003199118e-131, 1.18261259880703e-129, 0,
    0, 0, 0, 0.990049833749167;
    ss_A[0] = A0;

    matrix<double, NR_OF_STATES, NR_OF_STATES> A1; // TODO: Fake
    A1 = 1, 0.000157197751595795, 0.000349328336879545, 0,
    0, 3.55223671157049e-88, 1.17220272153963e-86, 0,
    0, 3.54147560806869e-88, 1.16865166460401e-86, 0,
    0, 0, 0, 0.990049833749167;
    ss_A[1] = A1;

    matrix<double, NR_OF_STATES, NR_OF_STATES> A2; // Fake
    A2 = 1, 0.000209653965071791, 0.000621196933546046, 0,
    0, 1.14063482569651e-66, 4.99218588886101e-65, 0,
    0, 8.53021934523809e-67, 3.73339825199388e-65, 0,
    0, 0, 0, 0.990049833749167;
    ss_A[2] = A2;

    matrix<double, NR_OF_STATES, NR_OF_CONTROLS> B0;
    B0 = 0.111072229419280, 0, 0,
    1.11128358545947, 0, 0,
    -0.388522380898623, 0, 0,
    0, 0.0995016625083194, -0.199003325016639;
    ss_B[0] = B0;

    matrix<double, NR_OF_STATES, NR_OF_CONTROLS> B1;
    B1 = 0.166598362062492, 0, 0,
    1.66724888056147, 0, 0,
    -0.388064085871257, 0, 0,
    0, 0.0995016625083194, -0.199003325016639;
    ss_B[1] = B1;

    matrix<double, NR_OF_STATES, NR_OF_CONTROLS> B2; // Fake
    B2 = 0.222134744337269, 0, 0,
    2.22360265985233, 0, 0,
    -0.387422173906905, 0, 0,
    0, 0.0995016625083194, -0.199003325016639;
    ss_B[2] = B2;
}