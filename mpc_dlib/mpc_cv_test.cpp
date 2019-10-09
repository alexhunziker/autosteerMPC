#include <iostream>
#include <fstream>
#include "mpc_follow_lane.cpp"
#include <dlib/string.h>
#include <math.h>

using namespace std;
using namespace dlib;

void simulate_wrapper(double state[NR_OF_STATES], double controls[NR_OF_CONTROLS], double speed, double yaw_rate);

matrix<double, NR_OF_STATES, 1> simulate_next_step(
    matrix<double, NR_OF_STATES, 1> current_state, matrix<double, NR_OF_CONTROLS, 1> controls,
    double speed, double yaw_rate);

int main(int argc, char **argv)
{
    initialize_mpc_objects();
    double controls[NR_OF_CONTROLS];
    double observables[NR_OF_STATES] = {0, 0, 0, 0, 0, 2, 0};

    for (int i = 1; i < 100; i++)
    {
        double bu[NR_OF_STATES];
        for (int i = 0; i < NR_OF_STATES; i++)
        {
            bu[i] = observables[i];
        }
        predict(15, 10, controls[0], controls[1], controls[2], observables[0], observables[1], observables[5], observables[2], observables[3], observables[4], observables[6],
                controls, bu);

        simulate_wrapper(observables, controls, observables[5], observables[3]);
        sleep(100);
    }

    return 0;
}

void simulate_wrapper(double state[NR_OF_STATES], double controls[NR_OF_CONTROLS], double speed, double yaw_rate)
{
    matrix<double, NR_OF_STATES, 1> current_state;
    for (int i = 0; i < NR_OF_STATES; i++)
    {
        current_state(i, 0) = state[i];
    }

    matrix<double, NR_OF_CONTROLS, 1> current_controls;
    for (int i = 0; i < NR_OF_CONTROLS; i++)
    {
        current_controls(i, 0) = controls[i];
    }

    matrix<double, NR_OF_STATES, 1> predicted;
    predicted = simulate_next_step(current_state, current_controls, speed, yaw_rate);

    cout << "INFO: Calculated state (by plant     ): ";
    for (int i = 0; i < NR_OF_STATES; i++)
    {
        state[i] = predicted(i, 0);
        cout << state[i] << ", ";
    }
    cout << "\n";
}

// TODO: Smaller step size for this
// Prediction of next state can include non-linearized formulas
matrix<double, NR_OF_STATES, 1> simulate_next_step(
    matrix<double, NR_OF_STATES, 1> current_state, matrix<double, NR_OF_CONTROLS, 1> controls,
    double speed, double yaw_rate)
{
    double yaw_angle = current_state(2, 0) + yaw_rate; // TODO: this is not good, what if we are off???
    cout << "YAW ANGLE: (sin) " << yaw_angle << " " << sin(yaw_angle) << "\n";

    if (speed < 0.5)
    {
        speed = 0.5;
    }

    for (int i = 0; i < (1.0 / STEP_SIZE); i++)
    {
        //matrix<double, NR_OF_STATES, NR_OF_STATES> A;
        //A = 1, 0, 0, 0, 0, cos(yaw_angle) * STEP_SIZE * STEP_SIZE, 0,
        //0, 1, 0, 0, 0, sin(yaw_angle) * STEP_SIZE * STEP_SIZE, 0,
        //0, 0, 1, 1 * STEP_SIZE * STEP_SIZE, 0, 0, 0,
        //0, 0, 0, 1 - (2 * cf * lf * lf + 2 * cr * lr * lr) / iz / speed * STEP_SIZE * STEP_SIZE, -(2 * cf * lf - 2 * cr * lr) / iz / speed * STEP_SIZE * STEP_SIZE, 0, 0,
        //0, 0, 0, -speed - (2 * cf * lf - 2 * cr * lr) / m / speed * STEP_SIZE * STEP_SIZE, 1 - (2 * cf + 2 * cr) / m / speed * STEP_SIZE * STEP_SIZE, 0, 0,
        //0, 0, 0, 0, 0, 1 - 0.1 * STEP_SIZE * STEP_SIZE, 0,
        //0, 0, 0, 0, 0, 0, 0;

        //matrix<double, NR_OF_STATES, NR_OF_CONTROLS> B;
        //B = -0.0832670692070945, 0.00498337491680536, -0.00996674983361072,

        //matrix<double, NR_OF_STATES, 1> bu = current_state;
        //current_state = A * current_state + B * controls;
        //cout << "\n" << current_state-bu;
    }
    matrix<double, NR_OF_STATES, NR_OF_STATES> A;
    A = 1, 0, 0, 0, 0, cos(yaw_angle) * STEP_SIZE, 0,
    0, 1, 0, 0, 0, sin(yaw_angle) * STEP_SIZE, 0,
    0, 0, 1, 0.0517384006163168, -0.00303079475590676, 0, 0,
    0, 0, 0, 0.227045209024172, -0.0147018562144484, 0, 0,
    0, 0, 0, -0.0514564967505693, 0.00333196362764945, 0, 0,
    0, 0, 0, 0, 0, 0.990049833749168, 0,
    0, 0, 0, 0, 0, -0.0995016625083194, 1;

    matrix<double, NR_OF_STATES, NR_OF_CONTROLS> B;
    B = -0.0832670692070945, 0.00498337491680536, -0.00996674983361072,
    0.0832670692070945, 0, 0,
    0.0990492070239366, 0, 0,
    1.56685387470395, 0, 0,
    0.678434587002140, 0, 0,
    0, 0.0995016625083194, -0.199003325016639,
    0, -0.00498337491680536, 0.00996674983361071;

    matrix<double, NR_OF_STATES, 1> bu = current_state;
    current_state = A * current_state + B * controls;

    if (current_state(2, 0) > PI)
    {
        current_state(2, 0) = current_state(2, 0) - 2 * PI;
    }
    if (current_state(2, 0) < -PI)
    {
        current_state(2, 0) = current_state(2, 0) + 2 * PI;
    }

    return current_state;
}
