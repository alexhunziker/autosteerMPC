#include <iostream>
#include <fstream>
#include "mpc_controller.cpp"
#include <dlib/string.h>
#include <math.h>

using namespace std;
using namespace dlib;

void simulate_wrapper(double state[NR_OF_STATES], double controls[NR_OF_CONTROLS], double speed, double yaw_rate);

matrix<double, NR_OF_STATES, 1> simulate_next_step(
    matrix<double, NR_OF_STATES, 1> current_state, matrix<double, NR_OF_CONTROLS, 1> controls,
    double speed, double yaw_rate
);

int main (int argc, char** argv)
{
    initialize_mpc_objects();
    double controls[NR_OF_CONTROLS];
    double observables[NR_OF_STATES] = {0, 0, 0, 0, 0, 2};

    for(int i=1; i<100; i++) {
        double bu[NR_OF_STATES];
        for(int i=0; i<NR_OF_STATES; i++){
            bu[i] = observables[i];
        }
        predict(10, 10, controls[0], controls[1], controls[5], observables[0], observables[1], observables[2],
                    controls, bu);

        simulate_wrapper(observables, controls, observables[5], observables[3]);
    }

    return 0;
}

void simulate_wrapper(double state[NR_OF_STATES], double controls[NR_OF_CONTROLS], double speed, double yaw_rate) {
    matrix<double, NR_OF_STATES, 1> current_state;
    for(int i=0; i<NR_OF_STATES; i++) {
        current_state(i,0) = state[i];
    }

    matrix<double, NR_OF_CONTROLS, 1> current_controls;
    for(int i=0; i<NR_OF_CONTROLS; i++) {
        current_controls(i,0) = controls[i];
    }

    matrix<double, NR_OF_STATES, 1> predicted;
    predicted = simulate_next_step(current_state, current_controls, speed, yaw_rate);

    cout << "INFO: Predicted state: ";
    for(int i=0; i<NR_OF_STATES; i++) {
        state[i] = predicted(i,0);
        cout << state[i] << ", ";
    }
    cout << "\n";
}

// TODO: Smaller step size for this
// Prediction of next state can include non-linearized formulas
matrix<double, NR_OF_STATES, 1> simulate_next_step(
    matrix<double, NR_OF_STATES, 1> current_state, matrix<double, NR_OF_CONTROLS, 1> controls,
    double speed, double yaw_rate
) {
    double yaw_angle = current_state(3,0) + yaw_rate;   // TODO: this is not good, what if we are off???

    if(speed<0.5) {
        speed = 0.5;
    }

    for(int i=0; i<(1.0/STEP_SIZE); i++) {
        matrix<double, NR_OF_STATES, NR_OF_STATES> A;
        A = 1, 0, 0, 0, 0, cos(yaw_angle)*STEP_SIZE*STEP_SIZE,
            0, 1, 0, 0, 0, sin(yaw_angle)*STEP_SIZE*STEP_SIZE,
            0, 0, 1, 1*STEP_SIZE*STEP_SIZE, 0, 0,
            0, 0, 0, 1-(2*cf*lf*lf+2*cr*lr*lr)/iz/speed*STEP_SIZE*STEP_SIZE, -(2*cf*lf-2*cr*lr)/iz/speed*STEP_SIZE*STEP_SIZE, 0,
            0, 0, 0,  -speed-(2*cf*lf-2*cr*lr)/m/speed*STEP_SIZE*STEP_SIZE, 1-(2*cf+2*cr)/m/speed*STEP_SIZE*STEP_SIZE, 0,
            0, 0, 0, 0, 0, 1-0.1*STEP_SIZE*STEP_SIZE;

        matrix<double, NR_OF_STATES, NR_OF_CONTROLS> B;
        B = 0, 0, 0,
            0, 0, 0,
            0, 0, 0,
            2*cf*lf/iz*STEP_SIZE*STEP_SIZE, 0, 0,
            2*cf/m*STEP_SIZE*STEP_SIZE, 0, 0,
            0, 1*STEP_SIZE*STEP_SIZE, -2*STEP_SIZE*STEP_SIZE;   // TODO: Something more elaborate here.

        matrix<double, NR_OF_STATES, 1> bu = current_state;
        current_state = A*current_state + B*controls;
        //cout << "\n" << current_state-bu;
    }
    return current_state;
}

