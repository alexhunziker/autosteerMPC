#include <dlib/control.h>
#include <math.h>
#include <dlib/string.h>

#include "mpc_gps_obst_ss.cpp"

using namespace std;
using namespace dlib;

#define PI 3.14159265

const bool DEBUG = true;

// Constants for Model
const int CONTROL_HORIZON = 40;
const float STEP_SIZE = 0.1;
// Contrary to MATLAB, dlib does not allow different control/prediction horizons
const int PREDICTION_HORIZON = CONTROL_HORIZON;

const float CRUISING_SPEED = 4.0;
const float TARGET_TIME_TO_COLLISION = 2.5;

mpc<NR_OF_STATES, NR_OF_CONTROLS, PREDICTION_HORIZON> mpc_objects[3];

mpc<NR_OF_STATES, NR_OF_CONTROLS, PREDICTION_HORIZON> initialize_mpc_object(int i);
mpc<NR_OF_STATES, NR_OF_CONTROLS, PREDICTION_HORIZON> get_best_mpc_object(int speed);

extern "C"
{
    void initialize_mpc_objects()
    {
        init_ss();
        for (int i = 0; i < 3; i++)
        {
            mpc<NR_OF_STATES, NR_OF_CONTROLS, PREDICTION_HORIZON> mpc_object = initialize_mpc_object(i);
            mpc_objects[i] = mpc_object;
        }
        if (DEBUG)
            printf("DEBUG: MPC Objects set up for GPS Obstacle.\n");
    }

    void predict(
        double speed_target,
        double x, double y, double speed,
        double yaw, double yaw_rate, double schwimm_angle,
        double yaw_target, double collision_time, double *controls, double *predicted_state)
    {

        matrix<double, NR_OF_STATES, 1> current_state;
        current_state = yaw, yaw_rate, schwimm_angle, speed, collision_time;

        mpc<NR_OF_STATES, NR_OF_CONTROLS, PREDICTION_HORIZON> mpc_controller = get_best_mpc_object(speed);

        matrix<double, NR_OF_STATES, 1> target_state;
        target_state = yaw_target, 0, 0, speed_target, TARGET_TIME_TO_COLLISION;
        if (DEBUG)
        {
            cout << "DEBUG: current state is:" << trans(current_state) << "\n";
            cout << "DEBUG: target state is: " << trans(target_state) << "\n";
        }
        mpc_controller.set_target(target_state);

        matrix<double, NR_OF_CONTROLS, 1> action = mpc_controller(current_state);
        cout << "INFO: best controll: " << trans(action) << "\n";

        for (int i = 0; i < NR_OF_CONTROLS; i++)
        {
            controls[i] = action(i, 0);
        }

        matrix<double, NR_OF_STATES, 1> calc_state = mpc_controller.get_A() * current_state + mpc_controller.get_B() * action;
        for (int i = 0; i < NR_OF_STATES; i++)
        {
            predicted_state[i] = calc_state(i, 0);
        }
    }
}

mpc<NR_OF_STATES, NR_OF_CONTROLS, PREDICTION_HORIZON> initialize_mpc_object(int i)
{

    // Linear Equation of the control and process dynamics
    // Note: Contrary to MATLAB, we are specifying the following states, not the derivative
    // x_{i+1} = A*x_i + B*u_i + C

    matrix<double, NR_OF_STATES, NR_OF_STATES> A;
    A = ss_A[i];

    matrix<double, NR_OF_STATES, NR_OF_CONTROLS> B;
    B = ss_B[i];

    matrix<double, NR_OF_STATES, 1> C;
    C = 0, 0, 0, 0, 0;

    // Matrix Q: Costs of divergence in states
    matrix<double, NR_OF_STATES, 1> Q;
    Q = 1.5, 0, 0, 1, 15;

    // Matrix R: Costs of controls
    matrix<double, NR_OF_CONTROLS, 1> R;
    R = 0.6, 0.5, 0.5; // Dlib does not allwo to punish rates

    // upper, lower: Control limits
    matrix<double, NR_OF_CONTROLS, 1> upper;
    matrix<double, NR_OF_CONTROLS, 1> lower;
    upper = 0.2, 1, 1;
    lower = -0.2, 0, 0;

    mpc<NR_OF_STATES, NR_OF_CONTROLS, PREDICTION_HORIZON> mpc_controller(A, B, C, Q, R, lower, upper);

    return mpc_controller;
}

mpc<NR_OF_STATES, NR_OF_CONTROLS, PREDICTION_HORIZON> get_best_mpc_object(int speed)
{
    int i = max(int(speed) - 2, 0);
    if (DEBUG)
        cout << "DEBUG: MPC GPS obst used with parameters idx " << i << "\n";
    return mpc_objects[i];
}