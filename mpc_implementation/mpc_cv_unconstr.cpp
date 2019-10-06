#include <dlib/control.h>
#include <math.h>

#include "mpc_cv_unconstr_ss.cpp"

using namespace std;
using namespace dlib;

#define PI 3.14159265

// Constants for Model
const int CONTROL_HORIZON = 60;
const float STEP_SIZE = 0.1;
// Contrary to MATLAB, dlib does not allow different control/prediction horizons
const int PREDICTION_HORIZON = CONTROL_HORIZON;
const int NR_OF_STATES = 6;
const int NR_OF_CONTROLS = 4;

const float CRUISING_SPEED = 4.0;

mpc<NR_OF_STATES, NR_OF_CONTROLS, PREDICTION_HORIZON> mpc_objects[3];

mpc<NR_OF_STATES, NR_OF_CONTROLS, PREDICTION_HORIZON> initialize_for_speed(float v);

extern "C"
{
    void initialize_mpc_objects()
    {
        for (int i = 0; i <= 3; i++)
        {
            mpc<NR_OF_STATES, NR_OF_CONTROLS, PREDICTION_HORIZON> mpc_object = initialize_mpc_object(i);
            mpc_objects[i] = mpc_object;
        }
        printf("DEBUG: MPC Objects set up for CV Unconstrained.", v);
    }

    void predict(
        double speed_target,
        double x, double y, double speed, double y_offset,
        double yaw, double yaw_rate, double schwimm_angle,
        double yaw_rate_target, double *controls, double *predicted_state)
    {

        matrix<double, NR_OF_STATES, 1> current_state;
        current_state = y_offset, yaw, yaw_rate, schwimm_angle, yaw_rate_target, speed; // TODO: yaw rate target here sensible?

        mpc<NR_OF_STATES, NR_OF_CONTROLS, PREDICTION_HORIZON> mpc_controller = get_best_mpc_object(speed);

        matrix<double, NR_OF_STATES, 1> target_state;
        target_state = 0, 0, yaw_rate_target, 0, yaw_rate_target, speed_target;
        mpc_controller.set_target(target_state);

        matrix<double, NR_OF_CONTROLS, 1> action = mpc_controller(current_state);
        cout << "INFO: best controll: " << trans(action);

        for (int i = 0; i < NR_OF_CONTROLS; i++)
        {
            controls[i] = action(i, 0);
        }

        matrix<double, NR_OF_STATES, 1> predicted_state = mpc_controller.getA() * current_state + mpc_controller.getB() * action;
        for (int i = 0; i < NR_OF_STATES; i++)
        {
            predicted_state[i] = predicted_state(i, 0);
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
    C = 0, 0, 0, 0, 0, 0;

    // Matrix Q: Costs of divergence in states
    matrix<double, NR_OF_STATES, 1> Q;
    Q = 5, 0, 1, 0, 10, 1;

    // Matrix R: Costs of controls
    matrix<double, NR_OF_CONTROLS, 1> R;
    R = 0, 0, 0.1, 0.1; // Dlib does not allwo to punish rates

    // upper, lower: Control limits
    matrix<double, NR_OF_CONTROLS, 1> upper;
    matrix<double, NR_OF_CONTROLS, 1> lower;
    upper = 0.78, 999, 1, 1;
    lower = -0.78, -999, 0, 0;

    mpc<NR_OF_STATES, NR_OF_CONTROLS, PREDICTION_HORIZON> mpc_controller(A, B, C, Q, R, lower, upper);

    return mpc_controller;
}

mpc<NR_OF_STATES, NR_OF_CONTROLS, PREDICTION_HORIZON> get_best_mpc_object(int speed)
{
    int i = max(int(speed) - 2, 0);
    return mpc_objects[i];
}