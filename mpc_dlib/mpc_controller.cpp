#include <dlib/control.h>
#include <math.h>

using namespace std;
using namespace dlib;

#define PI 3.14159265

// Constants for Model
const int CONTROL_HORIZON = 30;
// Contrary to MATLAB, dlib does not allow different control/prediction horizons
const int PREDICTION_HORIZON = CONTROL_HORIZON;
const int NR_OF_STATES = 6;
const int NR_OF_CONTROLS = 3;

// Parameters for vehicle
const float m = 50;     // Mass
const float lf = 1;     // Distance mass center/front wheel
const float lr = 1;     // Distance mass center/rear wheel
const float cf = 19000; // Cornering Stiffness front
const float cr = 33000; // Cornering Stiffness rear
const float iz = 2800;  // Yaw Moment of Innertia

const float cruising_speed = 4;

mpc<NR_OF_STATES, NR_OF_CONTROLS, PREDICTION_HORIZON> mpc_objects[21];

// extern C allows to use C-Linkage for next scope, required for ctypes,
extern  "C" {

mpc<NR_OF_STATES, NR_OF_CONTROLS, PREDICTION_HORIZON> initialize_for_speed(float v);

void initialize_mpc_objects() {
    for(int i=0; i<=20; i++) {
        mpc<NR_OF_STATES, NR_OF_CONTROLS, PREDICTION_HORIZON> mpc_object = initialize_for_speed((float)i/5);
        mpc_objects[i] = mpc_object;
    }

}

mpc<NR_OF_STATES, NR_OF_CONTROLS, PREDICTION_HORIZON> initialize_for_speed(float v){

     cout << "VERBOUSE: Starting to create MPC object\n";
    // Linear Equation of the control and process dynamics
    // Note: Contrary to MATLAB, we are specifying the following states, not the derivative
    // x_{i+1} = A*x_i + B*u_i + C

    // Matrix A: How the change in states depends on the current
    matrix<double, NR_OF_STATES, NR_OF_STATES> A;
    A = 1, 0, -v, 0, -1, v, // x_position
        0, 1, v, 0, 1, 0,   // y_position
        0, 0, 1, 1, 0, 0,   // yaw_angle
        0, 0, 0, 1-(2*cf*lf*lf+2*cr*lr*lr)/iz/v, -(2*cf*lf-2*cr*lr)/iz/v, 0,  // yaw_rate
        0, 0, 0, -v-(2*cf*lf-2*cr*lr)/m/v, 1-(2*cf+2*cr)/m/v, 0,        // schwimm_angle
        0, 0, 0, 0, 0, 0.9; // speed TODO: Something more elaborate here


    // Matrix B: How the change in states depends on the controls
    matrix<double, NR_OF_STATES, NR_OF_CONTROLS> B;
    // Steering Angle, Throttle, Breaks
    B = 0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        2*cf*lf/iz, 0, 0,
        2*cf/m, 0, 0,
        0, 1, -2;   // TODO: Something more elaborate here.


    // Matrix C: Unconditional changes in state / disturbance
    matrix<double, NR_OF_STATES, 1> C;
    C = 0, 0, 0, 0, 0, 0;  // No unconditional changes in state

    // Matrix Q: Costs of divergence in states
    matrix<double, NR_OF_STATES, 1> Q;
    Q = 0, 0, 2, 0, 0, 0.1;     // TODO: not quite right yet

    // Matrix R: Costs of controls
    matrix<double, NR_OF_CONTROLS, 1> R;
    R = 0, 0, 0;   // TODO: is there any way to do punish big rates here?

    // upper, lower: Costs of changes in controlss
    matrix<double, NR_OF_CONTROLS, 1> upper;
    matrix<double, NR_OF_CONTROLS, 1> lower;
    upper = 0.78, 1, 1;
    lower = -0.78, 0, 0;

    mpc<NR_OF_STATES, NR_OF_CONTROLS, PREDICTION_HORIZON> mpc_controller(A, B, C, Q, R, lower, upper);

    printf("DEBUG: MPC Object set up for speed %.1f m/s\n", v);
    return mpc_controller;
}

matrix<double, NR_OF_STATES> calculate_target_state(
                                        double x_target,
                                        double y_target,
                                        bool last_controls,
                                        matrix<double, NR_OF_STATES, 1> current_state){

    double x_current = current_state(0,0);
    double y_current = current_state(1,0);

    double yaw_target = atan(abs(y_current) - abs(y_target)) / (abs(x_current) - abs(x_target));
    if((x_current - x_target > 0) && ((y_current - y_target < 0))) {           // 2nd Q
        yaw_target = PI - yaw_target;
    } else if((x_current - x_target > 0) && ((y_current - y_target > 0))) {    // 3rd Q
        yaw_target = -PI - yaw_target;
    } else if((x_current - x_target < 0) && ((y_current - y_target > 0))) {    // 4th Q
        yaw_target = -yaw_target;
    }

    // TODO: Deal with full cyclicality of this thing (see MATLAB)

    matrix<double, NR_OF_STATES, 1> target_state;
    target_state = x_target, y_target, yaw_target, 0, 0, cruising_speed;

    cout << "DEBUG: target state: " << trans(target_state);
    return target_state;

}

// TODO: Change return type & argument types into something ctypes frinedly
matrix<double, NR_OF_CONTROLS> predict(
                                        mpc<NR_OF_STATES, NR_OF_CONTROLS, PREDICTION_HORIZON> mpc_controller,
                                        double x_target,
                                        double y_target,
                                        bool last_controls,
                                        matrix<double, NR_OF_STATES> current_state) {
    matrix<double, NR_OF_STATES> target_state = calculate_target_state(x_target, y_target, last_controls, current_state);

    mpc_controller.set_target(target_state);

    matrix<double, NR_OF_CONTROLS, 1> action = mpc_controller(current_state);
    cout << "INFO: best control: " << trans(action);

    // simulate

    double v = 4;
    action = 0.78, 1, 0;
    matrix<double, NR_OF_STATES, NR_OF_STATES> A;
    A = 1, 0, -v, 0, -1, v, // x_position
        0, 1, v, 0, 1, 0,   // y_position
        0, 0, 1, 1, 0, 0,   // yaw_angle
        0, 0, 0, 1-(2*cf*lf*lf+2*cr*lr*lr)/iz/v, -(2*cf*lf-2*cr*lr)/iz/v, 0,  // yaw_rate
        0, 0, 0, -v-(2*cf*lf-2*cr*lr)/m/v, 1-(2*cf+2*cr)/m/v, 0,        // schwimm_angle
        0, 0, 0, 0, 0, 0.9; // speed TODO: Something more elaborate here
        // Matrix B: How the change in states depends on the controls
    matrix<double, NR_OF_STATES, NR_OF_CONTROLS> B;
    // Steering Angle, Throttle, Breaks
    B = 0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        2*cf*lf/iz, 0, 0,
        2*cf/m, 0, 0,
        0, 1, -2;   // TODO: Something more elaborate here.


    // Matrix C: Unconditional changes in state / disturbance
    matrix<double, NR_OF_STATES, 1> C;
    C = 0, 0, 0, 0, 0, 0;  // No unconditional changes in state
    matrix<double, NR_OF_STATES, 1> next_state = A*current_state + B*action + C;
    cout << "DEBUG: expected next state: " << trans(next_state);
    cout << trans(A*current_state);
    cout << trans(B*action);

    return action;
}

}