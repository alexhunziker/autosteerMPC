#include <dlib/control.h>

using namespace std;
using namespace dlib;

const int CONTROL_HORIZON = 20;
const int PREDICTION_HORIZON = CONTROL_HORIZON;
const int NR_OF_STATES = 6;
const int NR_OF_CONTROLS = 3;

mpc<NR_OF_STATES, NR_OF_CONTROLS, PREDICTION_HORIZON> initialize(int speed) {

    // Linear Equation of the controll and process dynamics
    // x_{i+1} = A*x_i + B*u_i + C

    // Matrix A: How the change in states depends on the current
    matrix<double, NR_OF_STATES, NR_OF_STATES> A;
    // calculation and assignment of A

    // Matrix B: How the change in states depends on the controls
    matrix<double, NR_OF_STATES, NR_OF_CONTROLS> B;
    // calculation and assignemnts of B

    // Matrix C: Unconditional changes in state / disturbance
    matrix<double, NR_OF_STATES, 1> C;
    // calcualtion and assignment of C

    // Matrix Q: Costs of divergence in states
    matrix<double, NR_OF_STATES, 1> Q;

    // Matrix R: Costs of controls
    matrix<double, NR_OF_CONTROLS, 1> R;

    // upper, lower: Costs of changes in controls
    matrix<double, NR_OF_CONTROLS, 1> upper;
    matrix<double, NR_OF_CONTROLS, 1> lower;

    mpc<NR_OF_STATES, NR_OF_CONTROLS, PREDICTION_HORIZON> mpc_controller(A, B, C, Q, R, lower, upper);
    return mpc_controller
}

// TODO: Determine best return value for ctypes
matrix<double, NR_OF_CONTROLS> predict(mpc<NR_OF_STATES, NR_OF_CONTROLS, PREDICTION_HORIZON> mpc_controller, double x_target, double y_target, bool last_controls, bool current_state) {
    target_state = calculate_target_state();

    mpc_controller.set_target(target_state);

    matrix<double, NR_OF_CONTROLS, 1> action = mpc_controller(state);

    return action
}

matrix<double, NR_OF_STATES> calculate_target_state(){

}