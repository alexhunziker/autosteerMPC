#include <dlib/control.h>
#include <dlib/logger.h>
#include <math.h>

using namespace std;
using namespace dlib;

logger dlog("mpc_cv");

#define PI 3.14159265

// Constants for Model
const int CONTROL_HORIZON = 30;
const float STEP_SIZE = 0.01;
// Contrary to MATLAB, dlib does not allow different control/prediction horizons
const int PREDICTION_HORIZON = CONTROL_HORIZON;
const int NR_OF_STATES = 7;
const int NR_OF_CONTROLS = 3;

// Parameters for vehicle
const float m = 1575;   // Mass
const float lf = 1;     // Distance mass center/front wheel
const float lr = 1;     // Distance mass center/rear wheel
const float cf = 19000; // Cornering Stiffness front
const float cr = 33000; // Cornering Stiffness rear
const float iz = 2800;  // Yaw Moment of Innertia

const double cruising_speed = 4;
matrix<double, NR_OF_STATES> last_state;

mpc<NR_OF_STATES, NR_OF_CONTROLS, PREDICTION_HORIZON> mpc_objects[20];

// extern C allows to use C-Linkage for next scope, required for ctypes,
extern "C"
{

    mpc<NR_OF_STATES, NR_OF_CONTROLS, PREDICTION_HORIZON> initialize_for_speed(float v);

    void initialize_mpc_objects()
    {
        dlog.set_level(LALL);
        for (int i = 1; i <= 20; i++)
        {
            mpc<NR_OF_STATES, NR_OF_CONTROLS, PREDICTION_HORIZON> mpc_object = initialize_for_speed((float)i / 5);
            mpc_objects[i - 1] = mpc_object;
        }
    }

    mpc<NR_OF_STATES, NR_OF_CONTROLS, PREDICTION_HORIZON> initialize_for_speed(float v)
    {

        // Linear Equation of the control and process dynamics
        // Note: Contrary to MATLAB, we are specifying the following states, not the derivative
        // x_{i+1} = A*x_i + B*u_i + C

        // Matrix A: How the change in states depends on the current
        matrix<double, NR_OF_STATES, NR_OF_STATES> A;
        //A = 1, 0, -v * STEP_SIZE, 0, -1 * STEP_SIZE, v * STEP_SIZE, 0,                                                                        // x_position
        //    0, 1, v * STEP_SIZE, 0, 1 * STEP_SIZE, 0, 0,                                                                                      // y_position
        //    0, 0, 1, 1, 0, 0, 0,                                                                                                              // yaw_angle
        //    0, 0, 0, 1 - (2 * cf * lf * lf + 2 * cr * lr * lr) / iz / v * STEP_SIZE, -(2 * cf * lf - 2 * cr * lr) / iz / v * STEP_SIZE, 0, 0, // yaw_rate
        //    0, 0, 0, -v - (2 * cf * lf - 2 * cr * lr) / m / v * STEP_SIZE, 1 - (2 * cf + 2 * cr) / m / v * STEP_SIZE, 0, 0,                   // schwimm_angle
        //    0, 0, 0, 0, 0, 1 - 0.1 * STEP_SIZE - max((float)0.0, 4 - v * 4), 0,                                                               // speed TODO: Something more elaborate here
        //    0, 0, 0, 0, 0, 0, 0;                                                                                                              // seconds_to_obstacle

        A = 1, 0, -0.2, 0.00420655416113835, -0.00526604137908912, 0.0995016625083195, 0,
        0, 1, 0.2, -0.00420655416113835, 0.00526604137908912, 0, 0,
        0, 0, 1, 0.0517384006163168, -0.00303079475590676, 0, 0,
        0, 0, 0, 0.227045209024172, -0.0147018562144484, 0, 0,
        0, 0, 0, -0.0514564967505693, 0.00333196362764945, 0, 0,
        0, 0, 0, 0, 0, 0.990049833749168, 0,
        0, 0, 0, 0, 0, -0.0995016625083194, 1;

        // Matrix B: How the change in states depends on the controls
        matrix<double, NR_OF_STATES, NR_OF_CONTROLS>
            B;
        // Steering Angle, Throttle, Breaks
        //B = 0, 0, 0,
        //0, 0, 0,
        //0, 0, 0,
        //2 * cf * lf / iz * STEP_SIZE, 0, 0,
        //2 * cf / m * STEP_SIZE, 0, 0,
        //0, 1 * STEP_SIZE, -2 * STEP_SIZE,
        //0, 0, 0; // TODO: Something more elaborate here.
        B = -0.0832670692070945, 0.00498337491680536, -0.00996674983361072,
        0.0832670692070945, 0, 0,
        0.0990492070239366, 0, 0,
        1.56685387470395, 0, 0,
        0.678434587002140, 0, 0,
        0, 0.0995016625083194, -0.199003325016639,
        0, -0.00498337491680536, 0.00996674983361071;

        // Matrix C: Unconditional changes in state / disturbance
        matrix<double, NR_OF_STATES, 1>
            C;
        C = 0, 0, 0, 0, 0, 0, 0; // No unconditional changes in state

        // Matrix Q: Costs of divergence in states
        matrix<double, NR_OF_STATES, 1> Q;
        Q = 0, 0, 0, 5, 0, 2, 0; // TODO: not quite right yet

        // Matrix R: Costs of controls
        matrix<double, NR_OF_CONTROLS, 1> R;
        R = 0, 0.2, 0.2; // TODO: is there any way to do punish big rates here?

        // upper, lower: Control limits
        matrix<double, NR_OF_CONTROLS, 1> upper;
        matrix<double, NR_OF_CONTROLS, 1> lower;
        upper = 0.78, 1, 1;
        lower = -0.78, 0, 0;

        mpc<NR_OF_STATES, NR_OF_CONTROLS, PREDICTION_HORIZON> mpc_controller(A, B, C, Q, R, lower, upper);

        dlog << LDEBUG << "MPC Object set up for speed " << v << "m/s";
        return mpc_controller;
    }

    double get_target_speed(double x_target, double y_target, double yaw_rate_target, double x_current, double y_current)
    {
        double distance_speed = sqrt(pow(x_current - x_target, 2) + pow(y_current - y_target, 2)) / 2; //TODO: Do better
        double yaw_speed = 5 - yaw_rate_target * 3;
        cout << "distance_speed " << distance_speed << " yaw_speed " << yaw_speed;
        return min(cruising_speed, min(distance_speed, yaw_speed));
    }

    matrix<double, NR_OF_STATES, 1> calculate_target_state(
        double x_target,
        double y_target,
        double yaw_rate_target,
        double x_current, double y_current)
    {

        double target_speed = get_target_speed(x_target, y_target, 0.78, x_current, y_current);
        matrix<double, NR_OF_STATES, 1>
            target_state;
        target_state = x_target, y_target, 0, -0.78, 0, target_speed, 0;

        dlog << LDEBUG << "target state: " << trans(target_state);
        return target_state;
    }

    mpc<NR_OF_STATES, NR_OF_CONTROLS, PREDICTION_HORIZON> get_best_mpc_object(double speed)
    {
        int idx = min((int)(speed * 5), 19); // TODO: WHY THE FUCK DOES ONLY WORK FOR SPEEDS ~ >2???
        return mpc_objects[idx];
    }

    void predict(
        double x_target, double y_target,
        double steering_angle, double throttle, double breaks,
        double x, double y, double speed, double yaw_angle, double yaw_rate, double schwimm_angle, double obstacle_time,
        double *controls, double *predicted_state_holder)
    {

        matrix<double, NR_OF_STATES, 1> current_state;
        current_state = x, y, yaw_angle, yaw_rate, schwimm_angle, speed, obstacle_time;

        mpc<NR_OF_STATES, NR_OF_CONTROLS, PREDICTION_HORIZON> mpc_controller = get_best_mpc_object(speed);

        matrix<double, NR_OF_STATES, 1> target_state = calculate_target_state(x_target, y_target, yaw_rate, x, y);
        mpc_controller.set_target(target_state);

        matrix<double, NR_OF_CONTROLS, 1> action = mpc_controller(current_state);
        cout << "INFO: best controll: " << trans(action);

        for (int i = 0; i < NR_OF_CONTROLS; i++)
        {
            controls[i] = action(i, 0);
        }

        matrix<double, NR_OF_STATES, 1> predicted_state = mpc_controller.get_A() * current_state + mpc_controller.get_B() * action + mpc_controller.get_C();
        cout << "INFO: predicted state (by controller): " << trans(predicted_state);
        for (int i = 0; i > NR_OF_STATES; i++)
        {
            predicted_state_holder[i] = predicted_state(i, 0);
        }
    }
}