#include <dlib/control.h>
#include <math.h>

using namespace std;
using namespace dlib;

#define PI 3.14159265

#define NO_DISTANCE_RESTRICTION -100.0
#define SAFETY_FACTOR 1.5
#define CUTOFF_DISTANCE 10.0
#define DIST_CONVERSION 1.0 // TODO: Google real value here

// Constants for Model
const int CONTROL_HORIZON = 60;
const float STEP_SIZE = 0.1;
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

const float cruising_speed = 4;
matrix<double, NR_OF_STATES> last_state;

mpc<NR_OF_STATES, NR_OF_CONTROLS, PREDICTION_HORIZON> mpc_objects[20];

// extern C allows to use C-Linkage for next scope, required for ctypes,
extern "C"
{

    mpc<NR_OF_STATES, NR_OF_CONTROLS, PREDICTION_HORIZON> initialize_for_speed(float v);

    void initialize_mpc_objects()
    {
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
        A = 1, 0, -v * STEP_SIZE, 0, -1 * STEP_SIZE, STEP_SIZE, 0,                                                                            // x_position
            0, 1, v * STEP_SIZE, 0, 1 * STEP_SIZE, 0, 0,                                                                                      // y_position
            0, 0, 1, 1 * STEP_SIZE, 0, 0, 0,                                                                                                  // yaw_angle
            0, 0, 0, 1 - (2 * cf * lf * lf + 2 * cr * lr * lr) / iz / v * STEP_SIZE, -(2 * cf * lf - 2 * cr * lr) / iz / v * STEP_SIZE, 0, 0, // yaw_rate
            0, 0, 0, -v - (2 * cf * lf - 2 * cr * lr) / m / v * STEP_SIZE, 1 - (2 * cf + 2 * cr) / m / v * STEP_SIZE, 0, 0,                   // schwimm_angle
            0, 0, 0, 0, 0, 1 - 0.1 * STEP_SIZE, 0,                                                                                            // speed TODO: Something more elaborate here
            0, 0, 0, 0, 0, -STEP_SIZE, 1;                                                                                                     // Distance to Obstacle/Target

        double s_softener = ((v * v * v) / (cruising_speed * cruising_speed * cruising_speed));

        // Matrix B: How the change in states depends on the controls
        matrix<double, NR_OF_STATES, NR_OF_CONTROLS> B;
        // Steering Angle, Throttle, Breaks
        B = 0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        2 * cf * lf / iz * STEP_SIZE, 0, 0,
        2 * cf / m * STEP_SIZE, 0, 0,
        0, (1 - s_softener) * STEP_SIZE, -2 * STEP_SIZE, // TODO: Something more elaborate here.
            0, 0, 0;

        // Matrix C: Unconditional changes in state / disturbance
        matrix<double, NR_OF_STATES, 1> C;
        C = 0, 0, 0, 0, 0, 0, 0; // No unconditional changes in state

        // Matrix Q: Costs of divergence in states
        matrix<double, NR_OF_STATES, 1> Q;
        Q = 0, 0, 2, 0, 0, 0.1, 10; // TODO: maybe set speed weight to 0?

        // Matrix R: Costs of controls
        matrix<double, NR_OF_CONTROLS, 1> R;
        R = 0, 0.1, 0.1; // TODO: is there any way to do punish big rates here?, Otherwise I might have to introduce a proxy state, so that we do not move more than we can in one go.

        // upper, lower: Control limits
        matrix<double, NR_OF_CONTROLS, 1> upper;
        matrix<double, NR_OF_CONTROLS, 1> lower;
        upper = 0.78, 1, 1;
        lower = -0.78, 0, 0;

        mpc<NR_OF_STATES, NR_OF_CONTROLS, PREDICTION_HORIZON> mpc_controller(A, B, C, Q, R, lower, upper);

        printf("DEBUG: MPC Object (Blind) set up for speed %.1f m/s\n", v);
        return mpc_controller;
    }

    double calculate_target_distance(double speed, double x1, double y1, double x2, double y2, bool is_last, double distance)
    {
        double target_distance = NO_DISTANCE_RESTRICTION;

        if (distance < CUTOFF_DISTANCE)
        {
            target_distance = speed * SAFETY_FACTOR;
        }

        if ((abs(x1 * DIST_CONVERSION - x2 * DIST_CONVERSION) + abs(y1 * DIST_CONVERSION - y2 * DIST_CONVERSION)) < CUTOFF_DISTANCE)
        {
            target_distance = min(target_distance, (abs(x1 - x2) + abs(y1 - y2)));
        }

        return target_distance;
    }

    matrix<double, NR_OF_STATES, 1> calculate_target_state(
        double x_target,
        double y_target,
        double x_current, double y_current, double target_distance)
    {

        double yaw_target = atan(abs(y_current - y_target) / abs(x_current - x_target));
        if ((x_current - x_target > 0) && ((y_current - y_target < 0)))
        { // 2nd Q
            yaw_target = PI - yaw_target;
        }
        else if ((x_current - x_target > 0) && ((y_current - y_target > 0)))
        { // 3rd Q
            yaw_target = -PI - yaw_target;
        }
        else if ((x_current - x_target < 0) && ((y_current - y_target > 0)))
        { // 4th Q
            yaw_target = -yaw_target;
        }
        // TODO: Deal with full cyclicality of this thing (see MATLAB)

        matrix<double, NR_OF_STATES, 1> target_state;
        target_state = x_target, y_target, yaw_target, 0, 0, cruising_speed, target_distance;

        cout << "DEBUG: target state: " << trans(target_state);
        return target_state;
    }

    double calculate_yaw_angle()
    {
        // getting the angle from gps, set initially to right direction
        return 0.0;
    }

    double calculate_yaw_rate()
    {
        // see above, set to 0. OR: propagate from last simulation
        return 0.0;
    }

    mpc<NR_OF_STATES, NR_OF_CONTROLS, PREDICTION_HORIZON> get_best_mpc_object(double speed)
    {
        //int idx = min((int)(speed * 5), 19);          // TODO: WHY THE FUCK DOES ONLY WORK FOR SPEEDS ~ >2???
        int idx = 19;
        return mpc_objects[idx];
    }

    void predict(
        double x_target, double y_target,
        double steering_angle, double throttle, double breaks,
        double x, double y, double speed, double distance,
        double *controls, double *predicted_observables)
    {

        double yaw_angle = calculate_yaw_angle();
        double yaw_rate = calculate_yaw_angle();
        double schwimm_angle = 0.0;
        matrix<double, NR_OF_STATES, 1> current_state;
        current_state = x, y, yaw_angle, yaw_rate, schwimm_angle, speed, distance;
        double target_distance = calculate_target_distance(speed, x, y, x_target, y_target, false, distance);

        mpc<NR_OF_STATES, NR_OF_CONTROLS, PREDICTION_HORIZON> mpc_controller = get_best_mpc_object(speed);

        matrix<double, NR_OF_STATES, 1> target_state = calculate_target_state(x_target, y_target, x, y, target_distance);
        mpc_controller.set_target(target_state);

        matrix<double, NR_OF_CONTROLS, 1> action = mpc_controller(current_state);
        cout << "INFO: best controll: " << trans(action);

        for (int i = 0; i < NR_OF_CONTROLS; i++)
        {
            controls[i] = action(i, 0);
        }
    }
}