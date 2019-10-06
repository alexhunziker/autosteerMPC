#include <iostream>
#include <fstream>
#include "mpc_cv_unconstr.cpp"
#include <dlib/string.h>
#include <math.h>

using namespace std;
using namespace dlib;

#define PI 3.14159265

static double TARGET_YAW = 0.5;
static double TARGET_X = 10;
static double TARGET_Y = 10;
static double TARGET_SPEED = 4;
static double TIME_STEP = 0.01;

double accumulated_diff = 0.0;
double x_calc = 0.0;
double y_calc = 0.0;
double yaw_calc = 0.0;
double sanitized_yaw = 0.0;
double offset = 4;

int main(int argc, char **argv)
{
    initialize_mpc_objects();
    double controls[NR_OF_CONTROLS];
    double state[NR_OF_STATES] = {0, 0, 0, 0, 0, 2};
    double position[3];

    for (int i = 1; i < 1000; i++)
    {
        calc_position(state[1], state[2], state[5]);
        predict(get_speed_target(), x_calc, y_calc, state[5], get_y_offset(), sanitized_yaw, state[2], state[3], TARGET_YAW, controls, state);
        cout << "Predicted state is: " << state << "\n";
    }

    return 0;
}

double get_y_offset(double v, double target_yaw_rate, double actual_yaw_rate)
{
    // This is an approximation, assumes constant speed. But is used for testing only
    double correction_angle = (actual_yaw_rate - target_yaw_rate) + accumulated_diff;
    accumulated_diff += (actual_yaw_rate - target_yaw_rate) * TIME_STEP; // is this true?
    offset += correction_angle * v * TIME_STEP;
    cout << "Calculated Y-Offset is: " << offset;
    return offset
}

void calc_position(double yaw, double yaw_rate, double v)
{
    x_calc += cos(yaw) * v * TIME_STEP;
    y_calc += sin(yaw) * v * TIME_STEP;
    yaw_calc += yaw_rate * TIME_STEP;

    double yaw_off = yaw_calc - PI;
    sanitized_yaw = mod(abs(yaw_off), 2 * PI) * sign(yaw_off) + PI;
    cout << "Predicted position is: x= " << x_calc << ", y= " << y_calc << ", yaw= " << sanitized_yaw << "\n";
}

double get_target_speed()
{
    double diff_x = x_calc - TARGET_X;
    double diff_y = y_calc - TARGET_Y;
    double distance_speed = sqrt(pow(diff_x, 2) + pow(diff_y, 2)) / 2;

    double yaw_speed = TARGET_SPEED - 3*TARGET_YAW) + 1.5;
    double current_appropriate_speed = min(yaw_speed, distance_speed);
    double target_speed = min(current_appropriate_speed, yaw_speed);
    cout << "Target speed is: " << target_speed << "\n";
    return target_speed;
}