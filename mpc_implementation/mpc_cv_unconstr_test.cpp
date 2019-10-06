#include <iostream>
#include <fstream>
#include "mpc_cv_unconstr.cpp"
#include <dlib/string.h>
#include <math.h>

using namespace std;
using namespace dlib;

#define PI 3.14159265

static double TARGET_YAW = 0.7;
static double TARGET_X = 10;
static double TARGET_Y = 10;
static double TARGET_SPEED = 4;
static double TIME_STEP = 0.1;

double accumulated_diff = 0.0;
double x_calc = 0.0;
double y_calc = 0.0;
double yaw_calc = 0.0;
double sanitized_yaw = 0.0;
double offset = 2;

double get_y_offset(double v, double target_yaw_rate, double actual_yaw_rate)
{
    // This is an approximation, assumes constant speed. But is used for testing only
    double correction_angle = (actual_yaw_rate - target_yaw_rate) + accumulated_diff;
    cout << "DEBUG: this diff is " << actual_yaw_rate << "-" << target_yaw_rate << "=" << (actual_yaw_rate - target_yaw_rate) << "\n";
    accumulated_diff += (actual_yaw_rate - target_yaw_rate) * TIME_STEP; // is this true?
    offset += correction_angle * v * TIME_STEP * -1;
    cout << "Calculated Y-Offset is: " << offset << "\n";
    return offset;
}

double sign(double x)
{
    if (x > 0.0)
        return 1;
    else
        return -1;
}

void calc_position(double yaw, double yaw_rate, double v)
{
    x_calc += cos(yaw) * v * TIME_STEP;
    y_calc += sin(yaw) * v * TIME_STEP;
    yaw_calc += yaw_rate * TIME_STEP;

    double yaw_off = yaw_calc - PI;
    sanitized_yaw = fmod(abs(yaw_off), (2 * PI)) * sign(yaw_off) + PI;
    cout << "Predicted position is: x= " << x_calc << ", y= " << y_calc << ", yaw= " << sanitized_yaw << "\n";
}

double get_target_speed()
{
    double diff_x = x_calc - TARGET_X;
    double diff_y = y_calc - TARGET_Y;
    double distance_speed = sqrt(pow(diff_x, 2) + pow(diff_y, 2)) / 2;

    double yaw_speed = (TARGET_SPEED - 3 * TARGET_YAW) + 1.5;
    double current_appropriate_speed = min(yaw_speed, distance_speed);
    double target_speed = min(current_appropriate_speed, TARGET_SPEED);
    cout << "Target speed is: " << target_speed << "\n";
    return target_speed;
}

int main(int argc, char **argv)
{
    initialize_mpc_objects();
    double controls[NR_OF_CONTROLS];
    double state[NR_OF_STATES] = {0, 0, 0, 0, 0, 2};
    double position[3];

    for (int i = 1; i < 100; i++)
    {
        calc_position(state[1], state[2], state[5]);
        predict(get_target_speed(), x_calc, y_calc, state[5], get_y_offset(state[5], TARGET_YAW, state[2]), state[1], state[2], state[3], TARGET_YAW, controls, state);
        cout << "Predicted state is: ";
        for (int i = 0; i < NR_OF_STATES; i++)
            cout << state[i] << ", ";
        cout << "\n";
    }

    return 0;
}