#include <iostream>
#include <fstream>
#include "mpc_gps_obst.cpp"
#include <dlib/string.h>
#include <math.h>

using namespace std;
using namespace dlib;

#define PI 3.14159265

static double TARGET_X = -10;
static double TARGET_Y = -10;
static double TARGET_SPEED = 4;
static double TIME_STEP = 0.1;

double x_calc = 0.0;
double y_calc = 0.0;
double yaw_calc = 0.0;
double sanitized_yaw = 0.0;
double distance_to_collision = 10.0;

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

double get_target_speed(double target_yaw)
{
    double diff_x = x_calc - TARGET_X;
    double diff_y = y_calc - TARGET_Y;
    double distance_speed = sqrt(pow(diff_x, 2) + pow(diff_y, 2)) / 2;

    double yaw_speed = (TARGET_SPEED - 3 * abs(target_yaw - sanitized_yaw)) + 1.5;
    double current_appropriate_speed = min(yaw_speed, distance_speed);
    double target_speed = min(current_appropriate_speed, TARGET_SPEED);
    cout << "Target speed is: " << target_speed << "\n";
    return target_speed;
}

double calculate_yaw_target(double yaw)
{
    double yaw_target = atan2(TARGET_Y - y_calc, TARGET_X - x_calc);

    // Adjust so we actually take shortest diff
    double yaw_diff = abs(yaw - yaw_target);
    if (yaw_diff > PI)
    {
        if (yaw_target < 0 && yaw > 0)
            yaw_target = yaw_target + 2 * PI;
        if (yaw_target > 0 && yaw < 0)
            yaw_target = yaw_target - 2 * PI;
    }
    if (DEBUG)
        cout << "Calculated yaw target is " << yaw_target << "\n";
    return yaw_target;
}

double get_collision_distance(double v)
{
    distance_to_collision -= TIME_STEP * v;
    if (distance_to_collision < 1.0)
        return 0;
    return distance_to_collision / v;
}

double sanitize_velocity(double v)
{
    return max(0.0, v);
}

int main(int argc, char **argv)
{
    initialize_mpc_objects();
    double controls[NR_OF_CONTROLS];
    double state[NR_OF_STATES] = {0, 0, 0, 2, 3};
    double position[3];

    for (int i = 1; i < 100; i++)
    {
        double v = sanitize_velocity(state[3]);
        calc_position(state[0], state[1], v);
        double target_yaw = calculate_yaw_target(sanitized_yaw);
        predict(get_target_speed(target_yaw), x_calc, y_calc, v, state[0], state[1], state[2], target_yaw, get_collision_distance(v), controls, state);
        cout << "Predicted state is: ";
        for (int i = 0; i < NR_OF_STATES; i++)
            cout << state[i] << ", ";
        cout << "\n";
    }

    return 0;
}