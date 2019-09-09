#include <iostream>
#include <dlib/string.h>
#include "mpc_blind.cpp"

using namespace std;
using namespace dlib;

double bu[NR_OF_STATES] = {0, 0, 0, 0, 0, 2};

int main(int argc, char **argv)
{
    int fail_counter = 0;

    fail_counter += break_collision_hard();
    fail_counter += break_collision_soft();
    fail_counter += break_curve();
    fail_counter += break_destination();
    fail_counter += left_destination_back();
    fail_counter += left_destination_front();
    fail_counter += right_destiation_front();
    fail_counter += right_destination_back();
    fail_counter += straight_ahead();
    fail_counter += accelerate();

    cout << "\nFailed tests: " << fail_counter << ".\n";

    return 0;
}

// Should break due to Collision (hard)
int break_collision_hard()
{
    initialize_mpc_objects();
    double controls[NR_OF_CONTROLS];
    double observables[NR_OF_STATES] = {0, 0, 0, 0, 0, 2};

    predict(10, 10, controls[0], controls[1], controls[5], observables[0], observables[1], observables[5], controls, bu);

    if (controls[2] < 0.9 || controls[1] > 0.01)
    {
        printf("break_collision_hard failed");
        return 1;
    }

    return 0;
}

// Should break due to Collision (soft)
int break_collision_soft()
{
    initialize_mpc_objects();
    double controls[NR_OF_CONTROLS];
    double observables[NR_OF_STATES] = {0, 0, 0, 0, 0, 2};

    predict(10, 10, controls[0], controls[1], controls[5], observables[0], observables[1], observables[5], controls, bu);

    if (controls[2] < 0.1 || controls[1] > 0.01 || controls[2] > 0.9)
    {
        printf("break_collision_soft failed");
        return 1;
    }

    return 0;
}

// Should break due to curve
int break_curve()
{
    return 0;
}

// Should break due to destiantion approaching
int break_destination()
{
    return 0;
}

// Should turn left (destiation in front)
int left_destination_front()
{
    return 0;
}

// Should turn right (destination in front)
int right_destiation_front()
{
    return 0;
}

// Should turn left (destiation in back)
int left_destination_back()
{
    return 0;
}

// Should turn right (destination in back)
int right_destination_back()
{
    return 0;
}

// Should go straigt
int straight_ahead()
{
    return 0;
}

// Should accelerate
int accelerate()
{
    return 0;
}