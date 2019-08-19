// Copyright (C) 2006  Davis E. King (davis@dlib.net)
// License: Boost Software License   See LICENSE.txt for the full license.


#include <iostream>
#include <fstream>
#include "mpc_controller.cpp"
#include <dlib/string.h>


using namespace std;
using namespace dlib;

int main (int argc, char** argv)
{
    initialize_mpc_objects();
    double controls[3];
    predict(10, 10,
                0.0, 0.0, 0.0,
                0,0 , 2,
                controls);
    cout << "receivedd: " << controls[0] << ", " << controls[1] << ", " << controls[2];

    //matrix<double, NR_OF_STATES, 1> current_state;
    //current_state = 0.0, 0.0, 0.0, 0.0, 0.0, 4.0;
    //predict(mpc_object, 10, 10, false, current_state);
    //predict(mpc_object, 10, 10, false, current_state);

    return 0;

}

