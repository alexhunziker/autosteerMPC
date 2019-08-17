// Copyright (C) 2006  Davis E. King (davis@dlib.net)
// License: Boost Software License   See LICENSE.txt for the full license.


#include <iostream>
#include <fstream>
#include "mpc_test.cpp"
#include "mpc_controller.cpp"
#include <dlib/string.h>


using namespace std;
using namespace dlib;

int main (int argc, char** argv)
{
    //main_mpc();
    mpc<NR_OF_STATES, NR_OF_CONTROLS, PREDICTION_HORIZON> mpc_object = initialize(4.1);

    matrix<double, NR_OF_STATES, 1> current_state;
    current_state = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    predict(mpc_object, 10, 10, false, current_state);

    return 0;

}

