
#include <stdio.h>
#include <fstream>
#include <time.h>
#include "eigen3/Eigen/Dense"
#include "lpvDS.h"
#include "utils.h"

using namespace std;

int
main (int argc, char **argv)
{
    string path_model  = "/home/nbfigueroa/proj/catkin_ws_lags/src/lpvDS-lib/models/CShape-bottom-pqlf/";
    string path_dim    = path_model +  "dimensions";
    string path_Priors = path_model +  "Priors";
    string path_Mu     = path_model +  "Mu";
    string path_Sigma  = path_model +  "Sigma";
    string path_A      = path_model +  "A_k";

    /* Instantiate an LPV-DS class Option 1 */
    cout << "Initialization Test 1: " << endl;
    lpvDS lpvDS_test1(path_dim.c_str());
    lpvDS_test1.initialize_gamma(path_Priors.c_str(), path_Mu.c_str(), path_Sigma.c_str());
    lpvDS_test1.initialize_A(path_A.c_str());

    /* Instantiate an LPV-DS class Option 2 */
    cout << "Initialization Test 2: " << endl;
    lpvDS lpvDS_test2 (path_dim.c_str(), path_Priors.c_str(), path_Mu.c_str(), path_Sigma.c_str(), path_A.c_str());



    /* Testing the LPV-DS on training data from MATLAB */
    string path_att    = path_model +  "attractor";
    string path_data   = path_model +  "Data";
    string path_xi_dot = path_model +  "xi_dot";
    fileUtils fileUtils_;
    MatrixXd attractor, Data, xi_dot;
    attractor = fileUtils_.readMatrix(path_att.c_str());
    Data      = fileUtils_.readMatrix(path_data.c_str());
    xi_dot    = fileUtils_.readMatrix(path_xi_dot.c_str());


    return 0;
}
