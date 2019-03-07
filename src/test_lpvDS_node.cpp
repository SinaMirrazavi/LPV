/*
 * Copyright (C) 2018 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Author:  Nadia Figueroa
 * email:   nadia.figueroafernandez@epfl.ch
 * website: lasa.epfl.ch
 *
 * This work was supported by the EU project Cogimon H2020-ICT-23-2014.
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */


#include "ros/ros.h"
#include "lpvDS.h"
#include "utils.h"
#include "eigen3/Eigen/Dense"
#include <vector>


using namespace std;

double K;
double M;
vector<double> Priors;
vector<double> Mu;
vector<double> Sigma;
vector<double> A;
vector<double> attractor;
string path_model;

bool parseParams(const ros::NodeHandle& nh) {
    bool ret = true;


    if (!nh.getParam("model_path", path_model)) {
        ROS_ERROR("Couldn't retrieve model_path. ");
        ret = false;
    } else {
        cout << "Model path: "<< path_model << endl;
    }


    if (!nh.getParam("K", K))   {
        ROS_ERROR("Couldn't retrieve the number of guassians. ");
        ret = false;
    } else {
        cout << "Number of Components K: "<< (int)K << endl;
    }

    if (!nh.getParam("M", M))  {
        ROS_ERROR("Couldn't retrieve dimension of state. ");
        ret = false;
    } else {
        cout << "Dimensionality of state M: "<< (int)M << endl;
    }

    if (!nh.getParam("Priors", Priors))   {
        ROS_ERROR("Couldn't retrieve Priors. ");
        ret = false;
    } else {
        cout << "Priors: " << endl;
        for (int k = 0; k< int(K); k++)
            cout << Priors.at(k) << " ";
        cout << endl;
    }

    if (!nh.getParam("Mu", Mu))   {
        ROS_ERROR("Couldn't retrieve Mu. ");
        ret = false;
    } else {
        cout << "Mu: " << endl;
        for (int m = 0; m< int(K)*int(M); m++)
            cout << Mu.at(m) << " ";
        cout << endl;
    }

    if (!nh.getParam("Sigma", Sigma))  {
        ROS_ERROR("Couldn't retrieve Sigma. ");
        ret = false;
    } else {
        cout << "Sigma [0]: " << endl;
        for (int m = 0; m< int(M)*int(M); m++)
            cout << Sigma.at(m) << " ";
        cout << endl;
    }

    if (!nh.getParam("A", A))  {
        ROS_ERROR("Couldn't retrieve A. ");
        ret = false;
    } else {
        cout << "A [0]: " << endl;
        for (int m = 0; m< int(M)*int(M); m++)
            cout << A.at(m) << " ";
        cout << endl;
    }

    if (!nh.getParam("attractor", attractor))  {
        ROS_ERROR("Couldn't retrieve attractor. ");
        ret = false;
    } else {
        cout << "Priors: " << endl;
        for (int m = 0; m< int(M); m++)
            cout << attractor.at(m) << " ";
        cout << endl;
    }

    return ret;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_lpvDS_node");
    ros::NodeHandle nh;
    ros::NodeHandle _nh("~");


    if(!parseParams(_nh)) {
        ROS_ERROR("Errors while parsing arguments.");
        return 1;
    }

    /* Instantiate lpv-DS Model with parameters read from Yaml file*/
    lpvDS lpvDS_((int)K, (int)M, Priors, Mu, Sigma, A);

    /* Testing the LPV-DS on training data from MATLAB */
    string path_data   = path_model +  "Data";
    string path_xi_dot = path_model +  "xi_dot";
    fileUtils fileUtils_;
    MatrixXd Data, xi_dot;
    Data      = fileUtils_.readMatrix(path_data.c_str());
    xi_dot    = fileUtils_.readMatrix(path_xi_dot.c_str());
    int samples = Data.cols();

    cout << "Testing Accuracy of model..." << endl;

    /* Fill in attractor */
    VectorXd att; att.resize(M);
    for (int i = 0; i<(int)M; i++)
        att[i] = attractor.at(i);

    /* Fill in reference trajectories */
    MatrixXd xi_ref;  xi_ref.resize(M,samples);
    for (int i=0; i<M; i++)
        xi_ref.row(i) = Data.row(i);

    /* Compute estimated velocities from model */
    VectorXd xi_ref_test;  xi_ref_test.resize(M);
    VectorXd xi_dot_test;  xi_dot_test.resize(M);
    VectorXd xi_dot_mat;   xi_dot_mat.resize(M);
    VectorXd xi_dot_error;  xi_dot_error.resize(M);
    MatrixXd A_matrix; A_matrix.resize(M,M);
    VectorXd  est_error; est_error.resize(samples);
    for (int i=0; i<samples; i++){

        /* Computing desired velocity manually*/
        xi_ref_test = xi_ref.col(i);
        A_matrix = lpvDS_.compute_A(xi_ref_test);
        xi_dot_test = A_matrix*(xi_ref_test - att);

        /* Computing desired velocity directly*/
        xi_dot_test = lpvDS_.compute_f(xi_ref_test, att);

        /* Computing error between this estimate and MATLAB */
        xi_dot_mat = xi_dot.col(i);
        xi_dot_error =  xi_dot_test-xi_dot_mat;
        est_error[i] = xi_dot_error.norm();
    }

    /* Stats on Estimation error between MATLAB-C++ model */
    cout << "Average Estimation Error" << " (Norm of predicted Matlab and C++ velocities): " << est_error.mean() << endl;


    // Stop the node's resources
    ros::shutdown();
    // Exit tranquilly
    return 0;

}
