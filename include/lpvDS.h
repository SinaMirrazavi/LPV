/*
 * Copyright (C) 2016 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Author: Sina Mirrazavi
 * email:   sina.mirrazavi@epfl.ch
 * website: lasa.epfl.ch
 *
 * Later modified by Nadia Figueroa
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

#include <stdlib.h>
#include <string>
#include "eigen3/Eigen/Dense"
#include "ros/ros.h"
#include "utils.h"

using namespace std;
using namespace Eigen;

const double PI = 3.14159265358979323846264338327950288419716939937510;

class lpvDS
{	
private:

        int 		K_;
        int 		M_;
        VectorXd	gamma_;
		double 		*Prior_;
		VectorXd 	*Mu_;
		MatrixXd 	*Sigma_;
		MatrixXd 	*A_Matrix_;

public:             

        lpvDS(const char  *path_dims);
        lpvDS(const char  *path_dims, const char  *path_prior,const char  *path_mu,const char  *path_sigma, const char  *path_A);
        ~lpvDS(void);

        void        initialize_A(const char  *path_A);
        void        initialize_gamma(const char  *path_prior, const char  *path_mu, const char  *path_sigma);

        MatrixXd    compute_A(VectorXd xi);
        VectorXd    compute_gamma(VectorXd xi);
        VectorXd    compute_f(VectorXd xi, VectorXd xi_att);

private:
        void        initialize_params();
        double 		GaussianPDF(VectorXd x,VectorXd Mu,MatrixXd Sigma);
        fileUtils   fileUtils_;
        void 	 	ERROR();
};





