#ifndef GMM_H
#define GMM_H

#include <stdlib.h>
#include <string>
#include "eigen3/Eigen/Dense"
#include "ros/ros.h"
#include "utils.h"


using namespace std;
using namespace Eigen;

const double PI = 3.14159265358979323846264338327950288419716939937510;

class GMM
{
    private:

        int 		K_;
        int 		M_;
        double 		*Prior_;
        VectorXd 	*Mu_;
        MatrixXd 	*Sigma_;
        double      threshold;

    public:

        void        initialize(int Num_Com,int Num_state);
        void        initialize_GMM(const char  *path_prior_,const char  *path_mu_,const char  *path_sigma_,const char *path_threshold);
        double      PDF(VectorXd X);

    private:

        double 		GaussianPDF(VectorXd x,VectorXd Mu,MatrixXd Sigma);
        fileUtils   fileUtils_;
        void 	 	ERROR();

};

#endif // GMM_H
