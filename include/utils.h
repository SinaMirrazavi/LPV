#ifndef UTILS_H
#define UTILS_H

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string>
#include "eigen3/Eigen/Dense"

using namespace std;
using namespace Eigen;

#define MAXBUFSIZE  ((int) 1e5)
class fileUtils
{
 private:
    double  buff[MAXBUFSIZE];

 public:
    bool is_file_exist(const char *fileName);
    MatrixXd readMatrix(const char *filename);

};

#endif // UTILS_H


