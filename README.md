# LPV
[![Build Status](https://travis-ci.org/sinamr66/LPV.svg?branch=master)](https://travis-ci.org/sinamr66/LPV)

This package provides a c++ library for Gaussian Mixtire Model (GMM) based Linear Parameter Varying (LPV) systems . 


# Dependences 

- Eigen http://eigen.tuxfamily.org/index.php?title=Main_Page


# Features:

Implementation of GMM and LPV!




# How to run LPV

By initializing the LPV system, a GMM is aoutomatically constracted.

### Initialization:
```
LPV.initialize(int Num_Com,int Num_state);
```
Num_Com is the number of the components and 
Num_state is the dimension of the system
```
LPV.initialize_A(const char *path_);
LPV.initialize_theta(const char *path_prior_,const char *path_mu_,const char *path_sigma_);
```

### In the loop:
```
LPV.Calculate_A(VectorXd X)
```

# How to run GMM (without LPV!)

By initializing the LPV system, a GMM is aoutomatically constracted.

### Initialization:
```
GMM.initialize(int Num_Com,int Num_state);
```
Num_Com is the number of the components and 
Num_state is the dimension of the system
```
GMM.initialize_GMM(const char  *path_prior_,const char  *path_mu_,const char  *path_sigma_,const char *path_threshold);
```

### In the loop:
```
GMM.PDF(VectorXd X)
```
For more information contact Sina Mirrazavi. 
## Copyright
Please cite these papers if you are using this toolbox:

@ARTICLE{7439839,
author={S. S. M. Salehian and M. Khoramshahi and A. Billard},
journal={IEEE Transactions on Robotics},
title={A Dynamical System Approach for Softly Catching a Flying Object: Theory and Experiment},
year={2016},
volume={32},
number={2},
pages={462-471},
ISSN={1552-3098},
month={April},}
