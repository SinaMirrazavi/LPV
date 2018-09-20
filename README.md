# LPV
[![Build Status](https://travis-ci.org/sinamr66/LPV.svg?branch=master)](https://travis-ci.org/sinamr66/LPV) [![DOI](https://zenodo.org/badge/97956089.svg)](https://zenodo.org/badge/latestdoi/97956089)

This package provides a C++ library for evaluation of Gaussian Mixture Model (GMM) based Linear Parameter Varying (LPV) systems; i.e. GMM-based LPV-DS which are have been used in introduced in [1,2,3].


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

**References**     
> [1] Mirrazavi Salehian, S. S., Khoramshahi, M. and Billard, A. (2016) A Dynamical System Approach for Catching Softly a Flying Object: Theory and Experiment. in IEEE Transactions on Robotics, vol. 32, no. 2, pp. 462-471, April 2016.
> [2] Mirrazavi Salehian, S. S. (2018) Compliant control of Uni/ Multi- robotic arms with dynamical systems. PhD Thesis
> [3] Figueroa, N. and Billard, A. (2018) A Physically-Consistent Bayesian Non-Parametric Mixture Model for Dynamical System Learning. In Proceedings of the 2nd Conference on Robot Learning (CoRL). Accepted.     
