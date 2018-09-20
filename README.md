# lpvDS-lib
This package provides a C++ library for evaluation of Gaussian Mixture Model (GMM) based Linear Parameter Varying (LPV) Dynamical Systems; i.e. GMM-based LPV-DS which have been used and introduced in [1,2,3]. 

This version of the LPV library focuses on the formulation proposed in [3] where a non-linear DS formulated as:
<p align="center">
<img src="https://github.com/nbfigueroa/LPV/blob/nadia/img/f_x.gif"></>
  
is learned from demonstrations while ensuring global asymptotic stability derived from either a:
- QLF (Quadratic Lyapunov Function): <img src="https://github.com/nbfigueroa/LPV/blob/nadia/img/stab_qlf.gif">
- P-QLF(Parametrized QLF):  <img src="https://github.com/nbfigueroa/LPV/blob/nadia/img/stab_pqlf.gif">

Such formulation and learning approach, proposed in [3], enables the accurate encoding of highly non-linear, non-monotic trajectories, as the ones below:

<p align="center">
<img src="https://github.com/nbfigueroa/LPV/blob/nadia/img/3D-CShape-bottom_lpvO3.png"  width="350"><img src="https://github.com/nbfigueroa/LPV/blob/nadia/img/3D-Sink_lpvO3.png"  width="350"></>


while ensuring global asymptotic stability. To learn DS with this formulation and with any of the two Lyapunov constraints defined above go to the [ds-opt](https://github.com/nbfigueroa/ds-opt) package.

### Installation (Catkin package)
By following these instructions, we assume you have a ROS environment and use catkin workspace to compile code. 
Clone this respository in your ```./src``` folder and make sure that you have the [eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) library. Then, compile the package
```
$ cd ~/catkin_ws/
$ catkin_make
```
## Usage
First and foremost, one must have a GMM-based LPV-DS model, which requires the following parameters:
- GMM parameters: ``Priors``, ``Mu``,``Sigma``
- DS parameters:  ``A_k``,``b_k``

The ``lpvDS`` class can read these parameters in different formats:
- Text files: A text file of each parameter is needed.
- Yaml file: A single yaml file with all the parameters in single vector format.

Examples of these files are provided in the ``models/`` folder. To generate these files follow the ``demo_learn_lpvDS.m`` script in the [ds-opt](https://github.com/nbfigueroa/ds-opt) package. Once you have your lpv-DS model, you can either initialize an instance of the lpv-DS class as follows:

- For text file:
```
lpvDS.initialize(int K,int M);
lpvDS.initialize_A(const char *path_A_);
lpvDS.initialize_b(const char *path_b_);
lpvDS.initialize_gamma(const char *path_prior_,const char *path_mu_,const char *path_sigma_);
```
Where ``K`` is the number of the Gaussian components and ``M`` is the dimension of the system.

- For Yaml file: We assume that a Yaml file has been read via the ROS parameter server and each parameters is of ``std::vector<double>`` format

```
lpvDS.initialize(int K,int M);
lpvDS.initialize_A(std::vector<double> *A_);
lpvDS.initialize_b(std::vector<double> *b_);
lpvDS.initialize_gamma(std::vector<double> *Priors_, std::vector<double> *Mu_ ,std::vector<double> *Sigma_);
```

... continue modifying!
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

---
### Testing the class
You can try the testing script as well, this was generated by compiling the library,  type the following:
```
rosrun lpvDS_lib test_lpvDS
```
This test will load a stored lpv-DS model (the one that generates the C-Shape above) and the demonstrations that were used to traine it. It will check for numerical errors in the estimated velocities, you should see the following:
```
bla bla bla..
SVM Model: /home/nbfigueroa/dev/catkin_ws/src/SVMGrad/matlab/models/2d-example2-svm.txt
model.D: 2
model.nSV: 40
model.b: -0.962694
model.sigma: 0.1

SVM Testing File: /home/nbfigueroa/dev/catkin_ws/src/SVMGrad/matlab/models/2d-example2-data.txt
```

**References**     
> [1] Mirrazavi Salehian, S. S., Khoramshahi, M. and Billard, A. (2016) A Dynamical System Approach for Catching Softly a Flying Object: Theory and Experiment. in IEEE Transactions on Robotics, vol. 32, no. 2, pp. 462-471, April 2016.
> [2] Mirrazavi Salehian, S. S. (2018) Compliant control of Uni/ Multi- robotic arms with dynamical systems. PhD Thesis
> [3] Figueroa, N. and Billard, A. (2018) A Physically-Consistent Bayesian Non-Parametric Mixture Model for Dynamical System Learning. In Proceedings of the 2nd Conference on Robot Learning (CoRL). Accepted.     

This package was initially implemented by [Sina Mirrazavi](http://lasa.epfl.ch/people/member.php?SCIPER=233855) and has been extended and modified by [Nadia Figueroa](http://lasa.epfl.ch/people/member.php?SCIPER=238387).  

**Contact**: [Nadia Figueroa](http://lasa.epfl.ch/people/member.php?SCIPER=238387) (nadia.figueroafernandez AT epfl dot ch)
