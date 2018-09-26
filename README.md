# lpvDS-lib
This package provides a C++ library for execution of Gaussian Mixture Model (GMM) based Linear Parameter Varying (LPV) Dynamical Systems; i.e. GMM-based LPV-DS which have been used and introduced in [1,2,3]. 

This version of the LPV library focuses on the formulation proposed in [3] where a non-linear DS formulated as:
<p align="center">
<img src="https://github.com/nbfigueroa/LPV/blob/nadia/img/f_x.gif"></>
  
is learned from demonstrations in a decoupled manner. Where the GMM parameters <img src="https://github.com/nbfigueroa/LPV/blob/nadia/img/theta_gamma.gif"> used to parametrize <img src="https://github.com/nbfigueroa/LPV/blob/nadia/img/gamma.gif"> are estimated via the physically-consistent GMM approach proposed in [3] and provided in [phys-gmm](https://github.com/nbfigueroa/phys-gmm) and the DS parameters <img src="https://github.com/nbfigueroa/LPV/blob/nadia/img/DS_params.gif"> are estimated by solving a semi-definite optimization problem that ensures global asymptotic stability of the system via constraints derived from either a:
- QLF (Quadratic Lyapunov Function): <img src="https://github.com/nbfigueroa/LPV/blob/nadia/img/stab_qlf.gif">
- P-QLF(Parametrized QLF):  <img src="https://github.com/nbfigueroa/LPV/blob/nadia/img/stab_pqlf.gif">  

The whole learning scheme is provided in [ds-opt](https://github.com/nbfigueroa/ds-opt). This formulation and learning approach enables the accurate encoding of highly non-linear, non-monotic trajectories, as the ones below:

<p align="center">
<img src="https://github.com/nbfigueroa/LPV/blob/nadia/img/3D-CShape-bottom_lpvO3.png"  width="350"><img src="https://github.com/nbfigueroa/LPV/blob/nadia/img/3D-Sink_lpvO3.png"  width="350"></>


while ensuring global asymptotic stability. To learn DS with this formulation and with any of the two Lyapunov constraints defined above go to the [ds-opt](https://github.com/nbfigueroa/ds-opt) package. 

**Note**: This package supports inputs from [eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) and [mathlib](https://github.com/epfl-lasa/mathlib) libraries.

### Installation (Catkin package)
By following these instructions, we assume you have a ROS environment and use catkin workspace to compile code. 
Clone this respository in your ```./src``` folder and make sure that you have both the [eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) and [mathlib](https://github.com/epfl-lasa/mathlib) libraries installed. Then, compile the package
```
$ cd ~/catkin_ws/
$ catkin_make
```

## Usage
First and foremost, one must have a GMM-based LPV-DS model, which requires the following parameters:
- GMM parameters: ``Priors``, ``Mu``,``Sigma``
- DS parameters:  ``A_k``,``b_k``

The ``lpvDS`` class can read these parameters in different formats:
- Text files: A text file of each parameter is needed.; i.e. ``Priors.txt``,``Mu.txt``,``Sigma.txt``, ``A_k.txt``, ``b_k.txt``,``attractor.txt``, ``dimensions.txt``, where the last file should contain 2 numbers ``[K M]`` indicating the number of Gaussian components ``K`` and the dimensionality of the states ``M``.
- Yaml file: A single yaml file with all the parameters mentioned above in vector format.

Examples of these files are provided in the ``models/`` folder. To generate these files follow the ``demo_learn_lpvDS.m`` script in the [ds-opt](https://github.com/nbfigueroa/ds-opt) package. Once you have your lpv-DS model, you can either initialize an instance of the lpv-DS class as follows:

- For text files, you have multiple initialization options:
```C++
  /* Instantiate an LPV-DS class Option 1 */
  lpvDS lpvDS_(path_dim.c_str());
  lpvDS_.initialize_gamma(path_Priors.c_str(), path_Mu.c_str(), path_Sigma.c_str());
  lpvDS_.initialize_A(path_A.c_str());
  
  /* Instantiate an LPV-DS class Option 2 */
  lpvDS lpvDS_ (path_dim.c_str(), path_Priors.c_str(), path_Mu.c_str(), path_Sigma.c_str(), path_A.c_str());
```
Or you can read the parameter files using the ``fileUtils`` class and initialize an lpvDS class instance as follows:
```C++
   /* Instantiate an LPV-DS class Option 3 */
   fileUtils fileUtils_;
   MatrixXd dim, Priors, Mu, Sigma, A;
   dim     = fileUtils_.readMatrix(path_dim.c_str());
   Priors  = fileUtils_.readMatrix(path_Priors.c_str());
   Mu      = fileUtils_.readMatrix(path_Mu.c_str());
   Sigma   = fileUtils_.readMatrix(path_Sigma.c_str());
   A       = fileUtils_.readMatrix(path_A.c_str());
   int K = (int)dim(0,0);
   int M = (int)dim(1,0);
   lpvDS lpvDS_ (K, M, Priors, Mu, Sigma, A);
```
Where ``K`` is the number of the Gaussian components and ``M`` is the dimension of the system.

- **For Yaml file [Recommended]:** We use the ROS parameter server which reads a yaml file containing each of the parameters mentioned above in  ``vector<double>`` format from the C++ standard library.

```C++
vector<double> Priors = ...; /* Vector of Priors */
vector<double> Mu = ...;    /* Mu as a long vector*/
vector<double> Sigma = ...; /* Sigma as a long vector*/
vector<double> A = ...;     /* A as a long vector*/
int K = ...;                /* Number of components */
int M = ...;                /* Dimensionality of the state */
lpvDS lpvDS_(K,M, Priors, Mu, Sigma, A)
```
An example of such yaml file can be found in ```models/```. Also, to generate your own yaml files you can  follow the ``demo_learn_lpvDS.m`` script in the [ds-opt](https://github.com/nbfigueroa/ds-opt) package, specifically, the function
```Matlab
function save_lpvDS_to_Yaml(DS_name, pkg_dir,  ds_gmm, A_k, att, x0_all, dt)
```

### In the loop:
Once you have the lpvDS class instantiated and initialized in any of the available formats, you can use it in the loop as follows:
- For ``Eigen::VectorXd`` type inputs:

```C++
VectorXd att = ...;                        /* attractor */
VectorXd xi  = ...;                        /* current state */
Vextor Xd xi_dot;                          /* desired velocity */

 /* Option 1: Computing desired velocity manually*/
MatrixXd  A_matrix = lpv_DS_.compute_A(xi) /* computing the weight sum of A matrices */
xi_dot = A_matrix*(xi - att);              /* computing the desired velocity */

/* Option 2: Computing desired velocity directly*/
xi_dot = lpvDS_.compute_f(xi_ref_test, att);
```
- For ``Mathlib::vector`` type inputs:


This is a minimalistic code, proper resizing of vectors and matrices must be implemented.

---
### Testing the class with txt files
You can try the testing script as well, this was generated by compiling the library,  type the following:
```
$ rosrun lpvDS_lib test_lpvDS
```
To run it correctly, you must modify the ```path_model``` variable:
```C++
string path_model  = "/home/nbfigueroa/proj/catkin_ws_lags/src/lpvDS-lib/models/CShape-bottom-pqlf/";
```
This test will load a stored lpv-DS model (the one that generates the C-Shape above) and the demonstrations that were used to train it. It instantiates an lpv-DS model with the three different ways shown above for text files and checks for numerical errors in the estimated velocities, you should see the following:
```
Initialization Test 1: 
Initialized an M:3 dimensional GMM-based LPV-DS with K: 7 Components
...
Initialization Test 2: 
Initialized an M:3 dimensional GMM-based LPV-DS with K: 7 Components
...
Initialization Test 2: 
Initialized an M:3 dimensional GMM-based LPV-DS with K: 7 Components
** Initializing A's' **
** Initializing Priors **
** Initializing Mu **
** Initializing Sigma **
Testing Accuracy of model...
...
Average Estimation Error (Norm of predicted Matlab and C++ velocities): 0.00401846
```

### [Recommended] Testing the class with yaml file! 
To load parameters from a yaml file into the ROS parameter server, one must create a node that reads such yaml file. An example of such node is provided in ```src/test_lpvDS_node.cpp``` and can be run with the following launch file:
```
$ roslaunch lpvDS run_testLPVDS_node.launch
```
This node will do the same as the previous testing script. The path to the testing model is already provided in the launch file. Hence, by running the above command you should see the following:
```
Model path: /home/nbfigueroa/proj/catkin_ws_lags/src/lpvDS-lib/models/CShape-bottom-pqlf/
Number of Components K: 7
Dimensionality of state M: 3
Priors: 
0.0480226 0.0635593 0.0786252 0.0376648 0.0550847 0.118644 0.0983992 
Mu: 
-0.699769 0.245762 0.0777207 -0.434051 0.217584 0.343617 -0.608964 -0.00479354 0.0474029 -0.478294 0.31798 0.259048 -0.444281 0.141656 0.160792 -0.681747 0.0744 0.379855 -0.553359 0.143088 0.402569 
Sigma [0]: 
0.00597166 0.00124721 0.00131495 0.00124721 0.00428135 0.000868572 0.00131495 0.000868572 0.003528 
A [0]: 
0.249952 -0.191665 -0.241171 1.34761 -0.0730141 -0.353369 0.301089 -0.330761 -0.391286 
Priors: 
-0.689617 0.0712412 0.377028 
Initialized an M:3 dimensional GMM-based LPV-DS with K: 7 Components
** Initializing Priors **
** Initializing Mu **
** Initializing Sigma **
** Initializing A **
[File Dimensionality] row 6 2215
[File Dimensionality] row 3 2215
Testing Accuracy of model...
Average Estimation Error (Norm of predicted Matlab and C++ velocities): 0.00402145
```



**References**     
> [1] Mirrazavi Salehian, S. S., Khoramshahi, M. and Billard, A. (2016) A Dynamical System Approach for Catching Softly a Flying Object: Theory and Experiment. in IEEE Transactions on Robotics, vol. 32, no. 2, pp. 462-471, April 2016.  
> [2] Mirrazavi Salehian, S. S. (2018) Compliant control of Uni/ Multi- robotic arms with dynamical systems. PhD Thesis  
> [3] Figueroa, N. and Billard, A. (2018) A Physically-Consistent Bayesian Non-Parametric Mixture Model for Dynamical System Learning. In Proceedings of the 2nd Conference on Robot Learning (CoRL). Accepted.     

This package was initially implemented by [Sina Mirrazavi](http://lasa.epfl.ch/people/member.php?SCIPER=233855) and has been extended and modified by [Nadia Figueroa](http://lasa.epfl.ch/people/member.php?SCIPER=238387).  

**Contact**: [Nadia Figueroa](http://lasa.epfl.ch/people/member.php?SCIPER=238387) (nadia.figueroafernandez AT epfl dot ch)
