# RF-MPC-UAV

The RF-MPC algorithm, originally designed for quadruped robots (https://github.com/YanranDing/RF-MPC), is adapted for use with UAVs after some modifications.

<p align="center">
  <img src="https://user-images.githubusercontent.com/35834577/234013953-5487c7bc-f7a1-479f-8b1a-46be44341fa2.gif" width="800" />
</p>

I have provided a simple trajectory without considering the quadrotor UAV's dynamic constraints and optimization. You can run `test_traj.m` to see the results.
<p align="center">
  <img src="https://user-images.githubusercontent.com/35834577/234013478-a5326d28-6c1a-41d7-b705-7019dcb4eb81.PNG" width="400" />
</p>

## Requirement

**Basic**: MATLAB and MATLAB optimization toolbox

**Optional**: qpSWIFT (can be obtained from https://github.com/qpSWIFT)

## Installation

There is no need to install external packages.

## Usage

Navigate to the root directory and run the `MAIN_UAV.m` function

### MAIN

#### The Plant

The robot is modeled as a single rigid body (SRB). The SRB dynamics is defined in `...\fcns\dynamics_SRB_UAV.m`

#### VBL and vectorization

The code for variation-based linearization and vectorization steps is in `...\fcns_MPC\fcn_get_ABD_eta_UAV.m`

#### Quadratic Program (QP)

The code for QP formulation is in `...\fcns_MPC\fcn_get_QP_form_eta_UAV.m`

The QP could be solved by either the MATLAB QP solver `quadprog` or an efficient QP solver qpSWIFT (coming soon!)


## References

This code is based on the following paper:

- Yanran Ding, Abhishek Pandala, Chuanzheng Li, Young-Ha Shin, Hae-Won Park "Representation-Free Model Predictive Control for Dynamic Motions in Quadrupeds". In IEEE Transactions on Robotics. [PDF](Link)


## Acknowledgments

Thanks to Yanran Ding

