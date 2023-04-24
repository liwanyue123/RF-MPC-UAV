# RF-MPC-UAV

The RF-MPC algorithm, originally designed for quadruped robots(https://github.com/YanranDing/RF-MPC), is adapted for use with UAVs after some modifications.

 ![test 00_00_00-00_00_30](https://user-images.githubusercontent.com/35834577/234010625-42e95445-90b5-4c8d-a946-ba251f787805.gif)

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

