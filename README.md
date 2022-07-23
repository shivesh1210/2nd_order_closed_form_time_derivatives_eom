# Closed Form Time Derivatives of the Equations of Motions of Rigid Body Systems

This repository contains the MATLAB code for computing 2nd order analytical time derivatives of Equations of Motions of multi-body systems in closed form using body fixed representation of the twists. This repository accompanies the submission: Andreas Mueller, Shivesh Kumar, Closed Form Time Derivatives of the Equations of Motions of Rigid Body Systems. In: Springer Multibody System Dynamics 2021 (under review). 

# Usage instructions:

# Example scripts
* Panda_InvDyn_BodyFixed_ClosedForm.m: 2nd order time derivatives in closed form which uses robot parameters from Franka Emika Panda robot
* Kuka_InvDyn_BodyFixed_ClosedForm.m: 2nd order time derivatives in closed form which uses robot parameters from KUKA IIWA LBR robot
* RR_symbolic.m: Generate symbolic equations for 2nd order time derivatives using the closed form expressions for the planar 2R robot

# Main script
* ClosedFormInvDyn_BodyFixed.m: Function to compute 2nd order inverse dynamics in closed form - works in both numerical and symbolic modes

# Helper functions
* SE3Exp.m: Function to compute exponential mapping for SE(3)
* SO3Exp.m: Function to compute exponential mapping for SO(3)
* SE3Inv.m: Function to compute analytical inverse of exponential mapping for SE(3)
* SE3AdjMatrix.m: Function to compute (6x6) Adjoint Matrix for SE(3)
* SE3adjMatrix.m: Function to compute (6x6) adjoint Matrix for SE(3) - also known as spatial cross product in the literature
* SE3AdjInvMatrix.m: Function to compute Inverse of (6x6) Adjoint Matrix for SE(3)
* MassMatrixMixedData.m: Function to build mass-inertia matrix in SE(3) from mass, inertia and center of mass information
* InertiaMatrix.m: Function to build rotational inertia matrix from minimal set of parameters
* ScrewCoordinatesIFR.m: Function to compute the screw coordinates vector in the inertial frame of reference (IFR) from 3D axis vector and origin vector

# Citation
Müller, A., Kumar, S. Closed-form time derivatives of the equations of motion of rigid body systems. Multibody Syst Dyn 53, 257–273 (2021). https://doi.org/10.1007/s11044-021-09796-8
```
@article{mueller2021closed,
  title={Closed-form time derivatives of the equations of motion of rigid body systems},
  author={Mueller, Andreas and Kumar, Shivesh},
  journal={Multibody System Dynamics},
  volume={53},
  number={3},
  pages={257--273},
  year={2021},
  publisher={Springer Netherlands}
}
```
