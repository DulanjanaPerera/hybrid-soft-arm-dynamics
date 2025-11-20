# Hybrid Soft Continuum Arm Dynamics (Modal Space Formulation)

This repository contains MATLAB code to simulate the dynamics of a hybrid soft–rigid continuum arm using a modal joint–space formulation. The dynamic model is derived for a soft module actuated by three pneumatic muscle actuators (PMAs), and the current implementation uses the modal coordinates corresponding to PMA 2 and PMA 3.

The code implements the inertia, Coriolis/centrifugal, gravitational, and stiffness terms symbolically and provides a script to run and visualize the arm dynamics.

## Repository Structure

matlab/
  runDynamicSimulation.m   % Main script to run the dynamic simulation and visualization
  arm_dynamics_MSF.m      % ODE right-hand side for the modal dynamic model
  M_jointspace_MSF.m      % Inertia matrix in joint space (modal form)
  C_jointspace_MSF.m      % Coriolis-centrifugal matrix in joint space
  G_and_K_jointspace_MSF.m% Gravity + stiffness generalized forces in joint space
  backbonePos_xi_MSF.m    % Backbone position along arc-length parameter xi

docs/
  Dynamics for biomimetic continuum arms A Modal Approach.pdf
  Hybrid Soft Robots Incorporating Stiff and Hard Elements.pdf
  Novel Variable Stiffness Soft Robotic Gripper.pdf



