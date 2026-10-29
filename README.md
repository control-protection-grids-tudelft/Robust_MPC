# Robust_MPC

**Date:** 23/10/2025



## Overview

This repository contains a collection of functions for the modeling, analysis, and control of Modular Multilevel Converters (MMCs) and Multi-Terminal Direct Current (MTDC) transmission systems.

The main contribution of this repository is a framework for the commissioning and validation of robust state-feedback controllers [1] and model predictive controllers (MPCs) [2] for MMC-based MTDC systems. In addition, it includes well-established methods for stability analysis and robust state-feedback control design.

The presented solutions have been developed in MATLAB and Julia. The resulting controllers can be deployed on RTDS platforms using CBuilder blocks.

---

## Structure
The solution is comprised of three main parts:

### 1. Build State-Space Realization of MMC Dynamics
- Run `getPlantModel()` function to obtain state-space (ss) matrices `(A,B,C)` corresponding to the continuous time.
- Run `c2d()` function to discretize the ss.
- Run `getAugmentedSystem()` function to obtain an augmented ss with an integrator defined by rated dynamics `\Delta u(k) = u(k)-u(k-1)`.


### 2. Robust MPC Design via Controller Matching with Vertex Relaxation
- Run `getRobustK()` function to compute a robust state feedback gain `K`, i.e., an unconstrained control law.
- Run `matchingControllerDesignRelax()` function to compute `Q, R, P` by solving LMI from `[ ]`, i.e., controller matching with vertex relaxation.
- Run `quadprog()` function to compute optimal control input resulting of the RMPC


### 3. Closed-Loop Simulation
Tools: **MATLAB / RTDS**
- Run `simMPCwRealPlant()` function to simulate the closed-loop response comprised of MMC and MPC.
- Run `simSystemSateFeedBack()` function to simulate the closed-loop response comprised of MMC and robust state feedback gain `K`.

---

## Highlights
- **Robustness:** Handles HVDC dynamics uncertainties and faults.
- **Integration:** MATLAB and RTDS for simulation.
- **SAFE-GRID Alignment:** Designed for SAFE-GRID project requirements.

---
## References

[1] V. Reyes Dreke, R. Rane and A. Lekić, *"Robust Current Regulation of MMC-based MTDC Power Systems based on Lyapunov Inequality,"* 2026 International Conference on Smart Energy Systems and Technologies (SEST), 2026.

[2] V. Reyes Dreke, R. Rane and A. Lekić, *"Robust Tuning of Model Predictive Control for MMC-Based High-Voltage Power Systems,"* arxiv, 2026.

## Funding

This work is supported by the Horizon Europe project **PROSECCO** (Grant Agreement No. 101160687) and the **NWO Veni** project **SAFE-GRID** (Project No. 20248)


