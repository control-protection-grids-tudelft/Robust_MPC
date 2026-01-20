# Robust_MPC

**Date:** 23/10/2025
**Project:** 


---

## Overview
This repository provides a collection of toolboxes for **modeling, analysis, and controller synthesis** of modular multilevel converter (**MMC**)–based multi-terminal direct current (**MTDC**) power systems.

The toolboxes are developed in **MATLAB and Julia**, and the resulting control solutions can be deployed in RTDS-based hardware-in-the-loop (HIL) simulations. Controller synthesis primarily relies on **robust control techniques**, with a focus on ensuring stability and performance under modeling uncertainties and disturbances.

The implemented system models mainly capture the average-value dynamics of MMCs formulated in the dqz reference frame, enabling efficient analysis and controller design for large-scale MTDC systems.

> **Note:** This solution relates to the ****.

---

## Structure
The solution is comprised of three main parts:

### 1. Build State-Space Realization of MMC Dynamics
- Run `getPlantModel()` function to obtain state-space (ss) matrices `(A,B,C)` corresponding to the continuous time.
- Run `c2d()` function to discretize the ss.
- Run `getAugmentedSystem()` function to obtain an augmented ss with an integrator defined by rated dynamics `\Delta u(k) = u(k)-u(k-1)`.

---

### 2. Robust MPC Design via Controller Matching with Vertex Relaxation
- Run `getRobustK()` function to compute a robust state feedback gain `K`, i.e., an unconstrained control law.
- Run `matchingControllerDesignRelax()` function to compute `Q, R, P` by solving LMI from `[ ]`, i.e., controller matching with vertex relaxation.
- Run `quadprog()` function to compute optimal control input resulting of the RMPC

---

### 3. Closed-Loop Simulation
Tools: **MATLAB / RTDS**
- Run `simMPCwRealPlant()` function to simulate the closed-loop response comprised of MMC and MPC.
- Run `simSystemSateFeedBack()` function to simulate the closed-loop response comprised of MMC and robust state feedback gain `K`.

---

## Highlights
- **Robustness:** Handles HVDC dynamics uncertainties and faults.
- **Integration:** MATLAB and RTDS for simulation.
- **Safe-Grid Alignment:** Designed for Safe-Grid project requirements.


