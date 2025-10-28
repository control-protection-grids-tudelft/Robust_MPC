# Robust_MPC

**Date:** 23/10/2025
**Project:** Safe-Grid


---

## Overview
This is a repository concerning the development of a **robust model predictive control (RMPC)** solution to control output current of **modular multilevel converter (MMCs)** used in **high voltage direct current (HVDC)** transmission.
The RMPC is designed using a  tuning method, i.e., **controller matching with vertex relaxation**. 
Consult [1,2] for reading a detial theoretical description of the method.

> **Note:** This solution relates to the **Safe-Grid Project**.

---

## Structure
The solution is comprised of three main parts:

### 1. Build State-Space Realization of MMC Dynamics
- Run `getPlantModel()` function to obtain state-space (ss) matrices `(A,B,C)` corresponding to the continuous time.
- Run `c2d()` function to discretize the ss.
- Run `getAugmentedSystem()` function to obtain an augmented ss with an integrator.

---

### 2. Robust MPC Design via Controller Matching with Vertex Relaxation
- Run `getRobustK()` function to compute a robust state feedback gain, i.e., an unconstrained control law.
- Run `matchingControllerDesignRelax()` function to compute `Q, R, P` by solving LMI from `[ ]`, i.e., controller matching with vertex relaxation.
- Run `quadprog()` function to compute optimal control input resulting of the RMPC

---

### 3. Closed-Loop Simulation
Tools: **MATLAB / RTDS**
- Run `simMPCwRealPlant()` function to simulate the closed-loop response comprised of MMC and MPC.

---

## Highlights
- **Robustness:** Handles HVDC dynamics uncertainties and faults.
- **Integration:** MATLAB and RTDS for simulation.
- **Safe-Grid Alignment:** Designed for Safe-Grid project requirements.

``
