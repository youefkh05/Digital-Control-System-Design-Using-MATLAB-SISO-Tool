# Digital Control System Design Using MATLAB SISO Tool

This repository contains a complete MATLAB workflow for designing a digital compensator for a sampled-data control system using **SISO Design Tool**, **c2d**, and **d2c**, as required in the 2025 Control Engineering project. The objective is to achieve a **steady-state error of 10%** to a unit-step input and a **phase margin of at least 50°**.

---

## 🚀 Project Overview

We consider a continuous-time control loop with:

- Plant:  
  \[
  G_p(s) = \frac{4}{(2s+1)(0.5s+1)}
  \]

- Sensor/Feedback:  
  \[
  H(s) = \frac{1}{0.05s + 1}
  \]

- Sample Time:  
  \[
  T = 0.1 \text{ seconds}
  \]

The project workflow:

1. Build the continuous-time open-loop system.  
2. Calculate the required DC gain to achieve **10% steady-state error**.  
3. Discretize the system using **ZOH (Zero-Order Hold)** and `c2d`.  
4. Launch **SISO Tool** for discrete compensator design.  
5. Tune the controller until:  
   - \( e_{ss} = 10\% \)  
   - Phase margin ≥ 50°  
6. Export the designed controller to MATLAB.  
7. Validate step response and phase margin.  
8. Optionally convert back to continuous domain using `d2c`.

---

## 📁 Repository Structure

