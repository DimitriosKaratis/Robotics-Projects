# Robotics Projects – Door Opening with UR10

This repository contains the modeling and solution of robotics problems related to **trajectory planning and manipulator control**.  
The work was developed as part of the **Robotics course project** at the **Department of Electrical and Computer Engineering, Aristotle University of Thessaloniki**.

---

## 📌 Project Overview

The project is divided into two main parts, both focusing on robotic modeling and control for a **door-opening task using a UR10 robotic arm**.  
The implementation was carried out in **MATLAB** using the **Robotics Toolbox by Peter Corke**.

---

## 🔹 Part A: Door Handle Trajectory Planning

- Defined the kinematic frames for the room, the door, and the door handle.  
- Designed a trajectory for the **handle {H}** to rotate:
  - **−45° around its local x-axis** (simulating handle pressing).  
  - Then rotate the **entire door by −30°** around the zd axis while ensuring the relative orientation of the handle remains consistent.  
- The trajectory guarantees **zero initial and final velocity and acceleration**.  
- Door opening is completed in **T = 5 seconds**.  
- Implemented using:
  - **Homogeneous transformations**  
  - **Unit quaternions** for orientation interpolation  
  - Smooth position and orientation profiles  

---

## 🔹 Part B: UR10 Robot Arm Control

- The **UR10 6-DOF robotic arm** is used to follow the trajectory planned in Part A.  
- The robot starts from an initial configuration: *q0 = [−1.7752 −1.1823 0.9674 0.2149 1.3664 1.5708] rad*  
- The end-effector {e} rigidly grasps the door handle {H}, maintaining a constant relative pose throughout the motion.  
- The control task is to design **joint velocity commands (q̇r)** that:
  - Execute the handle trajectory.  
  - Open the door in **T = 5 seconds** with a sampling period of **ts = 0.01 sec**.  
  - Ensure **smooth joint positions and velocities**.  


**Useful MATLAB functions:**
- `ur10.fkine(q)` – Forward kinematics  
- `ur10.jacob0(q)` – Base frame Jacobian  
- `ur10.jacobe(q)` – End-effector Jacobian  
- `ur10.plot(qm)` – Visualizes UR10 trajectory  

---

## 🛠 Tools & Dependencies

- MATLAB (R2023 or later recommended)  
- Robotics Toolbox for MATLAB – [Peter Corke’s Toolbox](https://petercorke.com/toolboxes/robotics-toolbox/)  
