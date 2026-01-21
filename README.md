# LeRobot Humanoid Models

This repository contains the URDF models for the **LeRobot humanoid project**, along with a few minimal scripts to check URDF sanity and load the models in simulation.

These models are intended as an **early-stage research platform** for humanoid robotics, with a strong focus on rapid iteration and real-world validation.

---

## ⚠️ Current Status (Early Prototype)

Please note that this is a **very early version** of the models:

- The default joint configuration of the URDF is currently **not symmetric**.  
  (Do not expect clean or mirrored values yet.)

- The **mass distribution of the upper body is not yet realistic** and will be updated in future versions.

- Geometry, inertias, and kinematic structure are still evolving as the hardware design iterates.

Despite this, the models are already usable for:
- kinematics,
- basic dynamics,
- control and locomotion prototyping in simulation.


---

### CAD Model (Onshape)

The corresponding CAD model of the humanoid robot is available on Onshape:

https://cad.onshape.com/documents/fb645318a27646d1d8840be6/w/d1cae8805fb652b4d1614997/e/70edb54994a0fe82dd64a212

This CAD model reflects the current hardware design iteration and is provided to support:
- mechanical inspection,
- dimensioning,
- hardware replication,
- and future co-design work.

Please note that the CAD and URDF may temporarily diverge as both evolve during rapid prototyping.
---

## Included Models

This repository currently provides two URDF variants:

### 1) `urdf/bipedal_platform`

- A **12-DoF** URDF model of a bipedal robot  
- The upper body is represented using fixed joints  
- Intended for early locomotion experiments  

A short video of this platform walking is included:  
**`walk.mp4`**

---

### 2) `urdf/lerobot_humanoid`

- A **20-DoF** URDF model including:
  - legs  
  - arms  

- Intended as the first full-body humanoid prototype

---

## Reference Frames

Both URDFs define the following standard frames:

### Feet
- `foot_left`  
- `foot_right`

### Lower torso
- `torso`

### Hands
- `hand_left`  
- `hand_right`

These frames are meant to be used as contact points and end-effectors for control and planning.

---

## Example Code

This repository includes minimal example scripts to:

- load the URDF,  
- check basic model consistency,  
- visualize the robot.

These scripts are only meant as **sanity checks** and lightweight usage examples.

---

## Installation (for testing the example code)

### Requirements

The required Python environment is described in `environment.lock`.

In addition, the following external dependencies are needed:

- A fork of **Crocoddyl**  
  https://github.com/LudovicDeMatteis/crocoddyl/tree/topic/contact-6D-closed-loop

- A fork of **Sobec**  
  https://github.com/LudovicDeMatteis/sobec/tree/iros-2025

- **meshcat** (for visualization)

---
