# A-star + APF Hybrid Path Planning for AUVs in Complex Underwater Environments

This repository contains the MATLAB simulation code for the paper:

> *"[Path planning for underwater robots by integrating an improved A-star algorithm with the APF algorithm]", submitted to Ships and Offshore Structures.*

The code implements a hybrid path planning algorithm for Autonomous Underwater Vehicles (AUVs) that integrates an **improved A-star algorithm** with an **enhanced Artificial Potential Field (APF)** method, incorporating AUV kinematic constraints and dynamic obstacle avoidance capabilities.

---

## 📌 Overview

This work addresses key challenges in underwater robot path planning, including:

- Low planning efficiency and long computation times
- Local minima trapping in traditional APF methods
- Poor dynamic obstacle avoidance performance
- Unsmooth and kinematically infeasible trajectories

### Key Algorithmic Features

| Feature | Description |
|---------|-------------|
| **Dynamic weight adjustment** | A-star heuristic function weights are adaptively tuned to reduce planning time |
| **Goal-regulating repulsive field** | An improved repulsive potential function with target-distance factor to escape local minima |
| **Velocity-based repulsion** | A repulsive force component based on relative velocity between AUV and moving obstacles |
| **B-spline smoothing** | Cubic uniform B-spline curves eliminate sharp turns and generate smooth, feasible paths |
| **Hybrid A-star + APF** | A-star generates global waypoints; APF handles local obstacle avoidance between waypoints |

---


