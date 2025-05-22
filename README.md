# 🚗🚁 Unity Autonomous Navigation — Hybrid A* for Car and Drone

This project showcases autonomous navigation in Unity3D using path planning and control techniques for both a **car** and a **drone**. It was developed as part of an assignment from the KTH course DD2438 on vehicle motion models and navigation in partially constrained environments.

The objective was to plan and execute efficient paths through unknown (but fully accessible) maps using realistic models, and complete the journey from start to goal **as quickly as possible**, considering only **driving time**.

---

## 📌 Features

- Hybrid A* path planning for non-holonomic car model  
- A* path planning for holonomic drone model  
- Post-processing via Douglas-Peucker algorithm for path simplification  
- Adaptive PD controllers for real-time path tracking  
- Obstacle avoidance with raycast feedback and recovery behavior  
- Designed for Unity simulation with visual and physical feedback

---

## ⚙️ Implementation Details

### 🚘 Car Path Planning

- **Algorithm:** Hybrid A*  
- **Heuristic:** Euclidean distance to goal  
- **Cost function (`f`):** Path length so far + heuristic  
- **Motion constraints:**  
  - Limited to slight steering angles to prevent sharp turns  
  - Only considers positions surrounded by wide, obstacle-free areas to ensure safe clearance  
- **Post-processing:**  
  - **Douglas-Peucker algorithm** to remove redundant points and favor longer straight segments  
  - Improves speed and stability during driving  
- **Obstacle Handling:**  
  - Raycasts (forward and diagonals) detect potential collisions  
  - When stuck, the car moves backward and tries to realign  
  - Additional steering away from nearby obstacles
- **PD-Tracking:**  
  - Parameters `kp` and `kd` vary depending on the distance to the next waypoint  
  - Enables smooth acceleration on straight segments and deceleration near turns

### 🤖 Drone Path Planning

- **Algorithm:** Standard A* (grid-based)  
- **Motion constraints:**  
  - Movement limited to axis-aligned directions (forward, backward, left, right)  
  - Safety constraint on surrounding area (drone requires clearance)  
- **Heuristic & cost function:** Same as car  
- **Optimization:**  
  - Penalizes direction changes in A* to encourage long straight segments  
- **Post-processing:** Douglas-Peucker simplification for efficiency  
- **PD-Tracking:**  
  - Parameters `kp` and `kd` vary depending on the distance to the next waypoint  
  - Enables smooth acceleration on straight segments and deceleration near turns

---

## 🧪 How to Run

1. Open the project in **Unity**.
2. Load one of the test scenes (in `/Scenes/`).
3. Press Play to run the simulation.
4. The car and drone will autonomously compute and follow a path from start to goal.

---

## 🧠 Technologies Used

- Unity3D
- C# scripting
- Hybrid A* path planning
- Douglas-Peucker algorithm
- Custom PD controllers
- Raycasting for reactive obstacle avoidance

---


## 👤 Authors

- Chloé Leyssens  
- Alexandra Papadopoulos
