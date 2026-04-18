# Robot Motion Planning and Control — Lab 1

**UR5 Manipulator: Kinematics, Trajectory Planning, and Inverse Dynamics (Newton–Euler)**

---

## 1. Objective

This laboratory work implements a complete pipeline for robotic manipulator analysis:

* Extract kinematic parameters (Denavit–Hartenberg)
* Define dynamic parameters
* Plan a smooth trajectory between two configurations
* Solve inverse dynamics using the Newton–Euler formulation
* Compute and analyze dynamic matrices ( $M(q)$, $C(q,\dot{q})$,$G(q)$ )

---

## 2. Robot Model

The selected robot is the **UR5 6-DOF industrial manipulator**.

The model is loaded from the Robotics Toolbox:

```python
robot = rtb.models.DH.UR5()
```

This provides a structured representation of the robot’s kinematics and baseline dynamics.

---

## 3. Kinematic Model (DH Parameters)

The Denavit–Hartenberg (DH) convention is used to describe the robot geometry.


| Joint | θ (rad) | d (m)    | a (m)    | α (rad) |
| ----- | ------- | -------- | -------- | ------- |
| 1     | θ₁      | 0.089159 | 0        | +π/2    |
| 2     | θ₂      | 0        | -0.425   | 0       |
| 3     | θ₃      | 0        | -0.39225 | 0       |
| 4     | θ₄      | 0.10915  | 0        | +π/2    |
| 5     | θ₅      | 0.09465  | 0        | -π/2    |
| 6     | θ₆      | 0.0823   | 0        | 0       |

**Notes:**

* θᵢ are the joint variables (revolute joints)
* Units are in meters and radians
* Parameters correspond to the standard DH convention used in the Robotics Toolbox


These parameters define the forward kinematics chain.

---

## 4. Dynamic Model

Each link is assigned the following parameters:

* Link mass ( m_i )
* Center of mass ( r_i )
* Inertia tensor ( I_i )
* Motor inertia ( J_{m,i} )
* Viscous friction ( B_i )
* Coulomb friction ( T_{c,i} )
* Gear ratio ( G_i )
* Joint limits ( q_{lim,i} )

---

## 5. Assumptions and Parameter Justification

The manufacturer does **not provide complete actuator-level dynamics** (motor inertia, friction, gearbox details).

Therefore, a hybrid modeling approach is used:

### 5.1 Parameters from structured models

* Mass, COM, inertia tensors extracted from URDF / toolbox models

### 5.2 Estimated parameters

The following are approximated based on robotics literature:

* **Gear ratio**:
  ( $G \approx 80–120$ ) (harmonic drive typical range)

* **Viscous friction**:
  Small values to model damping

* **Coulomb friction**:
  Low asymmetric values to capture stiction effects

* **Motor inertia**:
  Assumed small relative to link inertia

These assumptions are standard practice in simulation when exact data is unavailable.

---

## 6. Configurations

Two configurations are defined:

* Initial configuration: ( $q_{start}$ )
* Final configuration: ( $q_{end}$ )

These represent valid joint-space positions within limits.

---

## 7. Trajectory Planning

A smooth joint-space trajectory is generated:

```python
tr = rtb.jtraj(q_start, q_end, n_steps)
```

This yields:

* Position ( $q(t)$ )
* Velocity ( $\dot{q}$(t) )
* Acceleration ( $\ddot{q}$(t) )

ensuring dynamic consistency.

---

## 8. Inverse Dynamics (Newton–Euler Formulation)

The joint torques are computed using:


$\tau = M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q)$


Three motion regimes are analyzed:

### 8.1 Full dynamics


$\dot{q} \neq 0,\quad \ddot{q} \neq 0$


### 8.2 Quasi-static


$\dot{q} \neq 0,\quad \ddot{q} \approx 0$


### 8.3 Static equilibrium


$\dot{q} = 0,\quad \ddot{q} = 0$


---

## 9. Dynamic Matrices Evaluation

At each time step, the following are computed:

* Inertia matrix ( $M(q)$ )
* Coriolis matrix ( $C(q,\dot{q})$ )
* Gravity vector ( $G(q)$ )

These characterize the robot’s dynamic response.

---

## 10. Results

The following outputs are generated:

* Joint torque profiles ( $\tau$(t) )
* Evolution of dynamic matrices
* Comparative analysis across motion regimes

### Observations

* Full dynamics produces **maximum torque demand**
* Quasi-static reduces torque due to negligible acceleration
* Static case corresponds to **pure gravity compensation**

---

## 11. Discussion

The results highlight:

* Strong dependence of torque on acceleration terms
* Dominance of gravity in low-motion regimes
* Sensitivity to assumed friction and gearbox parameters

Despite approximations, the model preserves realistic physical behavior.

---

## 12. Conclusion

This work demonstrates a full robotic dynamics pipeline:

* Kinematic modeling
* Trajectory generation
* Dynamic analysis

A physically consistent model is achieved despite incomplete manufacturer data by using justified engineering assumptions.

The Newton–Euler method proves effective for real-time torque computation.

---

## 13. How to Run

Install dependencies:

```bash
pip install roboticstoolbox-python numpy matplotlib
```

Run the notebook:

```bash
jupyter notebook Lab1.ipynb
```

---

## 14. Project Structure

```
Robot-Motion-Planning-Lab1/
│
├── Lab1.ipynb
├── README.md
```

---

## 15. References


* Robotics Toolbox for Python
* UR5 industrial robot documentation
* ROS URDF models

---
