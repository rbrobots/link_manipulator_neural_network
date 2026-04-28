# Link Manipulator Neural Network

## Overview

In *A study of neural network based inverse kinematics solution for three-joint robot* (Köker 2004), the authors propose using backpropagation to retrieve the inverse kinematics for a three-joint robot. They state that the advantage is that the artificial neural network uses shorter calculations (in comparison to inverse kinematic solutions which is time consuming) and therefore less computation power.

Given a 5-DOF link manipulator, a similar network is implemented in MATLAB to minimise the error between the target and predicted angles for joints q2, q3, and q4 (joint q1 is fixed at −90°, q5 at 0°). The network maps end-effector positions (x, y, z) — derived from forward kinematics — back to the joint angles that produced them.

---

## How to Run

1. Open MATLAB and set the working directory to this project folder.
2. Ensure `calculatePVA.m` is available on the MATLAB path (external dependency used by `returnPVA`).
3. Define input angle waypoints as column vectors in radians:

```matlab
q2 = [40 90 45 40]' * pi/180;
q3 = [-20 -70 -45 -50]' * pi/180;
q4 = [20 5 10 0]' * pi/180;
```

4. Call the main function:

```matlab
neural_network(q2, q3, q4)
```

The function prints `Training Complete`, then the MSE for each joint, then `Testing Complete`.

---

## File Reference

### `neural_network.m`

The main entry point. Trains the network on cubic trajectories derived from the input waypoints, then evaluates it on a fixed test set and reports mean squared error (MSE) per joint.

**Network architecture**

```
Input layer (3 neurons)     Hidden layer (3 neurons)     Output layer (3 neurons)
   x position          →       sigmoid activations    →       q2 angle
   y position                                                  q3 angle
   z position                                                  q4 angle
```

- **Activation:** Sigmoid — `1 / (1 + exp(-x))`
- **Weights:** 9×2 matrix — column 1 = input→hidden, column 2 = hidden→output. For neuron `n`, its weights from the three previous-layer neurons sit at rows `n`, `n+3`, `n+6`.
- **Learning rate:** `0.1` | **Biases:** `1` (hidden and output)

**Training:** Cubic trajectories are generated between each consecutive waypoint pair via `returnPVA`. Forward kinematics (`returnTransformation`) converts those angles to (x, y, z) end-effector positions as network inputs. Each time step runs a feedforward pass then a backpropagation pass to update weights.

**Testing:** Evaluated on a hardcoded set of 10 test waypoints. No weight updates occur. Outputs `mse_joint2`, `mse_joint3`, `mse_joint4`.

---

### `returnTransformation(target_joint)`

Computes the (x, y, z) position of a specified joint frame using DH-parameter forward kinematics.

| Parameter | Description |
|---|---|
| `target_joint` | Integer 1–6; use `6` for the end effector |

Returns a 1×3 symbolic vector as a function of `theta1 … theta5`. Evaluate numerically with `subs`:

```matlab
T0E = returnTransformation(6);
syms theta1 theta2 theta3 theta4 theta5;
pos = double(subs(T0E, {theta1 theta2 theta3 theta4 theta5}, {-90, q2, q3, q4, 0}));
```

**Link parameters**

| Variable | Value (cm) | Meaning |
|---|---|---|
| `d1` | 6.6 | Base height |
| `d2` | 12.0 | Link 1 length |
| `d3` | 12.7 | Link 2 length |
| `d4` | 3.2 | Link 3 length |
| `d5` | 6.6 | End-effector offset |

Six DH matrices (`T01` through `T5E`) are multiplied cumulatively; the translation column of the requested result is returned as the position.

---

### `returnPVA(start_pos, end_pos, duration)`

Generates a cubic trajectory between two joint positions, returning position, velocity, and acceleration at each 0.1 s time step.

| Parameter | Description |
|---|---|
| `start_pos` | Starting joint angle (radians) |
| `end_pos` | Target joint angle (radians) |
| `duration` | Travel time in seconds (e.g. `3` → 31 rows) |

Returns an N×3 matrix `[position, velocity, acceleration]` per row. Delegates to the external `calculatePVA` for the cubic polynomial computation.

---

## References

Köker, R., Öz, C., Çakar, T. and Ekiz, H., 2004. A study of neural network based inverse kinematics solution for a three-joint robot. *Robotics and Autonomous Systems*, 49(3–4), pp. 227–234.
