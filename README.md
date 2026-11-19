# 🛰️ KalmanNet: Data-Driven Kalman Filtering for Autonomous UAV Trajectory Tracking

[![Python 3.10+](https://img.shields.io/badge/Python-3.10%2B-blue.svg)](https://www.python.org/)
[![PyTorch 2.0+](https://img.shields.io/badge/PyTorch-2.0%2B-ee4c2c.svg?logo=pytorch&logoColor=white)](https://pytorch.org/)
[![Robotics & State Estimation](https://img.shields.io/badge/Domain-Robotics%20%26%20State%20Estimation-green.svg)]()
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

An implementation of **KalmanNet** for autonomous Unmanned Aerial Vehicle (UAV) trajectory estimation and navigation under non-linear aerodynamic maneuvers and GNSS-denied / noisy sensor environments. KalmanNet bridges classical physical state-space modeling with deep learning by retaining the kinematic transition structure while learning the dynamic **Kalman Gain** ($\mathcal{K}_t$) through a neural network, eliminating the need for exact prior knowledge of process ($Q$) and observation ($R$) noise covariance matrices.

---

## 📌 Motivation & Theoretical Background

Classical Kalman Filtering (KF, EKF, UKF) relies heavily on exact mathematical models and stationary Gaussian assumptions:
* In real-world aerial operations (urban canyons, tunnels, electronic jamming), GPS signals degrade or drop out.
* Sensor noise covariance ($R$) fluctuates with temperature and vibration, while process noise covariance ($Q$) varies with wind gusts and aerodynamic turbulence.
* Model mismatch in classical filters causes divergence, phase lag, or severe tracking degradation.

**KalmanNet** resolves this bottleneck by integrating a recurrent neural architecture into the linear state-space estimation loop:

$$\hat{\mathbf{x}}_t = \mathbf{F}\hat{\mathbf{x}}_{t-1} + \mathcal{K}_t\left(\mathbf{y}_t - \mathbf{H}\mathbf{F}\hat{\mathbf{x}}_{t-1}\right)$$

Where the Kalman Gain $\mathcal{K}_t$ is inferred dynamically at each timestep from the observation innovation and prior state features via recurrent GRU/linear layers.

---

## 🏗️ State-Space & Kinematic Formulation

The autonomous drone operates in a 6-dimensional kinematic state space:

$$\mathbf{x}_t = \begin{bmatrix} x_t & y_t & z_t & \dot{x}_t & \dot{y}_t & \dot{z}_t \end{bmatrix}^T$$

* **State Transition Matrix ($\mathbf{F}$):** Models constant velocity kinematics with sampling interval $\Delta t = 0.1\text{s}$:
$$\mathbf{F} = \begin{bmatrix} \mathbf{I}_{3\times3} & \Delta t \cdot \mathbf{I}_{3\times3} \\ \mathbf{0}_{3\times3} & \mathbf{I}_{3\times3} \end{bmatrix}$$
* **Observation Matrix ($\mathbf{H}$):** Maps noisy 3D position fixes from onboard sensors:
$$\mathbf{H} = \begin{bmatrix} \mathbf{I}_{3\times3} & \mathbf{0}_{3\times3} \end{bmatrix}$$
* **Non-Linear Dynamics:** Flight paths include 3D helical spirals, high-$g$ evasive turns, and non-stationary process noise.

---

## 📊 Empirical Benchmarks & Evaluation

The framework evaluates KalmanNet against:
1. **Classical KF (True Covariance):** Theoretical upper bound with access to exact simulated noise distributions.
2. **Mismatched KF:** Standard operational scenario where $Q$ and $R$ covariances are misspecified by an order of magnitude.
3. **Raw Noisy Observations:** Unfiltered sensor measurements.

| Estimator | Trajectory MSE (Linear) | Error (dB) | Robustness to Model Mismatch |
| :--- | :---: | :---: | :---: |
| **Raw Sensor Observations** | 2.9412 | +4.68 dB | Low |
| **Mismatched Classical KF** | 1.4820 | +1.71 dB | Poor (Severe lag / overshoot) |
| **Classical KF (True $Q, R$)** | 0.3812 | -4.19 dB | High (Requires perfect oracle) |
| **KalmanNet (Ours)** | **0.4015** | **-3.96 dB** | **Outstanding (Model-agnostic)** |

> *KalmanNet approaches the theoretical performance of the optimal Kalman Filter with zero prior knowledge of noise statistics, outperforming standard mismatched filters.*

---

## 📈 Visualizations

The evaluation script generates comparative 3D flight paths and 2D axis projections (`kalmannet_uav_evaluation.png`):
* Ground-truth aerodynamic trajectory vs estimated paths.
* State estimation error over time across $X, Y, Z$ and velocity components.
* Innovation residual distributions confirming unbiased tracking.

---

## 📁 Repository Structure

```
KalmanNet-UAV-Navigation/
├── KalmanNet_UAV.ipynb                 # Interactive Jupyter notebook: data gen, training, evaluation
├── kalmannet_uav_evaluation.png        # Comparative 3D/2D tracking performance plot
├── kalmannet_uav_results.csv           # Quantitative benchmark metrics
├── uav_flight_telemetry_sample.csv     # Sample high-frequency drone flight telemetry
├── requirements.txt                    # Environment dependencies
├── .gitignore                          # Git ignore definitions
└── README.md                           # Documentation
```

---

## 🚀 Quickstart Guide

### 1. Installation
```bash
git clone https://github.com/23051921/KalmanNet-UAV-Navigation.git
cd KalmanNet-UAV-Navigation

pip install -r requirements.txt
```

### 2. Run the Notebook
Launch Jupyter or open in Google Colab / Kaggle:
```bash
jupyter notebook KalmanNet_UAV.ipynb
```
