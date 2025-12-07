---
sidebar_position: 99
title: Mathematical Notation
description: Reference guide for mathematical notation used in the textbook
---

# Mathematical Notation Reference

This page provides a comprehensive reference for mathematical notation used throughout the Physical AI & Humanoid Robotics textbook.

## Vectors and Matrices

- **v** - Vector (bold lowercase)
- **M** - Matrix (bold uppercase)
- **vᵀ** - Vector transpose
- **‖v‖** - Vector norm (magnitude)
- **v · w** - Dot product
- **v × w** - Cross product

## Coordinate Frames

- **Frame W** - World frame
- **Frame B** - Body frame
- **Frame E** - End-effector frame
- **ᴬp** - Position vector expressed in frame A
- **ᴬRᴮ** - Rotation matrix from B to A
- **ᴬTᴮ** - Homogeneous transformation from B to A

## Kinematics

- **q** - Joint positions (configuration)
- **q̇** - Joint velocities
- **q̈** - Joint accelerations
- **p** - End-effector position
- **R** - Orientation (rotation matrix)
- **J** - Jacobian matrix

## Dynamics

- **M(q)** - Inertia matrix
- **C(q, q̇)** - Coriolis and centrifugal forces
- **g(q)** - Gravity vector
- **τ** - Joint torques
- **F** - External forces

## Reinforcement Learning

- **sₜ** - State at time t
- **aₜ** - Action at time t
- **rₜ** - Reward at time t
- **π(a|s)** - Policy (action distribution given state)
- **V(s)** - Value function
- **Q(s, a)** - Action-value function
- **γ** - Discount factor

## Probability and Statistics

- **p(x)** - Probability density/mass function
- **𝔼[X]** - Expected value
- **Var(X)** - Variance
- **𝒩(μ, σ²)** - Normal distribution
- **x ~ p** - x is distributed according to p
- **Cov(X, Y)** - Covariance
- **P(A|B)** - Conditional probability

## Control Theory

- **u(t)** - Control input at time t
- **y(t)** - System output at time t
- **e(t)** - Error signal (setpoint - output)
- **Kₚ** - Proportional gain (PID)
- **Kᵢ** - Integral gain (PID)
- **Kₐ** - Derivative gain (PID)
- **G(s)** - Transfer function (Laplace domain)
- **A, B, C, D** - State-space matrices

## Optimization

- **min f(x)** - Minimize function f over x
- **s.t.** - Subject to (constraint indicator)
- **∇f** - Gradient of f
- **∇²f** - Hessian matrix of f
- **λ** - Lagrange multiplier
- **α** - Learning rate (gradient descent)

## Computer Vision

- **I(x, y)** - Image intensity at pixel (x, y)
- **K** - Camera intrinsic matrix
- **R, t** - Rotation and translation (extrinsic)
- **f** - Focal length
- **u, v** - Pixel coordinates
- **X, Y, Z** - 3D world coordinates

## Neural Networks

- **W** - Weight matrix
- **b** - Bias vector
- **σ(·)** - Activation function (sigmoid, ReLU, tanh)
- **L** - Loss function
- **θ** - Model parameters
- **∂L/∂θ** - Gradient of loss with respect to parameters
- **η** - Learning rate

## Linear Algebra Operations

| Symbol | LaTeX | Meaning | First Used In |
|--------|-------|---------|---------------|
| **v** | `\mathbf{v}` | Vector | Module 0, Chapter 3 |
| **M** | `\mathbf{M}` | Matrix | Module 0, Chapter 3 |
| **vᵀ** | `\mathbf{v}^T` | Transpose | Module 0, Chapter 3 |
| **M⁻¹** | `\mathbf{M}^{-1}` | Matrix inverse | Module 0, Chapter 3 |
| **‖v‖** | `\lVert \mathbf{v} \rVert` | Norm | Module 0, Chapter 3 |
| **v · w** | `\mathbf{v} \cdot \mathbf{w}` | Dot product | Module 0, Chapter 3 |
| **v × w** | `\mathbf{v} \times \mathbf{w}` | Cross product | Module 0, Chapter 3 |

## Calculus Operations

| Symbol | LaTeX | Meaning | First Used In |
|--------|-------|---------|---------------|
| **dx/dt** | `\frac{dx}{dt}` | Time derivative | Module 0, Chapter 3 |
| **∂f/∂x** | `\frac{\partial f}{\partial x}` | Partial derivative | Module 0, Chapter 3 |
| **∇f** | `\nabla f` | Gradient | Module 2, Chapter 4 |
| **∫ f dx** | `\int f \, dx` | Integral | Module 1, Chapter 5 |

## Greek Letters Commonly Used

| Symbol | Name | Usage |
|--------|------|-------|
| **α (alpha)** | Learning rate, angle |
| **β (beta)** | Damping coefficient |
| **γ (gamma)** | Discount factor (RL) |
| **δ (delta)** | Small change, error |
| **ε (epsilon)** | Small positive value |
| **θ (theta)** | Angle, model parameters |
| **λ (lambda)** | Eigenvalue, Lagrange multiplier |
| **μ (mu)** | Mean, friction coefficient |
| **ν (nu)** | Velocity |
| **π (pi)** | Policy (RL), mathematical constant |
| **ρ (rho)** | Density |
| **σ (sigma)** | Standard deviation, activation function |
| **τ (tau)** | Torque, time constant |
| **φ (phi)** | Angle, feature function |
| **ω (omega)** | Angular velocity |

## Subscripts and Superscripts

- **xᵢ** - i-th element of vector x
- **Mᵢⱼ** - Element at row i, column j of matrix M
- **xᵗ** - Value at time step t
- **x⁽ᵏ⁾** - Value at iteration k
- **x*** - Optimal value
- **x̂** - Estimated value
- **x̄** - Mean value
- **ẋ** - Time derivative (velocity)
- **ẍ** - Second time derivative (acceleration)

## Set Notation

- **∈** - Element of
- **∉** - Not an element of
- **⊂** - Subset of
- **∪** - Union
- **∩** - Intersection
- **ℝ** - Set of real numbers
- **ℝⁿ** - n-dimensional real vector space
- **∅** - Empty set

## Special Functions

- **sign(x)** - Sign function (+1, 0, -1)
- **max(x, y)** - Maximum of x and y
- **min(x, y)** - Minimum of x and y
- **sat(x)** - Saturation function (clamps to limits)
- **ReLU(x)** - Rectified Linear Unit: max(0, x)
- **tanh(x)** - Hyperbolic tangent
- **sigmoid(x)** - Sigmoid function: 1/(1 + e⁻ˣ)

---

## LaTeX Code Examples

For those writing mathematical content, here are common LaTeX patterns:

**Fraction:**
```latex
\frac{numerator}{denominator}
```

**Square root:**
```latex
\sqrt{x} or \sqrt[n]{x}
```

**Summation:**
```latex
\sum_{i=1}^{n} x_i
```

**Product:**
```latex
\prod_{i=1}^{n} x_i
```

**Matrix:**
```latex
\begin{bmatrix}
a & b \\
c & d
\end{bmatrix}
```

**Piecewise function:**
```latex
f(x) = \begin{cases}
x^2 & \text{if } x \geq 0 \\
-x & \text{if } x < 0
\end{cases}
```

---

**Note**: All mathematical derivations in this textbook include step-by-step proofs and references to source material (per Constitution Principle II). This notation reference is consistent across all 21 chapters covering Modules 0-4 and the Capstone project.
