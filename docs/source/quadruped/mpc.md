# Model Predictive Controller

This page discusses the details of the model predictive controller (MPC) used to plan the ground reaction forces (GRFs).

## Optimization Objective

The objective function has three components: track the commanded velocity command, maintain desired body height, and reduces the size of the planned GRF. These three objectives can be written as

$$
\mathcal{J} = \sum_{i=1}^{N}{\|\mathbf{v}_i - \mathbf{v}_i^\mathrm{ref}\|_2^2 + \|\mathbf{z}_i - \mathbf{z}_i^\mathrm{ref}\|_2^2 + \|\mathbf{f}_i\|_2^2}
$$

where the planned velocity at the $i$-th time step is represented by $\mathbf{v}_i\in\mathbb{R}^3$, the commanded velocity at the $i$-th time step as $\mathbf{v}_i^\mathrm{ref}\in\mathbb{R}^3$, the planned body height at the $i$-th time step as $\mathbf{z}_i\in\mathbb{R}_+$, the desired body height at the $i$-th time step as $\mathbf{z}_i^\mathrm{ref}\in\mathbb{R}_+$, and the planned GRF at the $i$-th time step as $\mathbf{f}_i\in\mathbb{R}^{12}$.

## Constraint on GRF

Since only the stance feet generate GRFs and the stance feet should not move during the stance phase, we would then require the generated GRF in the $x$ and $y$ direction to be less than the friction force $\mu f_z$. We can then have the **friction cone constraints**

$$
\begin{align}
f_\min &\leq f_z \leq f_\max\\
-\mu f_z &\leq f_x \leq \mu f_z\\
-\mu f_z &\leq f_y \leq \mu f_z
\end{align}
$$

for the stance feet. For the swing feet we have the constraint $\mathbf{f} = \mathbf{0}$.

## System Dynamics

The system dynamics concerning the MPC controller is the centroidal dynamics, i.e., approximating the entire robot as a single point with mass and inertia. The centroidal dynamics are governed by the Newton-Euler equations

$$
\begin{align}
m\dot{\mathbf{p}} &= \sum_{j \in \mathrm{FN}}{\mathbf{f}_j} + m\mathbf{g}\\
\mathbf{I}\dot{\omega} + \dot{\omega}\times(\mathbf{I}\dot{\omega}) &= \sum_{j \in \mathrm{FN}}{\mathbf{r}_j\times\mathbf{f}_j}
\end{align}
$$

with $m\in\mathbb{R}_+$ representing the mass of the robot, $\mathbf{p}\in\mathbb{R}^{3}$ representing the center of mass (CoM) position of the robot in the world frame, $\mathbf{g}\in\mathbb{R}^{3}$ representing the gravitaional acceleration, $\mathbf{I}\in\mathbb{R}^{3\times3}$ representing the inertia matrix of the robot, $\omega\in\mathbb{R}^{3}$ representing the angular velocity of the robot, $\mathbf{r}_j\in\mathbb{R}^{3}$ representing the vector starting from the CoM and ends at the foot $j$ measured in the world frame, and $\mathrm{FN} = \{\mathrm{FL}, \mathrm{FR}, \mathrm{HL}, \mathrm{HR}\}$ represents the **F**oot **N**ames.