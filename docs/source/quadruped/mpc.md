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
m\ddot{\mathbf{p}} &= \sum_{j \in \mathrm{FN}}{\mathbf{f}_j} + m\mathbf{g}\\
\mathbf{I}\dot{\omega} + \omega\times(\mathbf{I}\omega) &= \sum_{j \in \mathrm{FN}}{\mathbf{r}_j\times\mathbf{f}_j}
\end{align}
$$

with $m\in\mathbb{R}_+$ representing the mass of the robot, $\mathbf{p}\in\mathbb{R}^{3}$ representing the center of mass (CoM) position of the robot in the world frame, $\mathbf{g}\in\mathbb{R}^{3}$ representing the gravitaional acceleration, $\mathbf{I}\in\mathbb{R}^{3\times3}$ representing the inertia matrix of the robot, $\omega\in\mathbb{R}^{3}$ representing the angular velocity of the robot, $\mathbf{r}_j\in\mathbb{R}^{3}$ representing the vector starting from the CoM and ends at the foot $j$ measured in the world frame, and $\mathrm{FN} = \{\mathrm{FL}, \mathrm{FR}, \mathrm{HL}, \mathrm{HR}\}$ represents the **F**oot **N**ames.

### Linearization of System Dynamics

Note, that the dynamics here are nonlinear. We can make a few assumptions to linearize the dynamics. The first assumption is that the robot is more or less symmetric, this means the off-diagonal terms of the inertia matrix are zero

$$
\mathbf{I} = \begin{bmatrix}
    \mathbf{I}_{xx} & 0 & 0\\
    0 & \mathbf{I}_{yy} & 0\\
    0 & 0 & \mathbf{I}_{zz}
\end{bmatrix}.
$$

The second assumption we make is that the roll and pitch velocities of the robot are small

$$
\omega = \begin{bmatrix}
    0\\
    0\\
    \omega_z
\end{bmatrix}
$$

This assumption would not be true if we are commanding the robot to perform highly dynamic tasks, however, it would be true if it is simply walking. The following analysis relies on two properties of cross products

$$
\mathbf{a} \times \mathbf{a} = 0 \quad \mathbf{a} \times (r\mathbf{b}) = r(\mathbf{a} \times \mathbf{b})
$$

with $\mathbf{a}, \mathbf{b}\in\mathbb{R}^3$ and $r \in \mathbb{R}$. Then, we have

$$
\omega\times(\mathbf{I}\omega) = \begin{bmatrix}
    0\\
    0\\
    \omega_z
\end{bmatrix}\times\Bigg(\begin{bmatrix}
    \mathbf{I}_{xx} & 0 & 0\\
    0 & \mathbf{I}_{yy} & 0\\
    0 & 0 & \mathbf{I}_{zz}
\end{bmatrix}\begin{bmatrix}
    0\\
    0\\
    \omega_z
\end{bmatrix}\Bigg) = \begin{bmatrix}
    0\\
    0\\
    \omega_z
\end{bmatrix}\times\begin{bmatrix}
    0\\
    0\\
    \mathbf{I}_{zz}\omega_z
\end{bmatrix} = 0
$$

Thus, the Euler equation becomes

$$
\mathbf{I}\dot{\omega} + \omega\times(\mathbf{I}\omega) = \sum_{j \in \mathrm{FN}}{\mathbf{r}_j\times\mathbf{f}_j} \rightarrow \mathbf{I}\dot{\omega} = \sum_{j \in \mathrm{FN}}{\mathbf{r}_j\times\mathbf{f}_j}.
$$

One tricky part in all robotics is how to represent the orientation. Here, we choose to use RPY angles $\Theta$, mostly because the orientation of the robot is going to be solely represented by its yaw angle, which gives us a linear relationship between the RPY angles and angular velocity

$$
\dot{\Theta} \approx \mathrm{R}_z(\psi)\omega
$$

where $\mathrm{R}_z(\psi)\in\mathbb{R}^{3\times3}$ is a rotation matrix translating $\omega$ in the global frame to the CoM frame. Note that the inertia tensor $\mathbf{I}$ is commonly computed in the body frame, to translate it in the world frame we need to perform the following transformation to get the inertia tensor in the world frame

$$
\mathbf{I}_\mathcal{G} \approx \mathrm{R}_z(\psi)\mathbf{I}_\mathcal{B}\mathrm{R}_z^T(\psi).
$$

Note that since the CoM frame is aligned with the world frame, we have $\mathbf{I}_\mathcal{G} = \mathbf{I}_\mathrm{CoM}$. 

After these approximations, the equations that govern the system dynamics are

$$
\begin{align}
    m\ddot{\mathbf{p}} &= \sum_{j \in \mathrm{FN}}{\mathbf{f}_j} + m\mathbf{g}\\
    \mathbf{I}_\mathcal{G}\dot{\omega} &= \sum_{j \in \mathrm{FN}}{[\mathbf{r}_j]_\times\mathbf{f}_j}.
\end{align}
$$

If we use the state 

$$
\mathbf{x} = \begin{bmatrix}
    \mathbf{p}\\
    \Theta\\
    \dot{\mathbf{p}}\\
    \omega\\
    \mathbf{g}
\end{bmatrix},
$$

we have the linearized system dynamics as

$$
\begin{bmatrix}
    \dot{\mathbf{p}}\\
    \dot{\Theta}\\
    \ddot{\mathbf{p}}\\
    \dot{\omega}\\
    \dot{\mathbf{g}}
\end{bmatrix} = \begin{bmatrix}
    0 & 0 & \mathbf{I}_{3} & 0 & 0\\
    0 & 0 & 0 & \mathrm{R}_z(\psi) & 0\\
    0 & 0 & 0 & 0 & \mathbf{I}_{3}\\
    0 & 0 & 0 & 0 & 0\\
    0 & 0 & 0 & 0 & 0
\end{bmatrix}\begin{bmatrix}
    \mathbf{p}\\
    \Theta\\
    \dot{\mathbf{p}}\\
    \omega\\
    \mathbf{g}
\end{bmatrix} + \begin{bmatrix}
    0 & 0 & 0 & 0\\
    0 & 0 & 0 & 0\\
    \mathbf{I}_{3}/m & \mathbf{I}_{3}/m & \mathbf{I}_{3}/m & \mathbf{I}_{3}/m\\
    \mathbf{I}_\mathcal{G}^{-1}[\mathbf{r}_\mathrm{FL}]_\times & \mathbf{I}_\mathcal{G}^{-1}[\mathbf{r}_\mathrm{FR}]_\times & \mathbf{I}_\mathcal{G}^{-1}[\mathbf{r}_\mathrm{HL}]_\times & \mathbf{I}_\mathcal{G}^{-1}[\mathbf{r}_\mathrm{HR}]_\times\\
    0 & 0 & 0 & 0
\end{bmatrix}\begin{bmatrix}
    \mathbf{f}_\mathrm{FL}\\
    \mathbf{f}_\mathrm{FR}\\
    \mathbf{f}_\mathrm{HL}\\
    \mathbf{f}_\mathrm{HR}
\end{bmatrix}
$$

with $\mathbf{g} = \begin{bmatrix}0 & 0 & -9.81\end{bmatrix}^T$ and $\mathbf{I}_{3}$ representing the $3\times3$ identity matrix. We can also write the above equation as

$$
\dot{\mathbf{x}} = \mathbf{A}\mathbf{x} + \mathbf{B}\mathbf{u}.
$$

Then, we can use Euler's integration to discretize the system dynamics

$$
\mathbf{x}^\prime = \mathbf{x} + \dot{\mathbf{x}}\Delta{t} = (\mathbf{I}_{15} + \mathbf{A}\Delta{t})\mathbf{x} + \mathbf{B}\Delta{t}\mathbf{u}.
$$

If we define 

$$
\bar{\mathbf{A}} = \mathbf{I}_{15} + \mathbf{A}\Delta{t}\quad\bar{\mathbf{B}} = \mathbf{B}\Delta{t}, 
$$

we have the discretized linear dynamics as

$$
\mathbf{x}^\prime = \bar{\mathbf{A}}\mathbf{x} + \bar{\mathbf{B}}\mathbf{u}.
$$

## MPC Formulation

Then, the final MPC formulation is

$$
\begin{align}
\min_{\mathbf{x}_i, \mathbf{u}_i}\ &\ \sum_{i=0}^{N-1}{\|\mathbf{v}_{i+1} - \mathbf{v}_{i+1}^\mathrm{ref}\|_2^2 + \|\mathbf{z}_{i+1} - \mathbf{z}_{i+1}^\mathrm{ref}\|_2^2 + \|\mathbf{f}_i\|_2^2}\\
\mathrm{subject\ to}\ &\ \mathbf{f}_i^\mathrm{stance} \in \mathrm{Friction\ Cone}\\
                      &\ \mathbf{f}_i^\mathrm{swing} = 0\\
                      &\ \mathbf{x}_{i+1} = \bar{\mathbf{A}}_{i}\mathbf{x}_{i} + \bar{\mathbf{B}}_{i}\mathbf{u}_{i}.
\end{align}
$$(eqn:mpc_formulation)

## MPC Formulation to QP Solver

The standard QP solver (e.g., ProxSuite), would solve problems in the form of

$$
\begin{align}
\min_x\ &\ \frac{1}{2}x^T\mathbf{H}x + \mathbf{g}^Tx\\
\mathrm{subject\ to}\ &\ \mathbf{A}x = \mathbf{b}\\
                      &\ \mathrm{lb} \leq \mathbf{C}x \leq \mathrm{ub}.
\end{align}
$$(eqn:qp_solver_formulation)

Thus, our final task is to transform the MPC formulation {eq}`eqn:mpc_formulation` into the form of {eq}`eqn:qp_solver_formulation`.