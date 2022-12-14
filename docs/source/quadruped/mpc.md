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
                      &\ \mathrm{LB} \leq \mathbf{C}x \leq \mathrm{UB}.
\end{align}
$$(eqn:qp_solver_formulation)

Thus, our final task is to transform the MPC formulation {eq}`eqn:mpc_formulation` into the form of {eq}`eqn:qp_solver_formulation`.

### Objective Function

First, we write the objective function in a quadratic form. Let's initially focus on writing the cost for a single time step

$$
\|\mathbf{v}_{i+1} - \mathbf{v}_{i+1}^\mathrm{ref}\|_2^2 + \|\mathbf{z}_{i+1} - \mathbf{z}_{i+1}^\mathrm{ref}\|_2^2 + \|\mathbf{f}_i\|_2^2
$$

in a quadratic form. The first two terms can be written as

$$
\begin{align}
&\ (\mathbf{x}_{i+1} - \mathbf{x}_{i+1}^\mathrm{ref})^T\mathbf{Q}_{i+1}(\mathbf{x}_{i+1} - \mathbf{x}_{i+1}^\mathrm{ref})\\
=&\ \mathbf{x}_{i+1}^T\mathbf{Q}_{i+1}\mathbf{x}_{i+1} - 2(\mathbf{x}_{i+1}^\mathrm{ref})^T\mathbf{Q}_{i+1}\mathbf{x}_{i+1} + (\mathbf{x}_{i+1}^\mathrm{ref})^T\mathbf{Q}_{i+1}\mathbf{x}_{i+1}^\mathrm{ref}.
\end{align}
$$

with 

$$
\mathbf{x}_{i+1}^\mathrm{ref} = \begin{bmatrix}
    \mathbf{e}_z^\mathrm{ref}\\
    \mathbf{0}_{3\times1}\\
    \mathbf{v}_{i+1}^\mathrm{ref}\\
    \mathbf{0}_{3\times1}\\
    \mathbf{0}_{3\times1}
\end{bmatrix}\in\mathbb{R}^{15\times1} \quad \mathbf{Q}_{i+1} = \mathrm{blockdiag}\Big(\mathbf{I}_{3}, \mathbf{0}_{3\times3}, \mathbf{I}_{3}, \mathbf{0}_{6\times6}\Big)\in\mathbb{R}^{15\times15}.
$$

The last term can be written as

$$
\mathbf{f}_i^T\mathbf{R}_{i}\mathbf{f}_i,\ \mathbf{R}_{i} = \mathbf{I}_{12}.
$$

Then, we have the equivalent objective function as

$$
\sum_{i=0}^{N-1}{\mathbf{x}_{i+1}^T\mathbf{Q}_{i+1}\mathbf{x}_{i+1} + \mathbf{f}_i^T\mathbf{R}_{i}\mathbf{f}_i - 2(\mathbf{x}_{i+1}^\mathrm{ref})^T\mathbf{Q}_{i+1}\mathbf{x}_{i+1}}.
$$

We can group the quadratic terms for the states as $\displaystyle\frac{1}{2}x_{\mathbf{x}}^T\mathbf{H}_\mathbf{Q}x_{\mathbf{x}}$, with

$$
x_{\mathbf{x}} = \begin{bmatrix}
    \mathbf{x}_{1}\\
    \vdots\\
    \mathbf{x}_{N}
\end{bmatrix}\in\mathbb{R}^{15N\times1} \quad \mathbf{H}_\mathbf{Q} = 2\mathrm{blockdiag}\Big(\mathbf{Q}_{1}, \cdots, \mathbf{Q}_{N}\Big)\in\mathbb{R}^{15N\times15N}.
$$

Similarly, we can group the quadratic terms for the GRFs as $\displaystyle\frac{1}{2}x_{\mathbf{f}}^T\mathbf{H}_\mathbf{R}x_{\mathbf{f}}$, with

$$
x_{\mathbf{f}} = \begin{bmatrix}
    \mathbf{f}_0\\
    \vdots\\
    \mathbf{f}_{N-1}
\end{bmatrix}\in\mathbb{R}^{12N\times1} \quad \mathbf{H}_\mathbf{R} = 2\mathrm{blockdiag}\Big(\mathbf{R}_{0}, \cdots, \mathbf{R}_{N-1}\Big)\in\mathbb{R}^{12N\times12N}.
$$

The linear terms can be grouped as $\mathbf{g}_\mathbf{x}^Tx_\mathbf{x}$, with

$$
\mathbf{g}_\mathbf{x} = \begin{bmatrix}
    -2\mathbf{Q}_{1}^T\mathbf{x}_{1}^\mathrm{ref}\\
    \vdots\\
    -2\mathbf{Q}_{N}^T\mathbf{x}_{N}^\mathrm{ref}
\end{bmatrix}\in\mathbb{R}^{15N\times1}.
$$

Additionally, we define $\mathbf{g}_\mathbf{f} = \mathbf{0}_{12N\times1}$. Then, we can have the MPC objective in the form of {eq}`eqn:qp_solver_formulation`, with

$$
\begin{align}
x &= \begin{bmatrix}
    x_{\mathbf{x}}\\
    x_{\mathbf{f}}
\end{bmatrix}\in\mathbb{R}^{27N\times1}\\
\mathbf{H} &= \begin{bmatrix}
    \mathbf{H}_\mathbf{Q} & \mathbf{0}_{15N\times12N}\\
    \mathbf{0}_{12N\times15N} & \mathbf{H}_\mathbf{R}
\end{bmatrix}\in\mathbb{R}^{27N\times27N}\\
\mathbf{g} &= \begin{bmatrix}
    \mathbf{g}_\mathbf{x}\\
    \mathbf{g}_\mathbf{f}
\end{bmatrix}\in\mathbb{R}^{27N\times1}.
\end{align}
$$

### Inequality Constraint

The inequality constraints of the MPC problem are the friction cone constraints

$$
\begin{align}
f_\min &\leq f_{z, ij} \leq f_\max\\
-\mu f_{z, ij} &\leq f_{x, ij} \leq \mu f_{z, ij}\\
-\mu f_{z, ij} &\leq f_{y, ij} \leq \mu f_{z, ij}.
\end{align}
$$

We can write this as six individual constraints

$$
\begin{align}
-f_{z, ij} &\leq -f_\min\\
f_{z, ij} &\leq f_\max\\
-f_{x, ij} - \mu f_{z, ij} &\leq 0\\
f_{x, ij} - \mu f_{z, ij} &\leq 0\\
-f_{y, ij} - \mu f_{z, ij} &\leq 0\\
f_{y, ij} - \mu f_{z, ij} &\leq 0.
\end{align}
$$

This can be written as $\bar{\mathbf{c}}_{\mathbf{f}}\mathbf{f}_{ij} \leq \bar{\mathrm{ub}}$, with

$$
\bar{\mathbf{c}}_{\mathbf{f}} = \begin{bmatrix}
0 & 0 & -1\\
0 & 0 & 1\\
-1 & 0 & -\mu\\
1 & 0 & -\mu\\
0 & -1 & -\mu\\
0 & 1 & -\mu
\end{bmatrix} \quad \bar{\mathrm{ub}}_\mathbf{f} = \begin{bmatrix}
-f_\min\\
f_\max\\
0\\
0\\
0\\
0
\end{bmatrix}
$$

with $i\in\{1, \cdots, N-1\}$ representing the $i$-th time step and $j\in\{\mathrm{FL}, \mathrm{FR}, \mathrm{HL}, \mathrm{HR}\}$ representing the foot $j$. Note that this constraint is only applied to stance feet. 

First, we would need a mask to select the stance feet, we call this masking matrix $\mathcal{M}_{st}$, the masking matrix would need to the number of stance feet $n_{st}$ and their indices $j$. The size of $\mathcal{M}_{st}$ is $3n_{st} \times 12$. If the front left (FL) and hind right (HR) feet are the stance feet then we have 

$$
\mathcal{M}_{st} = \begin{bmatrix}
\mathbf{I}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3\\
\mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{I}_3
\end{bmatrix}\in\mathbb{R}^{6\times12}.
$$

If the front left (FL), front right (FR), and hind right (HR) feet are the stance feet then we have

$$
\mathcal{M}_{st} = \begin{bmatrix}
\mathbf{I}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3\\
\mathbf{0}_3 & \mathbf{I}_3 & \mathbf{0}_3 & \mathbf{0}_3\\
\mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{I}_3
\end{bmatrix}\in\mathbb{R}^{9\times12}.
$$

Then, the friction cone constraint at each time step becomes

$$
\mathbf{c}_{\mathbf{f}}\mathcal{M}_{st}\mathbf{f}_i \leq \mathrm{ub}_\mathbf{f}
$$

with 

$$
\mathbf{c}_{\mathbf{f}} = \mathrm{blockdiag}\Big(\underbrace{\bar{\mathbf{c}}_{\mathbf{f}}, \cdots, \bar{\mathbf{c}}_{\mathbf{f}}}_{n_{st}}\Big)\in\mathbb{R}^{6n_{st}\times3n_{st}} \quad \mathrm{ub}_\mathbf{f} = \mathrm{vstack}\Big(\underbrace{\bar{\mathrm{ub}}_\mathbf{f}, \cdots, \bar{\mathrm{ub}}_\mathbf{f}}_{n_{st}}\Big)\in\mathbb{R}^{6n_{st}\times1}
$$

We can then combine the friction cone constraints for the entire time horizon, and be in the form of

$$
\mathbf{C}_\mathbf{f}x_\mathbf{f} \leq \mathrm{UB}_\mathbf{f}\\
$$(eqn:friction_cone_constraint)

with

$$
\begin{align}
\mathbf{C}_\mathbf{f} &= \mathrm{blockdiag}\Big(\mathbf{c}_{\mathbf{f}}\mathcal{M}_{st, 1}, \cdots, \mathbf{c}_{\mathbf{f}}\mathcal{M}_{st, N}\Big)\in\mathbb{R}^{6n_{st}N\times12N}\\
\mathrm{UB}_\mathbf{f} &= \mathrm{vstack}\Big(\mathrm{ub}_\mathbf{f}, \cdots, \mathrm{ub}_\mathbf{f}\Big)\in\mathbb{R}^{6n_{st}N\times1}
\end{align}
$$

If we also include $x_\mathbf{x}$, we have the final inequality constraint as $\mathbf{C}x \leq \mathrm{UB}$, where

$$
\mathbf{C} = \begin{bmatrix}
\mathbf{0} & \mathbf{C}_\mathbf{f}
\end{bmatrix}\in\mathbb{R}^{6n_{st}N\times27N} \quad \mathrm{UB} = \mathrm{UB}_\mathbf{f}\in\mathbb{R}^{6n_{st}N\times1}.
$$
