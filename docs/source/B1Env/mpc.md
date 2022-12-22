# Implementation for MPC 

## Main Class

```{eval-rst}
.. autoclass:: B1Env.mpc.controller.MPC

    .. autofunction:: B1Env.mpc.controller.MPC.__init__
    .. autofunction:: B1Env.mpc.controller.MPC.pre_construct_objective_function
    .. autofunction:: B1Env.mpc.controller.MPC.construct_objective_function
    .. autofunction:: B1Env.mpc.controller.MPC.pre_construct_inequality_constraint
    .. autofunction:: B1Env.mpc.controller.MPC.construct_inequality_constraint
    .. autofunction:: B1Env.mpc.controller.MPC.construct_equality_constraint
    .. autofunction:: B1Env.mpc.controller.MPC.solve
```

## Utility Functions

The utiltiy functions can be grouped into three groups, one group used to compute the objective function, one for computing the inequality constraint, and one for computing the equality constraint.

### Objective Function Utilities

```{eval-rst}
.. autofunction:: B1Env.mpc.utils.get_Q_step
.. autofunction:: B1Env.mpc.utils.get_HQ
.. autofunction:: B1Env.mpc.utils.get_HR
.. autofunction:: B1Env.mpc.utils.get_H
.. autofunction:: B1Env.mpc.utils.get_gx
.. autofunction:: B1Env.mpc.utils.get_g
```

### Inequality Constraint Utilities

```{eval-rst}
.. autofunction:: B1Env.mpc.utils.get_bcf
.. autofunction:: B1Env.mpc.utils.get_bubf
.. autofunction:: B1Env.mpc.utils.get_C
.. autofunction:: B1Env.mpc.utils.get_UB
```

### Equality Constraint Utilities

```{eval-rst}
.. autofunction:: B1Env.mpc.utils.get_b
.. autofunction:: B1Env.mpc.utils.get_A
```
