# Unitree B1 Simulation Environment
Simulation environment for Unitree B1 robot.

## Installation

First create a conda environment

```console
conda create --name b1_env python=3.8
```

Then install `pinocchio` using

```console
conda install -c conda-forge pinocchio
```

and install `bullet_utils` using

```console
git clone https://github.com/machines-in-motion/bullet_utils.git
cd bullet_utils
python3 -m pip install .
```

Then clone this repo, and install it using `pip`

```console
git clone https://github.com/RobotCasters/B1Env.git
cd B1Env
python3 -m pip install -e .
```
