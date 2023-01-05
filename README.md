# Unitree B1 Simulation Environment xx
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

Then clone this repo, and install it using `pip`

```console
git clone https://github.com/BolunDai0216/B1Env.git
cd B1Env
python3 -m pip install -e .
```
