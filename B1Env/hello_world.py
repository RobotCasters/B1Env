import numpy as np

from B1Env.b1_env import B1Sim


def main():
    """Hello world example for running the B1 simulation"""
    env = B1Sim(render_mode="human", useFixedBase=True)
    env.reset()

    for i in range(1000000):
        env.step(np.zeros((12,)))


if __name__ == "__main__":
    main()
