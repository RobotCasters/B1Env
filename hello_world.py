import numpy as np

from b1_env import B1Sim


def main():
    env = B1Sim(render_mode="human", useFixedBase=False)
    env.reset()

    for i in range(1000000):
        env.step(np.ones((12,)))


if __name__ == "__main__":
    main()
