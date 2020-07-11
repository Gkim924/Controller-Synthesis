# Some test environments
# ECE 498 Project - Team 5
# Written by: Greg Kim, Dhruv Mathur, Kristina Miller

import numpy as np
import matplotlib.pyplot as plt
import pypoman as ppm
from matplotlib.animation import FuncAnimation


def check_env():
    # This can be written to hard code in a change in environment
    # This should just be a list of obstacles

    A1 = np.array([[-1, 0], [1, 0], [0, -1], [0, 1]])

    b1 = np.array([[-8], [9], [-8], [9]])*10
    Goal = (A1, b1)

    # b1 = np.array([[-80], [90], [0], [10]])
    # Goal = (A1, b1)

    b2 = np.array([[5], [15], [5], [0]])*10
    b3 = np.array([[5], [15], [-10], [15]])*10
    b4 = np.array([[5], [0], [5], [10]])*10
    b5 = np.array([[-10], [15], [5], [10]])*10

    O = [(A1, b2), (A1, b3), (A1, b4), (A1, b5)]

    return O, Goal

def global_env():
    A2 = np.array([[-1, 0], [1, 0], [0, -1], [0, 1]])

    b1 = np.array([[5], [15], [5], [0]])*10
    b2 = np.array([[5], [15], [-10], [15]])*10
    b3 = np.array([[5], [0], [5], [10]])*10
    b4 = np.array([[-10], [15], [5], [10]])*10

    b5 = np.array([[-3], [3.5], [0], [5]])*10
    b6 = np.array([[-3], [7], [-5.5], [6]])*10
    b7 = np.array([[-3], [7], [1], [0]])*10
    O = [(A2, b1), (A2, b2), (A2, b3), (A2, b4), (A2, b5), (A2, b6), (A2, b7)]

    # b5 = np.array([[-20], [30], [0], [10]])
    # b6 = np.array([[-20], [30], [-20], [30]])
    # b7 = np.array([[-20], [30], [-40], [50]])
    # b8 = np.array([[-20], [30], [-60], [70]])
    # b9 = np.array([[-40], [50], [-10], [20]])
    # b10 = np.array([[-40], [50], [-30], [40]])
    # b11 = np.array([[-40], [50], [-50], [60]])
    # b12 = np.array([[-40], [50], [-70], [80]])
    # b13 = np.array([[-40], [50], [-90], [100]])
    # b14 = np.array([[-60], [70], [0], [10]])
    # b15 = np.array([[-60], [70], [-20], [30]])
    # b16 = np.array([[-60], [70], [-40], [50]])
    # b17 = np.array([[-60], [70], [-60], [70]])
    # b18 = np.array([[-60], [70], [-90], [100]])
    # O = [(A2, b1), (A2, b2), (A2, b3), (A2, b4), (A2, b5), (A2, b6), (A2, b7),
    #      (A2, b8), (A2, b9), (A2, b10), (A2, b11), (A2, b12), (A2, b13),
    #      (A2, b14), (A2, b15), (A2, b16), (A2, b17), (A2, b18)]

    # b5 = np.array([[-35], [40], [5], [30]])
    # b6 = np.array([[-35], [40], [-60], [105]])
    # b7 = np.array([[-40], [70], [-25], [30]])
    # b8 = np.array([[-40], [70], [-60], [65]])
    # b9 = np.array([[-65], [70], [-30], [60]])
    # O = [(A2, b1), (A2, b2), (A2, b3), (A2, b4), (A2, b5), (A2, b6), (A2, b7), (A2, b8), (A2, b9)]

    # b5 = np.array([[-40], [50], [0], [40]])
    # O = [(A2, b1), (A2, b2), (A2, b3), (A2, b4), (A2, b5)]

    # O = [(A2, b1), (A2, b2), (A2, b3), (A2, b4)]

    return O

def dyn_ob(ob0, dt, dir1, v = 6.5):
    A, b = ob0
    dt = dt*v
    dir = dir1
    if dir1 == 'up':
        b = np.array([[b[0][0]], [b[1][0]], [b[2][0]-dt], [b[3][0]+dt]])
        if b[3][0] >= 100:
            dir = 'down'

    elif dir1 == 'down':
        b = np.array([[b[0][0]], [b[1][0]], [b[2][0]+dt], [b[3][0]-dt]])
        if b[2][0] >= 0:
            dir = 'up'

    elif dir1 == 'left':
        b = np.array([[b[0][0]+dt], [b[1][0]-dt], [b[2][0]], [b[3][0]]])
        if b[0][0] >= 0:
            dir = 'right'

    elif dir1 == 'right':
        b = np.array([[b[0][0]-dt], [b[1][0]+dt], [b[2][0]], [b[3][0]]])
        if b[1][0] >= 100:
            dir = 'left'

    ob = (A, b)
    return ob, dir

def global_env_dyn():
    A2 = np.array([[-1, 0], [1, 0], [0, -1], [0, 1]])

    b2 = np.array([[-6], [7], [-7], [8]])*10
    b3 = np.array([[-4], [5], [-9], [10]])*10
    b4 = np.array([[-2], [3], [-2], [3]])*10

    # O = [[(A2, b2), 'left'], [(A2, b3), 'left'], [(A2, b4), 'left']]
    O = []

    return O



if __name__ == '__main__':
    A = np.array([[-1, 0], [1, 0], [0, -1], [0, 1]])
    b1 = np.array([[-4], [6], [-4], [6]])*10
    dyn_obs = [[(A, b1), 'right'], [(A, b1), 'up']]

    def animate(i):
        global dyn_obs

        plt.cla()
        for j in range(len(dyn_obs)):
            O, dir = dyn_obs[j]
            dyn_obs[j] = dyn_ob(O, 0.1, dir)

            ppm.polygon.plot_polygon(ppm.duality.compute_polytope_vertices(O[0], O[1]), alpha=0.2, color='r', linestyle='solid', fill=True, linewidth=None)

        plt.xlim(0, 100)
        plt.ylim(0, 100)

    fig, ax = plt.subplots(figsize=(6,6))
    animation = FuncAnimation(fig, func=animate, interval=10)

    plt.show()
