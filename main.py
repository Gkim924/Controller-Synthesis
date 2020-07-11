# Implementation of Fast Controller Synthesis
# ECE 498 Project - Team 5
# Written by: Greg Kim, Dhruv Mathur, Kristina Miller

import numpy as np
import matplotlib.pyplot as plt
import polytope as pc
from xref_yices import *
from kinematic_car import *
from ref2traj import *
from environment import *

# def check_env():
#     # This can be written to hard code in a change in environment
#     # This should just be a list of obstacles
#     A1 = np.array([[-1, 0], [1, 0], [0, -1], [0, 1]])
#     b1 = np.array([[-8], [9], [-8], [9]])*10
#     Goal = (A1, b1)
#
#     A2 = np.array([[-1, 0], [1, 0], [0, -1], [0, 1]])
#     b2 = np.array([[-3], [3.5], [0], [5]])*10
#     b3 = np.array([[-3], [7], [-5.5], [6]])*10
#     b4 = np.array([[-3], [7], [1], [0]])*10
#
#     b5 = np.array([[0], [10], [1], [0]])*10
#     b6 = np.array([[0], [10], [-10], [11]])*10
#     b7 = np.array([[1], [0], [0], [10]])*10
#     b8 = np.array([[-10], [11], [0], [10]])*10
#
#     O = [(A2, b5), (A2, b6), (A2, b7), (A2, b8)]
#
#     return O, Goal

def create_Theta(pos, err):
    A = np.array([[-1, 0], [1, 0], [0, -1], [0, 1]])
    b = np.array([[-(pos[0] - err/sqrt(2))],[(pos[0] + err/sqrt(2))],[-(pos[1] - err/sqrt(2))],[(pos[1] + err/sqrt(2))]])
    return (A, b)

def main(initial_pos, sense_err, max_err, dt = 0.1, vref = 2):
    # Main algorithm
    all_states = []
    # env0 = check_env()
    env0 = check_env()
    O, goal = env0
    # pos = get_pos()
    pos = initial_pos
    # ref_controller = get_ref_controller()
    Theta = create_Theta(initial_pos, sense_err)
    ref_controller = get_xref_yices(Theta, goal, O, 100, bloating, sense_err, N_min = 1)
    pt1 = [int(i) for i in ref_controller[0]]
    pt2 = [int(i) for i in ref_controller[1]]

    t, ref_states = get_xref(pt1, pt2, vref)
    # While pos0 not in goal:
    status = 'not done'
    i=0
    while status != 'done':
        # Set the time step
        t_step = [0, dt]
        i+=1
        # Update the position of the model
        uref = [vref, 0]
        qref = ref_states[i]
        q0 = pos
        u0 = controller(q0, qref, uref)
        pos = odeint(model, q0, t_step, args = (u0,))[1]
        all_states.append(pos)
        #Check to see if you're in the goal set
        G = pc.Polytope(goal[0], goal[1])
        if pos[0:2] in G:
            print('hit goal')
            print(pos)
            status = 'done'
        # Update the environment
        env1 = check_env()
        O1, goal1 = env1
        new_env = False
        #Check error from ref state to new state
        err = norm(np.array(pos[0:2]) - np.array(qref[0:2]))

        # If abs(ref_state - pos) > max_error or env1 != env0 or pos = waypoint1:
        if i == len(ref_states)-1 or new_env or err >= max_err:
            # ref_controller = get_ref_controller()
            Theta = create_Theta(pos, sense_err)
            ref_controller = get_xref_yices(Theta, goal, O, 100, bloating, sense_err, N_min = 1)
            print('new controller needed')
            i = 0
            print(pos)

            if ref_controller == None:
                print("can't find controller")
                print(pos)
                status = 'done'
            else:
                pt1 = [int(i) for i in ref_controller[0]]
                pt2 = [int(i) for i in ref_controller[1]]

                t, ref_states = get_xref(pt1, pt2, vref)

    return all_states



if __name__ == '__main__':
    initial_pos = [0.5, 0.5, 0]
    xref_path = main(initial_pos, 1, 3)
    x = []
    y = []
    for pt in xref_path:
        x.append(pt[0])
        y.append(pt[1])
    plt.plot(x, y)
    plt.show()
