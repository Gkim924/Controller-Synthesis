import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import math
from itertools import count
#import pandas as pd
import time
from main import *
from benchmarks import *
import pypoman as ppm
from itertools import compress

mouse_x = 0
mouse_y = 0

# program status
prog_status = 'pause'

# obstacle notification
obs_notification = False
obs_start_time = 0
obs_end_time = 0

# Globals for testing using animate function
x = 5
y = 5
theta = 0
delta = 0
j = 0
status = 'not done'
pts = [[0], [0]]
O_local, goal = check_env()
O_global = global_env()
O_local_dyn = []
O_global_dyn = global_env_dyn()
sensed_bool = [False for i in range(len(O_global))]
sensed_dyn_bool = [False for i in range(len(O_global_dyn))]
update_env = False
print_status = 'Computing Controller'
comp_time = 'no computation yet'
curr_time = 0
ref_controller = None
qref = [0, 0, 0]
goal_dir = 'up'

x_trail = []
y_trail = []

# Function that builds current frame for animate function
ref_states = []
def animate(i):
    global x
    global y
    global theta
    global delta
    global j
    global ref_states
    global status
    global pos
    global update_env
    global print_status
    global comp_time
    global curr_time
    global O_global
    global O_local
    global O_global_dyn
    global O_local_dyn
    global sensed_bool
    global sensed_dyn_bool
    global ref_controller
    global prog_status
    global x_trail
    global y_trail
    global qref
    global goal
    global goal_dir

    pos = [x, y, theta]
    x_trail.append(x)
    y_trail.append(y)
    dt = 0.1

    if status == 'not done':
        vref = 2.5
        sense_err = 2
        sensed_bool, update_env = sense_env_bool(O_global, sensed_bool, x, y, sensor_dist = 12)
        O_local = list(compress(O_global, sensed_bool))

        O_global_dyn = move_obstacle(O_global_dyn)
        sensed_dyn_bool = sense_env_bool_dyn(O_global_dyn, sensed_dyn_bool, x, y, sensor_dist = 12)
        O_local_dyn = list(compress(O_global_dyn, sensed_dyn_bool))

        goal, goal_dir = dyn_ob(goal, dt, goal_dir, v = 6.5)
        update_env = True

        if O_local_dyn != []:
            update_env = True

        x_est = x + np.random.normal(0, float(1/5))
        y_est = y + np.random.normal(0, float(1/5))

        if np.linalg.norm(np.array([x_est, y_est]) - np.array(qref[0:2])) >= 0.1 or abs(np.degrees(qref[2]) - np.degrees(theta)) >= 15:
            update_env = True

        # O, goal = check_env()
        if j >= len(ref_states)-2 or update_env:
            # ref_controller = get_ref_controller()
            Theta = create_Theta([x_est, y_est, theta], sense_err)
            add_O = [ob[0] for ob in O_local_dyn]
            O_local = O_local + add_O
            ts = time.time()
            ref_controller = get_xref_yices(Theta, goal, O_local, 10, bloating, sense_err, N_min = 1)
            tf = time.time()
            comp_time = str(round(tf - ts, 5))

            update_env = False
            j = 0

            if ref_controller == None:
                print_status = "Can't Find Controller"
                status = 'done'
            else:
                print_status = 'Controller Found ' + (u'\u2713')
                pts[0] = [pt[0] for pt in ref_controller]
                pts[1] = [pt[1] for pt in ref_controller]

                pt1 = [float(k) for k in ref_controller[0]]
                pt2 = [float(k) for k in ref_controller[1]]

                t, ref_states = get_xref(pt1, pt2, vref)

        if status == 'not done':
            t_step = [0, dt]
            j+=1
            # Update the position of the model
            uref = [vref, 0]
            qref = ref_states[j]
            q0 = pos
            u0 = controller([x_est, y_est, theta], qref, uref)
            pos = odeint(model, q0, t_step, args = (u0,))[1]
            #Check to see if you're in the goal set
            G = pc.Polytope(goal[0], goal[1])
            pos_set = create_Theta(pos, sense_err)
            pos_set = pc.Polytope(pos_set[0],pos_set[1])

            if pos_set <= G:
                print_status = 'Reached Goal'
                status = 'done'
                prog_status = 'pause'
            # Update the environment
            # env1 = check_env()
            # O1, goal1 = env1
            # new_env = False
            #Check error from ref state to new state
            # err = norm(np.array(pos[0:2]) - np.array(qref[0:2]))
            x, y , theta = pos

    else:
        x, y , theta = pos



    # x += 0.1
    # y += 0.01
    # theta += 0.07
    delta += 0

    # Euclidian distance to goal
    goal_b = goal[1]
    goal_x = goal_b[1] - ((goal_b[1] + goal_b[0]) / 2)
    goal_y = goal_b[3] - ((goal_b[3] + goal_b[2]) / 2)
    dist = math.sqrt((goal_x - x)**2 + (goal_y - y)**2)

    # Have to clear axes between frames
    #  fig, ax = plt.subplots()
    plt.cla()

    # Add car to plot
    plot_car(x,y,theta,delta)
    plot_car(x_est, y_est, theta, cabcolor="-r", truckcolor="-r", alpha = 0.5)

    # Add obstacles to plot
    plotObstacles(O_local, goal)
    plotGlobalObstacles(O_global)
    plotGlobalDyn(O_global_dyn)
    plotLocalDyn(O_local_dyn)

    # Add path to plot
    plotPath(pts)
    plt.scatter(x_trail, y_trail, color = 'g', marker = '.', linewidths=1.3)

    # Restore plot info so plot is consistent between frames
    plt.axis([0,100,0,100])
    ax = plt.axes()
    ax.set_facecolor("white")

    plt.text(102,97, 'INFO:', fontsize=12)
    # Show position
    plt.text(102,91,'(x,y): (' + str(round(x,2)) + ',' + str(round(y,2)) + ')', fontsize = 12)

    # Show Yaw angle
    plt.text(102,84,'Yaw: ' + str(round(math.degrees(theta),2)) + (u'\u00b0'), fontsize = 12)

    # Show Steering angle
    plt.text(102,78, 'Steering: ' + str(round(delta,2)) + (u'\u00b0'), fontsize = 12)

    # Show dist to goal
    plt.text(102,71, 'Dist to goal: ' + str(round(dist,2)), fontsize = 12)

    plt.text(102,68, '_____________________________' , fontsize = 12)


    #plt.text(102,68, 'Last Clicked Coords: (' + str(mouse_x) + ',' + str(mouse_y) + ')' , fontsize = 12)

    if print_status == 'Controller Found ' + (u'\u2713'):
        plt.text(102,56, print_status , fontsize = 12, color='green')

    elif print_status == 'Reached Goal':
        plt.text(102,56, print_status + (u'\u2713') , fontsize = 12, color='green')
        prog_status = 'stop'

    elif print_status == "Can't Find Controller":
        plt.text(102,56, print_status + ' ' + (u'\u2718') , fontsize = 12, color='red')
        prog_status = 'stop'

    plt.text(102,44, 'Time to compute path: '+ comp_time + 's' , fontsize = 12)
    plt.text(102,32, 'Elapsed Time: '+ str(curr_time)+'s' , fontsize = 12)

    # make sensor radius
    sensor_radius = plt.Circle((x,y), 12, color='yellow',alpha=0.4)
    ax.add_artist(sensor_radius)

    # Obstacle notification
    global obs_notification, obs_start_time, obs_end_time

    if obs_notification:
        plt.text(50,107, 'A wild OBSTACLE appeared!', fontsize = 16, ha='center', va='center', bbox=dict(boxstyle="round",
                    ec=(0.98,0.91,0.01),
                    fc=(0.98,0.8,0.0),
                    ))

        obs_end_time = time.time()

        if obs_end_time - obs_start_time > 5:
            obs_notification = False


    if prog_status == 'pause':
        plt.text(110,20, "Paused", fontsize=12, ha='center', va='center', bbox=dict(boxstyle="round",
                ec=(1.,0.5,0.5),
                fc=(1.,0.8,0.8),
                ))
        animation.event_source.stop()

    elif prog_status == 'run':

        # running status
        plt.text(110,20, "Running", fontsize=12, ha='center', va='center', bbox=dict(boxstyle="round",
                ec=(0.5,1.0,0.5),
                fc=(0.8,1.0,0.8),
                ))
        curr_time = round(curr_time + dt, 1)

    elif prog_status == 'stop':

        # stopped status
        plt.text(110,20, "Finished", fontsize=12, ha='center', va='center', bbox=dict(boxstyle="square",
                ec=(0.0,0.0,0.0),
                fc=(0.8,1.0,0.8),
                ))
        curr_time = round(curr_time + dt, 1)
        animation.event_source.stop()




def on_click(event):

       global mouse_x, mouse_y
       global O_global
       global update_env

       global obs_notification
       global obs_start_time
       global obs_end_time

       mouse_x = round(event.xdata,4)
       mouse_y = round(event.ydata,4)

       # O.append(add_static_obs(O, size = 5))
       O_global.append(create_Theta([mouse_x, mouse_y], 5*sqrt(2)))
       sensed_bool.append(False)

       #print('MOUSE:(' + str(round(event.x,4)), str(round(event.y,4)) + ')')

       # set obstacle notification
       obs_start_time = time.time()

       obs_notification = True



def on_key(event):

    global prog_status

    print('Pressed: ', event.key)

    if event.key == "r":

        if prog_status == 'pause':
            prog_status = 'run'
            animation.event_source.start()

        elif prog_status == 'run':
            prog_status = 'pause'



    print('Prog Status:', prog_status)


# Draw the car
def plot_car(x, y, yaw, steer=0.0, cabcolor="-r", truckcolor="-k", alpha = 1):

    # Vehicle parameters
    LENGTH = 0.4  # [m]
    WIDTH = 0.2  # [m]
    BACKTOWHEEL = 0.1  # [m]
    WHEEL_LEN = 0.03  # [m]
    WHEEL_WIDTH = 0.02  # [m]
    TREAD = 0.07  # [m]
    WB = 0.25  # [m] wheel base

    outline = np.array([[-BACKTOWHEEL, (LENGTH - BACKTOWHEEL), (LENGTH - BACKTOWHEEL),
                         -BACKTOWHEEL, -BACKTOWHEEL],
                        [WIDTH / 2, WIDTH / 2, - WIDTH / 2, -WIDTH / 2, WIDTH / 2]])

    fr_wheel = np.array([[WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
                         [-WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD, WHEEL_WIDTH -
                          TREAD, WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD]])


    rr_wheel = np.copy(fr_wheel)

    fl_wheel = np.copy(fr_wheel)
    fl_wheel[1, :] *= -1
    rl_wheel = np.copy(rr_wheel)
    rl_wheel[1, :] *= -1

    # Rotates the body and back wheels
    Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                     [-math.sin(yaw), math.cos(yaw)]])

    # Rotates the front wheels
    Rot2 = np.array([[math.cos(steer), math.sin(steer)],
                     [-math.sin(steer), math.cos(steer)]])

    # Translate wheels to origin, do steering rotation, translate wheels back
    fr_wheel[1, :] += 0.07
    fl_wheel[1, :] -= 0.07

    fr_wheel = (fr_wheel.T.dot(Rot2)).T
    fl_wheel = (fl_wheel.T.dot(Rot2)).T

    # Translate wheels to correct positions
    fr_wheel[1, :] -= 0.07
    fl_wheel[1, :] += 0.07

    fr_wheel[0, :] += WB
    fl_wheel[0, :] += WB

    fr_wheel = (fr_wheel.T.dot(Rot1)).T
    fl_wheel = (fl_wheel.T.dot(Rot1)).T

    outline = (outline.T.dot(Rot1)).T
    rr_wheel = (rr_wheel.T.dot(Rot1)).T
    rl_wheel = (rl_wheel.T.dot(Rot1)).T

    scale_by = 15

    outline[0, :] *= scale_by
    outline[0, :] += x

    outline[1, :] *= scale_by
    outline[1, :] += y

    fr_wheel[0, :] *= scale_by
    fr_wheel[0, :] += x

    fr_wheel[1, :] *= scale_by
    fr_wheel[1, :] += y

    rr_wheel[0, :] *= scale_by
    rr_wheel[0, :] += x

    rr_wheel[1, :] *= scale_by
    rr_wheel[1, :] += y

    fl_wheel[0, :] *= scale_by
    fl_wheel[0, :] += x

    fl_wheel[1, :] *= scale_by
    fl_wheel[1, :] += y

    rl_wheel[0, :] *= scale_by
    rl_wheel[0, :] += x

    rl_wheel[1, :] *= scale_by
    rl_wheel[1, :] += y


    plt.plot(np.array(outline[0, :]).flatten(),
             np.array(outline[1, :]).flatten(), truckcolor, alpha = alpha)
    plt.plot(np.array(fr_wheel[0, :]).flatten(),
             np.array(fr_wheel[1, :]).flatten(), truckcolor, alpha = alpha)
    plt.plot(np.array(rr_wheel[0, :]).flatten(),
             np.array(rr_wheel[1, :]).flatten(), truckcolor, alpha = alpha)
    plt.plot(np.array(fl_wheel[0, :]).flatten(),
             np.array(fl_wheel[1, :]).flatten(), truckcolor, alpha = alpha)
    plt.plot(np.array(rl_wheel[0, :]).flatten(),
             np.array(rl_wheel[1, :]).flatten(), truckcolor, alpha = alpha)
    plt.plot(x, y, "*", alpha = alpha)


def plotObstacles(O, goal):
    # O, goal = check_env()
    for obs in O:
        ppm.polygon.plot_polygon(ppm.duality.compute_polytope_vertices(obs[0], obs[1]), alpha=0.6, color='r', linestyle='solid', fill=True, linewidth=None)
    ppm.polygon.plot_polygon(ppm.duality.compute_polytope_vertices(goal[0], goal[1]), alpha=0.6, color='g', linestyle='solid', fill=True, linewidth=None)
# Show the path in plot, can update path via this function when obstacle appears

def plotGlobalObstacles(O):
    # O, goal = check_env()
    for obs in O:
        ppm.polygon.plot_polygon(ppm.duality.compute_polytope_vertices(obs[0], obs[1]), alpha=0.2, color='r', linestyle='solid', fill=True, linewidth=None)

def plotGlobalDyn(O):
    # O, goal = check_env()
    for obs, dir in O:
        ppm.polygon.plot_polygon(ppm.duality.compute_polytope_vertices(obs[0], obs[1]), alpha=0.2, color='r', linestyle='solid', fill=True, linewidth=None)

def plotLocalDyn(O):
    # O, goal = check_env()
    for obs, dir in O:
        ppm.polygon.plot_polygon(ppm.duality.compute_polytope_vertices(obs[0], obs[1]), alpha=0.6, color='r', linestyle='solid', fill=True, linewidth=None)

def move_obstacle(O_dyn):
    for j in range(len(O_dyn)):
        O, dir = O_dyn[j]
        O_dyn[j] = dyn_ob(O, 0.1, dir)

    return O_dyn


def plotPath(pts):

    plt.plot(pts[0], pts[1], 'k', linewidth=1.5, linestyle='dashed')
    plt.scatter(pts[0], pts[1], color = 'k')



fig, ax = plt.subplots(figsize=(10,6))
ax.set(xlim=(0,100),ylim=(0,100))
ax.set_facecolor("grey")
plt.subplots_adjust(right=0.65)



#(-0.5,-1.5,0.2,0.5)

# mouse handler
cid = fig.canvas.mpl_connect('button_press_event', on_click)

# key handler (R - run/pause)
pid = fig.canvas.mpl_connect('key_press_event', on_key)

animation = FuncAnimation(fig, func=animate, interval=10)

plt.show()
