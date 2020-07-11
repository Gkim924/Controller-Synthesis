import numpy as np
import polytope as pc

def add_static_obs(O, size = 5):
    # Adds a random square static obstacle to "O" with side length "size"
    A = np.array([[-1, 0], [1, 0], [0, -1], [0, 1]])
    xmin = np.random.randint(100)
    ymin = np.random.randint(100)
    b = np.array([xmin, xmin + size, ymin, ymin + size])

    O.append((A, b))
    return O

def add_moving_obs(O, t, size = 5): # t NEEDS TO BE GLOBAL
    A = np.array([[-1, 0], [1, 0], [0, -1], [0, 1]])
    xmin = np.random.randint(100)
    ymin = np.random.randint(100)
    motion = np.random.randint(4)
    curr_t = t
    speed = 1
    if motion == 0:
        b = np.array([xmin + ((t-curr_t)*speed), xmin + size + ((t-curr_t)*speed), ymin, ymin + size])
    if motion == 1:
        b = np.array([xmin - ((t-curr_t)*speed), xmin + size - ((t-curr_t)*speed), ymin, ymin + size])
    if motion == 2:
        b = np.array([xmin, xmin + size, ymin + ((t-curr_t)*speed), ymin + size + ((t-curr_t)*speed)])
    if motion == 3:
        b = np.array([xmin, xmin + size, ymin - ((t-curr_t)*speed), ymin + size - ((t-curr_t)*speed)])

    O.append((A, b))
    return O

def add_obs_click(O, clickx, clicky, size = 5):
    # Adds a static obstacle of size centered around click coordinates
    A = np.array([[-1, 0], [1, 0], [0, -1], [0, 1]])
    xmin = clickx - (size/2)
    ymin = clicky - (size/2)
    b = np.array([xmin, xmin + size, ymin, ymin + size])

    O.append((A, b))
    return O

def sense_env_bool(O, O_sensed_bool, robotx, roboty, sensor_dist = 5):
    # This is horrible but not worth solving quadratic minimization problem since in 2d with rectangles
    update_env = False
    for i in range(len(O)):
        A, b = O[i][0], O[i][1]
        fat_b = np.array([[b[0] + sensor_dist], [b[1] + sensor_dist], [b[2] + sensor_dist], [b[3] + sensor_dist]])
        p = pc.Polytope(A, fat_b)
        if [robotx, roboty] in p and O_sensed_bool[i] == False:
            # print('obstacle detected')
            O_sensed_bool[i] = True
            update_env = True
        # else:
            # print('not detected')

    return O_sensed_bool, update_env

def sense_env_bool_dyn(O, O_dyn_sensed_bool, robotx, roboty, sensor_dist = 5):
    # This is horrible but not worth solving quadratic minimization problem since in 2d with rectangles
    for i in range(len(O)):
        ob, dir = O[i][0], O[i][1]
        A, b = ob[0], ob[1]
        fat_b = np.array([[b[0] + sensor_dist], [b[1] + sensor_dist], [b[2] + sensor_dist], [b[3] + sensor_dist]])
        p = pc.Polytope(A, fat_b)
        if [robotx, roboty] in p and O_dyn_sensed_bool[i] == False:
            O_dyn_sensed_bool[i] = True
        # else:
            # print('not detected')

    return O_dyn_sensed_bool
