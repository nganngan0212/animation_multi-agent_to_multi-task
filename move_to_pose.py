"""

Move to specified pose

Author: Daniel Ingram (daniel-s-ingram)
        Atsushi Sakai (@Atsushi_twi)
        Seied Muhammad Yazdian (@Muhammad-Yazdian)

P. I. Corke, "Robotics, Vision & Control", Springer 2017, ISBN 978-3-319-54413-7

"""

import matplotlib.pyplot as plt
import numpy as np
from random import random

# the library for read mat files
import scipy.io as sio 

from Robot import Robot as Robot
import Task
import plot
import PathFinderController


# Simulation parameters
AT_TARGET_ACCEPTANCE_THRESHOLD = 0.1

# simulation parameters
my_controller = PathFinderController.PathFinderController(9, 9)
TIME_STEP = 0.01
plot.dt = TIME_STEP

# Robot specifications
MAX_LINEAR_SPEED = 15
MAX_ANGULAR_SPEED = 7

show_animation = True

def run_simulation(robots):
    
    
    x = x_start
    y = y_start
    theta = theta_start

    for i in range(len(x_goal)):

        # print("Goal x: %.2f m\nGoal y: %.2f m\n" %
        #     (x_goal[i], y_goal[i]))

        x_diff = x_goal[i] - x
        y_diff = y_goal[i] - y

        x_traj, y_traj = [], []

        rho = np.hypot(x_diff, y_diff)       # hypot() return Euclidean norm of the vector
        while rho > 0.001:
            x_traj.append(x)
            y_traj.append(y)

            x_diff = x_goal[i] - x
            y_diff = y_goal[i] - y

            rho, v, w = my_controller.calc_control_command(
                x_diff, y_diff, theta)

            if abs(v) > MAX_LINEAR_SPEED:
                v = np.sign(v) * MAX_LINEAR_SPEED       # sign() return dau cua mot so: am (return -1), duong (return 0), 0 (return 0)

            if abs(w) > MAX_ANGULAR_SPEED:
                w = np.sign(w) * MAX_ANGULAR_SPEED

            theta = theta + w * dt
            x = x + v * np.cos(theta) * dt
            y = y + v * np.sin(theta) * dt

            if show_animation:  # pragma: no cover
                
                plt.cla()

                plt.arrow(x_start, y_start, np.cos(theta_start),
                        np.sin(theta_start), color='r', width=0.1)
                # plt.arrow(x_goal, y_goal, np.cos(theta_goal),
                #           np.sin(theta_goal), color='g', width=0.1)
                plt.plot(x_goal, y_goal, 'ro')
                plot.plot_vehicle(x, y, theta, x_traj, y_traj)

def main():

    sol = sio.loadmat('ResultM1.mat')

    # x_start = 1
    # y_start = 1
    # theta_start = 0
    # x_goal = [18, 5]
    # y_goal = [15, 10]
    # theta_goal = 2 * np.pi * random() - np.pi
    
    robot1 = Robot(1, 'y', 12, 5, my_controller)
    robot2 = Robot(2, 'y', 10, 4, my_controller)

    robot1_start = [5, 5, 0]    # x, y and theta
    robot1.set_start_poses(robot1_start)

    robot2_start = [5, 7, 0]
    robot2.set_start_poses(robot2_start)

    set_robots: list[Robot] = [robot1, robot2]  # ok
    
    # run_simulation(set_robots)


if __name__ == '__main__':
    main()
