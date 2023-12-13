
import matplotlib.pyplot as plt
import numpy as np
from random import random

# the library for read mat files
# import scipy.io as sio 
from Pose import Pose
from Robot import Robot
from Task import Task
import plot
from PathFinderController import PathFinderController
from Config import *

my_controller = PathFinderController(9, 9)
plot.dt = TIME_STEP


simulation_running = True
all_robots_are_at_target = False

def run_simulation(robots, tasks):
    """Simulates all robots simultaneously"""

    global all_robots_are_at_target
    global simulation_running

    robot_id = []
    for instance in robots:
        robot_id.append(instance.id)

    time = 0
    while simulation_running: # and time < TIME_DURATION:
        time += TIME_STEP
        # robots_are_at_target = []
        tasks_are_done = []

        for instance in robots:
            if not instance.is_at_target:
                instance.move(TIME_STEP)
            # robots_are_at_target.append(instance.is_at_target)

        for instance in tasks:
            tasks_are_done.append(instance.is_done)

        if all(tasks_are_done):
            print("All tasks are done!!!!!")
            plt.pause(5)
            simulation_running = False

        if SHOW_ANIMATION:
            plt.cla()
            plt.xlim(0, PLOT_WINDOW_SIZE_X)
            plt.ylim(0, PLOT_WINDOW_SIZE_Y)

            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])

            for instance in robots:

                plt.arrow(instance.pose_start[0], instance.pose_start[1], 
                          np.cos(instance.pose_start[2]),
                          np.sin(instance.pose_start[2]), 
                          color='r', 
                          width=0.1)
                plot.plot_vehicle(instance.pose[0],
                             instance.pose[1],
                             instance.pose[2],
                             instance.x_traj,
                             instance.y_traj)
            for task in tasks:
                # print("Task %.0f: state is "  %(task.id) + str(task.is_done))
                if task.is_done == True:
                    plt.plot(task.position[0], 
                            task.position[1], 
                            'go')
                else:
                    plt.plot(task.position[0], 
                            task.position[1], 
                            'ro')

            plt.pause(TIME_STEP)

def main():

    # sol = sio.loadmat('ResultM1.mat')

    # x_start = 1
    # y_start = 1
    # theta_start = 0
    # x_goal = [18, 5]
    # y_goal = [15, 10]
    # theta_goal = 2 * np.pi * random() - np.pi
    
    robot1 = Robot(1, 'y', 12, 5, my_controller)
    robot2 = Robot(2, 'y', 10, 4, my_controller)

    task1 = Task(1,[10, 10], 1)
    task2 = Task(2,[15, 5], 1)
    task3 = Task(3, [25, 25], 1)

    robot1_start = [5, 5, 0]    # x, y and theta
    robot1.set_start_poses(robot1_start)
    robot1.add_task(task1)

    robot2_start = [5, 7, 0]
    robot2.set_start_poses(robot2_start)
    robot2.add_task(task2)
    robot2.add_task(task3)

    robots: list[Robot] = [robot1, robot2]  # ok
    tasks: list[Task] = [task1, task2, task3]  # ok
    
    run_simulation(robots, tasks)


if __name__ == '__main__':
    main()
