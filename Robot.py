import numpy as np
from Task import Task
import copy
from Config import *

class Robot:
    """
    Constructs an instantiate of the 3-DOF wheeled Robot navigating on a
    2D plane

    Parameters
    ----------
    name : (string)
        The name of the robot
    color : (string)
        The color of the robot
    max_linear_speed : (float)
        The maximum linear speed that the robot can go
    max_angular_speed : (float)
        The maximum angular speed that the robot can rotate about its vertical
        axis
    path_finder_controller : (PathFinderController)
        A configurable controller to finds the path and calculates command
        linear and angular velocities.
    pose : [x, y, theta]
        Current pose
    list_tasks : list[Task]
        The sequence of tasks the agent need to perform
    count_task : 
        Used to count the current task
    """
    def __init__(self, id, color, max_linear_speed, max_angular_speed,
                 path_finder_controller):
        self.id = id
        self.color = color
        self.MAX_LINEAR_SPEED = max_linear_speed
        self.MAX_ANGULAR_SPEED = max_angular_speed
        self.path_finder_controller = path_finder_controller
        self.x_traj = []
        self.y_traj = []
        self.pose = [0, 0, 0]
        self.pose_start = [0, 0, 0]
        self.list_tasks: list[Task] = []
        self.is_at_target = False
        self.count_task = 0
        print("Initialed the robot with id %.0f. \n" % (id))


    def set_start_poses(self, pose_start):
        """
        Sets the start and target positions of the robot

        Parameters
        ----------
        pose_start : (Pose)
            Start postion of the robot (see the Pose class)
        pose_target : (Pose)
            Target postion of the robot (see the Pose class)
        """
        self.pose_start = pose_start
        self.pose = pose_start

        print("Set starting position for the robot id %.0f." % (self.id))
        print("Initial x: %.2f m\nInitial y: %.2f m\nInitial theta: %.2f rad." %
            (pose_start[0], pose_start[1], pose_start[2]))


    def add_task(self, new_task):

        self.list_tasks.append(copy.copy(new_task))

        print("Set target position for the robot id %.0f.\n" % (self.id))
        # print("Target x: %.2f m\nTarget y: %.2f m\n" %
        #     (ta[0], pose_target[1]))
        # self.find_next_task()
        if len(self.list_tasks) == 1:
            self.current_target = self.list_tasks[0]

    def find_next_task(self):

        if len(self.list_tasks) == 0:
            print("The agent's task list is idle. \n")
            return
        if len(self.list_tasks) == self.count_task:
            self.is_at_target = True
            return
        
        self.current_target = self.list_tasks[self.count_task]

        if self.current_target.is_done == True:
            self.is_at_target = False
            self.count_task += 1
            if len(self.list_tasks) == self.count_task:
                self.is_at_target = True
                return
            self.current_target = self.list_tasks[self.count_task]

    def move(self, dt):
        """
        Moves the robot for one time step increment

        Parameters
        ----------
        dt : (float)
            time step
        """

        self.x_traj.append(self.pose[0])
        self.y_traj.append(self.pose[1])

        rho, linear_velocity, angular_velocity = \
            self.path_finder_controller.calc_control_command(
                self.current_target.position[0] - self.pose[0],
                self.current_target.position[1] - self.pose[1],
                self.pose[2])

        if rho < AT_TARGET_ACCEPTANCE_THRESHOLD:
            # self.is_at_target = True
            self.find_next_task()
            self.current_target.new_robot_comes(self.id)
            print("Robot id: %.0f is at its target. \n" % (self.id))

        if abs(linear_velocity) > self.MAX_LINEAR_SPEED:
            linear_velocity = (np.sign(linear_velocity)
                               * self.MAX_LINEAR_SPEED)

        if abs(angular_velocity) > self.MAX_ANGULAR_SPEED:
            angular_velocity = (np.sign(angular_velocity)
                                * self.MAX_ANGULAR_SPEED)

        self.pose[2] = self.pose[2] + angular_velocity * dt
        self.pose[0] = self.pose[0] + linear_velocity * \
            np.cos(self.pose[2]) * dt
        self.pose[1] = self.pose[1] + linear_velocity * \
            np.sin(self.pose[2]) * dt
