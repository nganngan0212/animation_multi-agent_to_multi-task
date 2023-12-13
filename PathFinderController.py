import numpy as np

class PathFinderController:

    def __init__(self, Kp_rho, Kp_alpha):
        self.Kp_rho = Kp_rho
        self.Kp_alpha = Kp_alpha
        # self.Kp_beta = Kp_beta

    def calc_control_command(self, x_diff, y_diff, theta):

        rho = np.hypot(x_diff, y_diff)      # return square root of x_diff and y_diff
        # alpha = (np.arctan2(y_diff, x_diff)
        #          - theta + np.pi) % (2 * np.pi) - np.pi       # adding  % (2 * np.pi) - np.pi  to normalize the angle to in range -pi to pi rad
        # beta = (theta_goal - theta - alpha + np.pi) % (2 * np.pi) - np.pi
        alpha = (np.arctan2(y_diff, x_diff)-theta)
        v = self.Kp_rho * rho
        w = self.Kp_alpha * alpha

        if alpha > np.pi / 2 or alpha < -np.pi / 2:
            v = -v

        return rho, v, w
