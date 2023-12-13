import numpy as np
import matplotlib.pyplot as plt

def transformation_matrix(x, y, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta), x],
        [np.sin(theta), np.cos(theta), y],
        [0, 0, 1]
    ])


def plot_vehicle(x, y, theta, x_traj, y_traj):  # pragma: no cover
    
    # Corners of triangular vehicle when pointing to the right (0 radians)
    p1_i = np.array([0.5, 0.25, 1]).T      # n.aray().T ma tran chuyen vi
    p2_i = np.array([-0.5, 0.25, 1]).T
    p3_i = np.array([-0.5, -0.25, 1]).T
    p4_i = np.array([0.5, -0.25, 1]).T

    T = transformation_matrix(x, y, theta)
    p1 = np.matmul(T, p1_i)
    p2 = np.matmul(T, p2_i)
    p3 = np.matmul(T, p3_i)
    p4 = np.matmul(T, p4_i)

    plt.plot([p1[0], p2[0]], [p1[1], p2[1]], 'k-')
    plt.plot([p2[0], p3[0]], [p2[1], p3[1]], 'k-')
    plt.plot([p3[0], p4[0]], [p3[1], p4[1]], 'k-')
    plt.plot([p4[0], p1[0]], [p4[1], p1[1]], 'k-')

    plt.arrow(x, y, np.cos(theta),
                        np.sin(theta), color='r', width=0.1)

    plt.plot(x_traj, y_traj, 'b--')

    # for stopping simulation with the esc key.
    plt.gcf().canvas.mpl_connect(
        'key_release_event',
        lambda event: [exit(0) if event.key == 'escape' else None])

    plt.xlim(0, 30)
    plt.ylim(0, 30)

    # plt.pause(dt)

