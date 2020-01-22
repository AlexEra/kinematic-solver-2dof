import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def grad_to_rad(theta):
    """Transform angles to radians"""
    for i, ang in enumerate(theta):
        theta[i] = ang * math.pi / 180
    return theta


def build_transform_matrix(theta, link_len, link_offs):
    """Transform matrix for {1} relatively {0}"""
    t_1 = np.array([[math.cos(theta[0]), -math.sin(theta[0]), 0, 0],
                    [math.sin(theta[0]), math.cos(theta[0]), 0, 0],
                    [0, 0, 1, link_len[0]],
                    [0, 0, 0, 1]])

    """Transform matrix for {2} relatively {1}"""
    t_2 = np.array([[math.cos(theta[1]), -math.sin(theta[1]), 0, 0],
                    [0, 0, -1, 0],
                    [math.sin(theta[1]), math.cos(theta[1]), 0, 0],
                    [0, 0, 0, 1]])

    """Transform matrix for {3} relatively {2}"""
    t_3 = np.array([[math.cos(theta[2]), -math.sin(theta[2]), 0, link_offs[1]],
                    [math.sin(theta[2]), math.cos(theta[2]), 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])

    """Transform matrix for {4} relatively {3}"""
    t_4 = np.array([[1, 0, 0, link_offs[2]],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])
    return t_1, t_2, t_3, t_4


def forward_kinematics(t_1, t_2, t_3, t_4):
    """Compute transform matrix and coordinates"""
    t_02 = np.dot(t_1, t_2)
    t_03 = np.dot(t_02, t_3)
    t_04 = np.dot(t_03, t_4)
    x = t_04[0][3]
    y = t_04[1][3]
    z = t_04[2][3]
    return (x, y, z), (t_02, t_03, t_04)


def inverse_kinematics(coordinates, link_len):
    """Compute angles in joints"""
    x, y, z = coordinates
    l_1, l_2, l_3 = link_len[0], link_len[1], link_len[2]
    angle_1 = math.atan2(y, x)
    angle_3 = -math.acos((l_2 ** 2 + l_3 ** 2 - x ** 2 - y ** 2 - (z - l_1) ** 2) / (2 * l_2 * l_3)) + math.pi
    phi = math.acos((2 * l_1 ** 2 - 2 * z * l_1) / (2 * math.sqrt(x ** 2 + y ** 2 + (z - l_1) ** 2)) * l_1)
    gama = math.acos((l_2 ** 2 - l_3 ** 2 + x ** 2 + y ** 2 + (z - l_1) ** 2) / (
            2 * l_2 * math.sqrt(x ** 2 + y ** 2 + (z - l_1) ** 2)))
    if (angle_3 == 0) or (angle_3 < 0):
        angle_2 = phi + gama - math.pi / 2
    else:
        angle_2 = phi - gama - math.pi / 2

    """Computing the coordinates of the end-effector"""
    x_r = l_3 * (math.cos(angle_1) * math.cos(angle_2) * math.cos(angle_3) - math.cos(angle_1) * math.sin(angle_2) *
                 math.sin(angle_3)) + l_2 * math.cos(angle_1) * math.cos(angle_2)
    y_r = l_3 * (math.cos(angle_2) * math.cos(angle_3) * math.sin(angle_1) - math.sin(angle_1) * math.sin(angle_2) *
                 math.sin(angle_3)) + l_2 * math.sin(angle_1) * math.cos(angle_2)
    z_r = l_1 + l_3 * (math.cos(angle_2) * math.sin(angle_3) + math.sin(angle_2) * math.cos(angle_3)) + \
          l_2 * math.sin(angle_2)

    return (angle_1, angle_2, angle_3), (x_r, y_r, z_r)


def print_results(coordinates, control_coordinates, theta, angles):
    """Print results"""
    print('goal coordinates: ', coordinates)
    print('calculated coordinates: ', control_coordinates)
    print('goal angles: ', theta[0] * 180 / math.pi, theta[1] * 180 / math.pi, theta[2] * 180 / math.pi)
    print("calculated angles: ", angles[0] * 180 / math.pi, angles[1] * 180 / math.pi, angles[2] * 180 / math.pi)


def show_robot(angles, link_len, link_offs):
    t_1, t_2, t_3, t_4 = build_transform_matrix(angles, link_len, link_offs)
    crd, mtx = forward_kinematics(t_1, t_2, t_3, t_4)
    t_02, t_03, t_04 = mtx
    x_points = [0, t_1[0, 3], t_02[0, 3], t_03[0, 3], t_04[0, 3]]
    y_points = [0, t_1[1, 3], t_02[1, 3], t_03[1, 3], t_04[1, 3]]
    z_points = [0, t_1[2, 3], t_02[2, 3], t_03[2, 3], t_04[2, 3]]
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    Axes3D.plot(ax, xs=x_points, ys=y_points, zs=z_points)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.scatter(x_points, y_points, z_points, color='orange', marker='o')
    plt.xlim(0, 2)
    plt.ylim(0, 2)
    ax.set_zlim(0, 3)
    plt.show()
