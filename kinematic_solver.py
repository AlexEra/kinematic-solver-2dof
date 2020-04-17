import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from create_transformation_mtx import *


def forward_kinematics(link_lengths, beta2, thetas):
    """Compute transform matrix and coordinates"""
    a_vec, beta_vec, d_vec, theta_vec = get_table(link_lengths, beta2, thetas)
    t_mtx = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    for i in range(0, len(a_vec)):
        t_mtx = np.dot(t_mtx, build_transformation_matrix(a_vec[i], beta_vec[i], d_vec[i], theta_vec[i]))
    return t_mtx[0][3], t_mtx[1][3], t_mtx[2][3]  # return x, y, z


def get_table(link_lengths, beta2, thetas):
    a_vec = [0, 0, link_lengths[1], link_lengths[2]]  # lengths of links
    beta_vec = grad_to_rad([0, beta2, 0, 0])  # twist angles
    d_vec = [link_lengths[0], 0, 0, 0]  # links offset
    theta_vec = grad_to_rad([thetas[0], thetas[1], thetas[2], 0])  # angles in joint
    return a_vec, beta_vec, d_vec, theta_vec


def inverse_kinematics(coordinates, link_len):
    """Compute angles in joints with help anti gradient"""
    x_t, y_t, z_t = coordinates
    l_1, l_2, l_3 = link_len[0], link_len[1], link_len[2]
    theta_1 = math.atan2(y_t, x_t)
    theta_2, theta_3 = 0, 0
    x, y, z = calculate_coordinates(link_len, theta_1, theta_2, theta_3)
    epsilon_x, epsilon_y, epsilon_z = 0.001, 0.001, 0.001  # accuracy coefficients
    gama_1, gama_2, gama_3 = 0.005, 0.005, 0.005  # coefficient of anti gradient
    for i in range(1, 100000):
        if (abs(x_t - x) < epsilon_x) and (abs(y_t - y) < epsilon_y) and (abs(z_t - z) < epsilon_z):
            break
        dx2 = -math.cos(theta_1) * (l_2 * math.sin(theta_2) + l_3 * math.sin(theta_2 + theta_3))
        dx3 = -l_3 * math.cos(theta_1) * math.sin(theta_2 + theta_3)
        dy2 = -math.sin(theta_1) * (l_2 * math.sin(theta_2) + l_3 * math.sin(theta_2 + theta_3))
        dy3 = -l_3 * math.sin(theta_1) * math.sin(theta_2 + theta_3)
        dz2 = l_2 * math.cos(theta_2) + l_3 * math.cos(theta_2 + theta_3)
        dz3 = l_3 * math.cos(theta_2 + theta_3)
        theta_2 = theta_2 + gama_2 * ((x_t - x) * dx2 + (y_t - y) * dy2 + (z_t - z) * dz2)  # calculate angle in joint 2
        theta_3 = theta_3 + gama_3 * ((x_t - x) * dx3 + (y_t - y) * dy3 + (z_t - z) * dz3)  # calculate angle in joint 3
        x, y, z = calculate_coordinates(link_len, theta_1, theta_2, theta_3)
    angles = [theta_1, theta_2, theta_3]
    return angles, (x, y, z)


def calculate_coordinates(lnk, t_1, t_2, t_3):
    p_1 = math.cos(t_1) * (lnk[1] * math.cos(t_2) + lnk[2] * math.cos(t_2 + t_3))
    p_2 = math.sin(t_1) * (lnk[1] * math.cos(t_2) + lnk[2] * math.cos(t_2 + t_3))
    p_3 = lnk[0] + lnk[1] * math.sin(t_2) + lnk[2] * math.sin(t_2 + t_3)
    return p_1, p_2, p_3


def print_results(coordinates, control_coordinates, theta, angles):
    """Print results"""
    print('goal coordinates: ', coordinates)
    print('calculated coordinates: ', control_coordinates)
    print('goal angles: ', theta[0] * 180 / math.pi, theta[1] * 180 / math.pi, theta[2] * 180 / math.pi)
    print("calculated angles: ", angles[0] * 180 / math.pi, angles[1] * 180 / math.pi, angles[2] * 180 / math.pi)


def show_robot(link_lengths, beta2, thetas):
    a_vec, beta_vec, d_vec, theta_vec = get_table(link_lengths, beta2, rad_to_grad(thetas))
    t_01 = build_transformation_matrix(a_vec[0], beta_vec[0], d_vec[0], theta_vec[0])
    t_02 = np.dot(t_01, build_transformation_matrix(a_vec[1], beta_vec[1], d_vec[1], theta_vec[1]))
    t_03 = np.dot(t_02, build_transformation_matrix(a_vec[2], beta_vec[2], d_vec[2], theta_vec[2]))
    t_04 = np.dot(t_03, build_transformation_matrix(a_vec[3], beta_vec[3], d_vec[3], theta_vec[3]))
    x_points = [0, t_01[0, 3], t_02[0, 3], t_03[0, 3], t_04[0, 3]]
    y_points = [0, t_01[1, 3], t_02[1, 3], t_03[1, 3], t_04[1, 3]]
    z_points = [0, t_01[2, 3], t_02[2, 3], t_03[2, 3], t_04[2, 3]]
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
