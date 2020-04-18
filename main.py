from kinematic_solver import *

link_lengths = [1, 1, 1]
beta = [0, 90, 0, 0]
thetas = [0, 0, 0]

coordinates = forward_kinematics(link_lengths, beta, thetas)

angles, control_coordinates = inverse_kinematics(coordinates, link_lengths)

print_results(coordinates, control_coordinates, grad_to_rad(thetas), angles)

show_robot(link_lengths, beta, angles)
