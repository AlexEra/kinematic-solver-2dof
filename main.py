from kinematic_solver import *

link_lengths = [1, 1, 1]
beta2 = 90
thetas = [0, 0, 45]

coordinates = forward_kinematics(link_lengths, beta2, thetas)

angles, control_coordinates = inverse_kinematics(coordinates, link_lengths)

print_results(coordinates, control_coordinates, grad_to_rad(thetas), angles)

show_robot(link_lengths, beta2, angles)
