from kinematic_solver import *

theta = [0, 0, 0]  # three input angles
link_lengths = [1, 1, 1]  # lengths of links
link_offset = [0, 1, 1]  # links offset

theta = grad_to_rad(theta)

trans_mtx_1, trans_mtx_2, trans_mtx_3, trans_mtx_4 = build_transform_matrix(theta, link_lengths, link_offset)

coordinates, _ = forward_kinematics(trans_mtx_1, trans_mtx_2, trans_mtx_3, trans_mtx_4)

angles, control_coordinates = inverse_kinematics(coordinates, link_lengths)

print_results(coordinates, control_coordinates, theta, angles)

show_robot(angles, link_lengths, link_offset)
