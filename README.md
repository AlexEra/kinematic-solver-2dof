# kinematic-solver-3dof
kinematic solver for 3dof mainpulator with revolute joints

# Using
main.py contains initial parameters, such as: link lengths, twist angles and joint angles.

# Using with own manipulator
Change in main.py program next parameters (modified Denavit-Hartenberg's parameters):
1. link_lengths;
2. beta - twist angles;
3. thetas - joint angles.

In kinematic_solver.py change all functions to your decison. If your manipulator is 3DoF too, that show_robot() function isn't modify.

Function get_table() create lists with Denavit-Hartenberg's parameters, which corresponds to table.
