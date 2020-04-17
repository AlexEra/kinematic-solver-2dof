import numpy as np
import math


def grad_to_rad(theta):
    """Transform angles to radians"""
    for i, ang in enumerate(theta):
        theta[i] = ang * math.pi / 180
    return theta


def rad_to_grad(theta):
    """Transform angles to grad"""
    for i, ang in enumerate(theta):
        theta[i] = ang * 180 / math.pi
    return theta


def build_transformation_matrix(a, beta, d, theta):
    """Building the transformation matrix"""
    t_mtx = np.array([[math.cos(theta), -math.sin(theta), 0, a],
                      [math.sin(theta) * math.cos(beta), math.cos(theta) * math.cos(beta), -math.sin(beta),
                       -math.sin(beta) * d],
                      [math.sin(theta) * math.sin(beta), math.cos(theta) * math.sin(beta), math.cos(beta),
                       math.cos(beta) * d],
                      [0, 0, 0, 1]])
    return t_mtx
