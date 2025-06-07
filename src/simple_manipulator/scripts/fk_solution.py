import sympy as sym
import numpy as np
# TODO: use sympy to re-write your numerical forward kinematics solution, and
# get its jacobian with respect to only the end-effector position

def fk_solution():

    sym.init_printing(use_unicode=True)
    
    # TODO: use these symbols for your solution
    theta_1, theta_2, theta_3, \
            theta_4, theta_5, theta_6 = sym.symbols("theta_1 theta_2 theta_3 theta_4 theta_5 theta_6")

    
    # TODO: write down you forward kinematics solution here using sympy.
    # NOTE: you should look up the documentation in sympy for:
    #   - symbols
    #   - Matrix
    #   - jacobian
    #   - lambdify
    T01 = sym.Matrix([[ sym.cos(theta_1),-sym.sin(theta_1),                 0,    0],
                      [ sym.sin(theta_1), sym.cos(theta_1),                 0,    0],
                      [                0,                0,                 1,  0.1],
                      [                0,                0,                 0,    1]])
    
    T12 = sym.Matrix([[ sym.cos(theta_2),                0,  sym.sin(theta_2),    0],
                      [                0,                1,                 0, 0.15],
                      [-sym.sin(theta_2),                0,  sym.cos(theta_2),  0.2],
                      [                0,                0,                 0,    1]])
    
    T23 = sym.Matrix([[ sym.cos(theta_3),                0,  sym.sin(theta_3),    0],
                      [                0,                1,                 0, -0.1],
                      [-sym.sin(theta_3),                0,  sym.cos(theta_3),  0.8],
                      [                0,                0,                 0,    1]])
    
    T34 = sym.Matrix([[ sym.cos(theta_4),                0,  sym.sin(theta_4),    0],
                      [                0,                1,                 0,  0.1],
                      [-sym.sin(theta_4),                0,  sym.cos(theta_4),  0.8],
                      [                0,                0,                 0,    1]])
    
    T45 = sym.Matrix([[                1,                0,                 0, -0.1],
                      [                0, sym.cos(theta_5), -sym.sin(theta_5),    0],
                      [                0, sym.sin(theta_5),  sym.cos(theta_5),  0.3],
                      [                0,                0,                 0,    1]])
    
    T56 = sym.Matrix([[ sym.cos(theta_6),-sym.sin(theta_6),                 0,    0],
                      [ sym.sin(theta_6), sym.cos(theta_6),                 0,    0],
                      [                0,                0,                 1,  0.2],
                      [                0,                0,                 0,    1]])
    
    T6f = sym.Matrix([[                1,                0,                 0,    0],
                      [                0,                1,                 0,    0],
                      [                0,                0,                 1,  0.1],
                      [                0,                0,                 0,    1]])
    
    T0f = T01 * T12 * T23 * T34 * T45 * T56 * T6f
    homogeneous_point = sym.Matrix([0,0,0,1])
    # TODO: Multiply the homogeneous transformation matrix representing the manipulator's
    # forward kinematics by [0, 0, 0, 1]^T and get the jacobian of the result with respect to
    # our variables of interest 
    # TODO: get a callable method for the forward kinematics for the position of the end effector
    # and for evaluating its jacobian
    # NOTE: you should look up the documentation in sympy for:
    #   - jacobian
    #   - lambdify
    # TODO: this method should return both callable functions
    params = [theta_1, theta_2, theta_3, theta_4, theta_5, theta_6]
    Theta = sym.Matrix(params)
    tf_pos = sym.Matrix((T0f * homogeneous_point)[0:3])
    pos_jacobian = tf_pos.jacobian(Theta)



    fk_function = sym.lambdify(params,tf_pos, 'numpy')
    jac_function = sym.lambdify(params,pos_jacobian, 'numpy')

    return jac_function, fk_function


if __name__=='__main__':
    fk_solution()

