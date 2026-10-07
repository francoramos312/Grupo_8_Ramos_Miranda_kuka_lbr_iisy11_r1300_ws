#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as R

# PARÁMETROS DH Y CINEMÁTICA DIRECTA (KUKA LBR iisy 11)

def dh(theta, d, a, alpha):
    """
    Matriz de transformación DH:

    A = Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
    """

    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)

    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,   sa,       ca,       d],
        [0,   0,        0,        1]
    ], dtype=float)

# MATRICES DH DEL ROBOT

def get_fk_matrices(q):

    q1, q2, q3, q4, q5, q6 = q

    A01 = dh(
        -q1,
        0.300,
        0.0,
        -np.pi / 2
    )

    A12 = dh(
        q2,
        -0.08925,
        0.590,
        0.0
    )

    A23 = dh(
        q3 - np.pi / 2,
        0.08925,
        0.0,
        np.pi / 2
    )

    A34 = dh(
        q4 + np.pi,
        -0.532,
        0.0,
        np.pi / 2
    )

    A45 = dh(
        q5 + np.pi,
        0.0,
        0.0,
        np.pi / 2
    )

    A56 = dh(
        q6,
        -0.0837,
        0.0,
        0.0
    )

    At = np.array([
        [-1,  0,  0,      0],
        [ 0,  1,  0,      0],
        [ 0,  0, -1, -0.0943],
        [ 0,  0,  0,      1]
    ], dtype=float)

    return [A01, A12, A23, A34, A45, A56, At]

# CINEMÁTICA DIRECTA

def fk(q):

    matrices = get_fk_matrices(q)

    T = matrices[0]

    for matrix in matrices[1:]:
        T = T @ matrix

    return T

class JointSubscriber(Node):

    def __init__(self):

        super().__init__('joint_subscriber')

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.sub_callback,
            10
        )

        self.get_logger().info(
            'Nodo de Cinemática Directa activo.'
        )

    def sub_callback(self, msg):

        # DATOS DE LAS ARTICULACIONES

        joints = dict(zip(msg.name, msg.position))

        if not all(
            joint in joints
            for joint in [
                'joint_1',
                'joint_2',
                'joint_3',
                'joint_4',
                'joint_5',
                'joint_6'
            ]
        ):
            return

        j1 = joints['joint_1']
        j2 = joints['joint_2']
        j3 = joints['joint_3']
        j4 = joints['joint_4']
        j5 = joints['joint_5']
        j6 = joints['joint_6']

        # VECTOR DE ARTICULACIONES

        q = np.array([
            j1,
            j2,
            j3,
            j4,
            j5,
            j6
        ], dtype=float)

        T = fk(q)

        # POSICIÓN

        x = T[0, 3]
        y = T[1, 3]
        z = T[2, 3]

        # MATRIZ DE ROTACIÓN

        rotation_matrix = T[0:3, 0:3]

        # CONVERTIR ORIENTACIÓN A CUATERNIÓN

        rotation = R.from_matrix(
            rotation_matrix
        )

        quaternion = rotation.as_quat()

        print(
            f"x:{x:.3f} "
            f"y:{y:.3f} "
            f"z:{z:.3f} "
            f"orientation:{quaternion}"
        )

def main(args=None):

    rclpy.init(args=args)
    node = JointSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()