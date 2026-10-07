#!/usr/bin/env python3
import threading
import time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState


# ============================================================
# PARÁMETROS DH Y CINEMÁTICA DIRECTA (KUKA LBR iisy 11)
# ============================================================

def dh(theta, d, a, alpha):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,   sa,       ca,      d],
        [0,   0,        0,       1]
    ], dtype=float)


def get_fk_matrices(q):
    A01 = dh(-q[0],         0.300,    0.0,   -np.pi/2)
    A12 = dh(q[1],         -0.08925,  0.590,  0.0)
    A23 = dh(q[2]-np.pi/2,  0.08925,  0.0,    np.pi/2)
    A34 = dh(q[3]+np.pi,   -0.532,    0.0,    np.pi/2)
    A45 = dh(q[4]+np.pi,    0.0,      0.0,    np.pi/2)
    A56 = dh(q[5],         -0.0837,   0.0,    0.0)

    At = np.array([
        [-1, 0,  0, 0],
        [ 0, 1,  0, 0],
        [ 0, 0, -1, -0.0943],
        [ 0, 0,  0, 1]
    ], dtype=float)

    return [A01, A12, A23, A34, A45, A56, At]


def fk(q):
    m = get_fk_matrices(q)
    T = m[0]
    for mat in m[1:]:
        T = T @ mat
    return T


def jacobian_geometrico(q):
    m = get_fk_matrices(q)
    origins = []
    axes = []
    
    T = np.eye(4)
    for i in range(6):
        origins.append(T[:3, 3].copy())
        axes.append(T[:3, 2].copy())
        T = T @ m[i]

    T_total = T @ m[6]
    p_n = T_total[:3, 3]

    J = np.zeros((6, 6))
    for i in range(6):
        z_i = axes[i]
        p_i = origins[i]
        J[:3, i] = np.cross(z_i, p_n - p_i)
        J[3:, i] = z_i

    J[:, 0] = -J[:, 0]
    return J


# ============================================================
# CINEMÁTICA INVERSA Y LÍMITES
# ============================================================

Q_MIN = np.deg2rad([-185, -225, -150, -180, -120, -360])
Q_MAX = np.deg2rad([185, 45, 150, 180, 120, 360])
Q_HOME = np.array([0.0, -0.5, 0.8, 0.0, 0.5, 0.0], dtype=float)


def aplicar_limites(q):
    return np.clip(q, Q_MIN, Q_MAX)


def ik_position(p_des, q0):
    q = np.array(q0, dtype=float)
    gain = 0.6
    tol_pos = 5e-3  # Tolerancia de 5 mm para mayor flexibilidad
    max_iterations = 300
    max_step = 0.08

    for k in range(max_iterations):
        T_curr = fk(q)
        p_curr = T_curr[:3, 3]
        e_pos = p_des - p_curr

        if np.linalg.norm(e_pos) < tol_pos:
            return q, True, k, np.linalg.norm(e_pos)

        J = jacobian_geometrico(q)
        J_v = J[:3, :]

        lmbda = 0.02
        J_plus = J_v.T @ np.linalg.inv(J_v @ J_v.T + (lmbda ** 2) * np.eye(3))
        dq = J_plus @ e_pos

        norm_dq = np.linalg.norm(dq)
        if norm_dq > max_step:
            dq = dq * (max_step / norm_dq)

        q = q + gain * dq
        q = aplicar_limites(q)

    return q, False, max_iterations, np.linalg.norm(p_des - fk(q)[:3, 3])


# ============================================================
# NODO ROS 2 CON ANIMACIÓN SUAVE
# ============================================================

class IKNode(Node):

    def __init__(self):
        super().__init__("ik_node")
        self.q = Q_HOME.copy()
        self.have_solution = True
        self.is_moving = False

        self.publisher = self.create_publisher(JointState, "/joint_states", 10)

        self.subscription = self.create_subscription(
            Point, "/target", self.target_callback, 10
        )

        self.timer = self.create_timer(0.02, self.publish_solution)

        self.input_thread = threading.Thread(target=self.console_loop, daemon=True)
        self.input_thread.start()

        self.get_logger().info("Nodo IK activo con animación de movimiento suave.")

    def move_smoothly_to(self, q_target, duration=2.0):
        """Interpola el movimiento articular suavemente usando Smoothstep."""
        self.is_moving = True
        q_start = np.copy(self.q)
        steps = int(duration * 50)  # 50 pasos por segundo

        for i in range(1, steps + 1):
            alpha = i / steps
            s = alpha * alpha * (3 - 2 * alpha)  # Función de interpolación suave
            self.q = (1 - s) * q_start + s * q_target
            time.sleep(1.0 / 50.0)

        self.is_moving = False

    def solve_with_multiseed(self, p_des):
        q_sol, success, iters, err = ik_position(p_des, self.q)
        if success:
            return q_sol, success, iters, err

        q_sol, success, iters, err = ik_position(p_des, Q_HOME)
        if success:
            return q_sol, success, iters, err

        seeds = [
            np.array([0.5, -0.8, 1.0, 0.0, 0.5, 0.0]),
            np.array([-0.5, -0.8, 1.0, 0.0, 0.5, 0.0]),
            np.array([0.0, -1.2, 1.5, 0.0, 0.2, 0.0]),
        ]
        for seed in seeds:
            q_sol, success, iters, err = ik_position(p_des, seed)
            if success:
                return q_sol, success, iters, err

        return q_sol, False, iters, err

    def target_callback(self, msg):
        if self.is_moving:
            return

        p_des = np.array([msg.x, msg.y, msg.z], dtype=float)
        q_sol, success, iterations, error = self.solve_with_multiseed(p_des)

        if success:
            print(
                f"\n[ROS] Target -> x:{msg.x:.3f} y:{msg.y:.3f} z:{msg.z:.3f} | "
                f"Iteraciones: {iterations} | Error: {error*1000:.3f} mm"
            )
            threading.Thread(target=self.move_smoothly_to, args=(q_sol, 2.0), daemon=True).start()
        else:
            print(f"\n[ROS] Posición inalcanzable. Error residual: {error:.4f} m")

    def console_loop(self):
        time.sleep(1.0)
        while rclpy.ok():
            try:
                if self.is_moving:
                    time.sleep(0.1)
                    continue

                print("\n--- Ingrese Coordenadas ---")
                x = float(input("x: ").strip())
                y = float(input("y: ").strip())
                z = float(input("z: ").strip())

                p_des = np.array([x, y, z], dtype=float)
                q_sol, success, iterations, error = self.solve_with_multiseed(p_des)

                if success:
                    print(
                        f"Target -> x:{x:.3f} y:{y:.3f} z:{z:.3f} | "
                        f"Iteraciones: {iterations} | Error: {error*1000:.3f} mm"
                    )
                    self.move_smoothly_to(q_sol, duration=2.0)
                else:
                    print(f"Error: Posición inaccesible. Error residual: {error:.4f} m")

            except ValueError:
                print("Error: Ingrese números válidos.")
            except (KeyboardInterrupt, EOFError):
                break

    def publish_solution(self):
        if not self.have_solution:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [f"joint_{i}" for i in range(1, 7)]
        msg.position = self.q.tolist()
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = IKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()