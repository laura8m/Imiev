#!/usr/bin/env python3
"""
Diagnóstico de simetría de giro del iMiEV.

Publica cmd_vel con giro a la DERECHA durante 5s, luego a la IZQUIERDA durante 5s.
Mide la velocidad angular real vía /odom Y /joint_states (steering joints).

Uso:  ros2 run imiev test_turning
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import threading
import time


class TurnTester(Node):
    def __init__(self):
        super().__init__('turn_tester')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribir a múltiples fuentes de odometría
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.odom_local_sub = self.create_subscription(Odometry, '/odometry/local', self.odom_local_cb, 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)

        self.lock = threading.Lock()
        self.yaw_rate_odom = 0.0
        self.yaw_rate_odom_local = 0.0
        self.steering_left = 0.0
        self.steering_right = 0.0
        self.odom_received = False
        self.odom_local_received = False
        self.joints_received = False

        # Parámetros del test
        self.linear_vel = 1.0
        self.angular_vel = 0.25
        self.test_duration = 5.0

        self.get_logger().info('=' * 60)
        self.get_logger().info('    DIAGNÓSTICO DE GIRO - iMiEV')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Linear vel: {self.linear_vel} m/s')
        self.get_logger().info(f'Angular vel: ±{self.angular_vel} rad/s')
        self.get_logger().info(f'Duración por sentido: {self.test_duration}s')
        self.get_logger().info('=' * 60)

    def odom_cb(self, msg: Odometry):
        with self.lock:
            self.yaw_rate_odom = msg.twist.twist.angular.z
            self.odom_received = True

    def odom_local_cb(self, msg: Odometry):
        with self.lock:
            self.yaw_rate_odom_local = msg.twist.twist.angular.z
            self.odom_local_received = True

    def joint_cb(self, msg: JointState):
        with self.lock:
            self.joints_received = True
            for i, name in enumerate(msg.name):
                if 'front_left' in name and 'steering' in name:
                    self.steering_left = msg.position[i] if i < len(msg.position) else 0.0
                elif 'front_right' in name and 'steering' in name:
                    self.steering_right = msg.position[i] if i < len(msg.position) else 0.0

    def get_data(self):
        with self.lock:
            return {
                'yaw_odom': self.yaw_rate_odom,
                'yaw_odom_local': self.yaw_rate_odom_local,
                'steer_left': self.steering_left,
                'steer_right': self.steering_right,
            }

    def run_turn(self, angular_z: float) -> list:
        samples = []
        twist = Twist()
        twist.linear.x = self.linear_vel
        twist.angular.z = angular_z

        start = time.time()
        while (time.time() - start) < self.test_duration:
            self.cmd_pub.publish(twist)
            data = self.get_data()
            samples.append(data)
            # Print live
            self.get_logger().info(
                f'  yaw_odom={data["yaw_odom"]:+.4f}  '
                f'yaw_local={data["yaw_odom_local"]:+.4f}  '
                f'steer_L={data["steer_left"]:+.4f}  '
                f'steer_R={data["steer_right"]:+.4f}'
            )
            time.sleep(0.5)

        return samples

    def send_stop(self):
        twist = Twist()
        for _ in range(20):
            self.cmd_pub.publish(twist)
            time.sleep(0.05)

    def run_test(self):
        self.get_logger().info('Esperando 3s para conectar subscripciones...')
        time.sleep(3.0)

        # Verificar qué subscripciones están recibiendo datos
        with self.lock:
            self.get_logger().info(f'  /odom recibido:           {"✅ SÍ" if self.odom_received else "❌ NO"}')
            self.get_logger().info(f'  /odometry/local recibido: {"✅ SÍ" if self.odom_local_received else "❌ NO"}')
            self.get_logger().info(f'  /joint_states recibido:   {"✅ SÍ" if self.joints_received else "❌ NO"}')

        # --- Test 1: DERECHA ---
        self.get_logger().info('')
        self.get_logger().info(f'>>> FASE 1: Girando a la DERECHA (angular.z = -{self.angular_vel:.2f})...')
        samples_right = self.run_turn(-self.angular_vel)

        self.send_stop()
        self.get_logger().info('Parando 3s...')
        time.sleep(3.0)

        # --- Test 2: IZQUIERDA ---
        self.get_logger().info('')
        self.get_logger().info(f'>>> FASE 2: Girando a la IZQUIERDA (angular.z = +{self.angular_vel:.2f})...')
        samples_left = self.run_turn(self.angular_vel)

        self.send_stop()

        # --- Resultados ---
        self.print_results(samples_right, samples_left)

    def print_results(self, samples_right, samples_left):
        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info('    RESULTADOS')
        self.get_logger().info('=' * 60)

        # Usar últimas 60% de muestras (descartar transitorio)
        def avg_stable(samples, key):
            n = len(samples)
            stable = samples[max(0, n // 3):]
            vals = [s[key] for s in stable]
            return sum(vals) / len(vals) if vals else 0.0

        # Yaw rates
        yr_right_odom = avg_stable(samples_right, 'yaw_odom')
        yr_left_odom = avg_stable(samples_left, 'yaw_odom')
        yr_right_local = avg_stable(samples_right, 'yaw_odom_local')
        yr_left_local = avg_stable(samples_left, 'yaw_odom_local')

        # Steering angles
        st_right_l = avg_stable(samples_right, 'steer_left')
        st_right_r = avg_stable(samples_right, 'steer_right')
        st_left_l = avg_stable(samples_left, 'steer_left')
        st_left_r = avg_stable(samples_left, 'steer_right')

        self.get_logger().info('--- Yaw rates (rad/s) ---')
        self.get_logger().info(f'  /odom:          DERECHA={yr_right_odom:+.4f}   IZQUIERDA={yr_left_odom:+.4f}')
        self.get_logger().info(f'  /odometry/local: DERECHA={yr_right_local:+.4f}   IZQUIERDA={yr_left_local:+.4f}')
        self.get_logger().info('')
        self.get_logger().info('--- Steering angles (rad) ---')
        self.get_logger().info(f'  Giro DERECHA:   steer_L={st_right_l:+.4f}  steer_R={st_right_r:+.4f}')
        self.get_logger().info(f'  Giro IZQUIERDA: steer_L={st_left_l:+.4f}   steer_R={st_left_r:+.4f}')

        # Comparación de steering (más fiable que odom)
        avg_steer_right = (abs(st_right_l) + abs(st_right_r)) / 2.0
        avg_steer_left = (abs(st_left_l) + abs(st_left_r)) / 2.0

        if avg_steer_left > 0.001:
            ratio = avg_steer_right / avg_steer_left
        elif avg_steer_right > 0.001:
            ratio = float('inf')
        else:
            ratio = 1.0

        self.get_logger().info('')
        self.get_logger().info(f'RATIO steering |derecha/izquierda| = {ratio:.3f}  (ideal = 1.000)')

        if avg_steer_right < 0.001 and avg_steer_left < 0.001:
            self.get_logger().warn('⚠️  Las ruedas NO GIRAN en ningún sentido.')
            self.get_logger().warn('   → Verifica que Gazebo está corriendo y el bridge está activo.')
            self.get_logger().warn('   → Comprueba: ros2 topic list | grep cmd_vel')
        elif 0.85 <= ratio <= 1.15:
            self.get_logger().info('✅ SIMÉTRICO: Las ruedas responden igual a ambos lados.')
            self.get_logger().info('   → El problema está en el PLANNER / COSTMAP, no en las ruedas.')
        else:
            self.get_logger().warn(f'⚠️  ASIMÉTRICO (ratio={ratio:.3f})')
            self.get_logger().warn('   → Revisa fricción, inercia, o joints en robot.xacro')

        self.get_logger().info('=' * 60)
        self.get_logger().info('Test completado.')


def main(args=None):
    rclpy.init(args=args)
    node = TurnTester()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.run_test()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()
