#!/usr/bin/env python3
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker


class FollowTheGapZandvoort(Node):
    def __init__(self):
        super().__init__('ftg_zandvoort_kristopher')

        # --- Parámetros de Alta Velocidad (Time Attack Zandvoort) ---
        self.fov_deg = 88.0              # FOV más enfocado hacia adelante para trazar recto
        self.max_lidar_range = 13.0      # Mayor horizonte para detectar rectas largas
        self.bubble_radius = 0.34        # Burbuja ajustada al chasis real para no frenar de más
        self.disparity_threshold = 0.30  # Sensibilidad de esquinas
        self.car_half_width = 0.24       # Margen lateral ceñido al vértice (apex)

        # Perfil agresivo de velocidades (m/s)
        self.v_max = 10.2                # Velocidad punta en recta principal
        self.v_min = 2.65                 # Velocidad mínima en horquillas cerradas
        self.kp_base = 0.72              # Ganancia proporcional en curva
        self.kd_steer = 0.16             # Amortiguación derivativa anti-oscilación
        self.prev_steer = 0.0
        self.current_speed = 0.0

        # --- Estado del Contador de Vueltas y Cronómetro ---
        self.start_pos = None
        self.lap_count = 0
        self.target_laps = 10
        self.lap_start_time = None
        self.lap_times = []
        self.best_lap = None
        self.left_start_zone = False
        self.zone_radius = 2.0           # Radio ligeramente mayor por la alta velocidad
        self.min_lap_time = 5.5          # Permite contar vueltas rápidas menores a 10 s
        self.last_print_time = 0.0

        # Suscriptores y Publicadores
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/ego_racecar/odom', self.odom_callback, 10
        )
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, '/drive', 10
        )
        self.hud_pub = self.create_publisher(
            Marker, '/ftg_hud_marker', 10
        )
        self.target_pub = self.create_publisher(
            Marker, '/ftg_target_marker', 10
        )

        self.get_logger().info(
            'Modo Time Attack Zandvoort iniciado - Kristopher Pacheco'
        )

    # ==========================================================
    # 1. CRONÓMETRO, CONTADOR DE VUELTAS Y HUD 3D EN RVIZ
    # ==========================================================
    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        now = time.time()

        if self.start_pos is None:
            self.start_pos = (x, y)
            self.lap_start_time = now
            return

        dist_to_start = math.hypot(x - self.start_pos[0], y - self.start_pos[1])

        if dist_to_start > (self.zone_radius * 2.2):
            self.left_start_zone = True

        elapsed_current = now - self.lap_start_time
        if (
            self.left_start_zone
            and dist_to_start <= self.zone_radius
            and elapsed_current >= self.min_lap_time
        ):
            self.lap_count += 1
            self.lap_times.append(elapsed_current)
            if self.best_lap is None or elapsed_current < self.best_lap:
                self.best_lap = elapsed_current

            self.lap_start_time = now
            self.left_start_zone = False
            self.print_dashboard(elapsed_current, lap_completed=True)

        elif now - self.last_print_time >= 0.15:
            self.last_print_time = now
            self.print_dashboard(elapsed_current, lap_completed=False)

        self.publish_hud_marker(elapsed_current)

    def publish_hud_marker(self, current_lap_time: float):
        best_str = f"{self.best_lap:.2f}s" if self.best_lap else "--.--s"
        last_str = f"{self.lap_times[-1]:.2f}s" if self.lap_times else "--.--s"

        marker = Marker()
        marker.header.frame_id = 'ego_racecar/base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'hud_kristopher'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.9
        marker.scale.z = 0.32

        if self.lap_count >= self.target_laps:
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.1, 1.0, 0.2, 1.0
        else:
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.0, 0.95, 1.0, 1.0

        marker.text = (
            f"Kristopher Pacheco - Zandvoort\n"
            f"Vuelta: {self.lap_count}/{self.target_laps} | Tiempo: {current_lap_time:.2f}s\n"
            f"Ultima: {last_str} | Mejor: {best_str} | Vel: {self.current_speed:.1f} m/s"
        )
        self.hud_pub.publish(marker)

    def publish_target_marker(self, target_dist: float, target_angle: float):
        marker = Marker()
        marker.header.frame_id = 'ego_racecar/base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'target_gap'
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = float(target_dist * math.cos(target_angle))
        marker.pose.position.y = float(target_dist * math.sin(target_angle))
        marker.pose.position.z = 0.15
        marker.scale.x = 0.35
        marker.scale.y = 0.35
        marker.scale.z = 0.35

        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.1, 1.0, 0.1, 0.9
        self.target_pub.publish(marker)

    def print_dashboard(self, current_lap_time: float, lap_completed: bool):
        best_str = f"{self.best_lap:.3f} s" if self.best_lap else "--.--- s"
        last_str = f"{self.lap_times[-1]:.3f} s" if self.lap_times else "--.--- s"

        if lap_completed:
            print("\n" + "=" * 62)
            print(f" [META] VUELTA {self.lap_count} COMPLETADA EN: {current_lap_time:.3f} s")
            print("=" * 62)

        print(
            f"\r[ZANDVOORT | K. Pacheco] "
            f"Vuelta: {self.lap_count}/{self.target_laps} | "
            f"Cronometro: {current_lap_time:6.2f} s | "
            f"Ultima: {last_str} | "
            f"Mejor: {best_str} | Vel: {self.current_speed:4.1f} m/s",
            end="",
            flush=True,
        )

    # ==========================================================
    # 2. ALGORITMO FOLLOW THE GAP DE ALTA VELOCIDAD
    # ==========================================================
    def preprocess_lidar(self, ranges: np.ndarray, angle_inc: float) -> np.ndarray:
        proc = np.array(ranges, dtype=np.float32)
        proc = np.nan_to_num(proc, nan=0.0, posinf=self.max_lidar_range, neginf=0.0)

        # Ventana de suavizado corta (3) para menor latencia a alta velocidad
        kernel = np.ones(3, dtype=np.float32) / 3.0
        proc = np.convolve(proc, kernel, mode='same')
        proc = np.clip(proc, 0.0, self.max_lidar_range)

        diffs = np.abs(np.diff(proc))
        disparities = np.where(diffs > self.disparity_threshold)[0]
        for idx in disparities:
            closer_dist = min(proc[idx], proc[idx + 1])
            if closer_dist <= 0.05:
                continue
            angle_span = math.atan2(self.car_half_width, closer_dist)
            num_samples = int(math.ceil(angle_span / angle_inc))
            if proc[idx] < proc[idx + 1]:
                end_i = min(len(proc), idx + 1 + num_samples)
                proc[idx + 1 : end_i] = np.minimum(proc[idx + 1 : end_i], closer_dist)
            else:
                start_i = max(0, idx - num_samples + 1)
                proc[start_i : idx + 1] = np.minimum(proc[start_i : idx + 1], closer_dist)

        return proc

    def apply_safety_bubble(self, proc_ranges: np.ndarray, angle_inc: float) -> np.ndarray:
        valid_idx = np.where(proc_ranges > 0.05)[0]
        if len(valid_idx) == 0:
            return proc_ranges

        min_idx = valid_idx[np.argmin(proc_ranges[valid_idx])]
        min_dist = proc_ranges[min_idx]

        # Solo aplicar burbuja completa si el obstáculo está a menos de 2.8 m
        if min_dist > 2.8:
            return proc_ranges

        angle_bubble = math.atan2(self.bubble_radius, max(min_dist, 0.1))
        bubble_samples = int(math.ceil(angle_bubble / angle_inc))

        start_i = max(0, min_idx - bubble_samples)
        end_i = min(len(proc_ranges), min_idx + bubble_samples + 1)
        proc_ranges[start_i:end_i] = 0.0
        return proc_ranges

    def find_max_gap(self, free_space_ranges: np.ndarray):
        threshold = max(1.6, 0.38 * np.max(free_space_ranges))
        mask = free_space_ranges > threshold

        padded = np.concatenate(([False], mask, [False]))
        starts = np.where(padded[1:] & ~padded[:-1])[0]
        ends = np.where(~padded[1:] & padded[:-1])[0]

        if len(starts) == 0:
            mask_nonzero = free_space_ranges > 0.1
            padded_nz = np.concatenate(([False], mask_nonzero, [False]))
            starts = np.where(padded_nz[1:] & ~padded_nz[:-1])[0]
            ends = np.where(~padded_nz[1:] & padded_nz[:-1])[0]
            if len(starts) == 0:
                return 0, len(free_space_ranges) - 1

        lengths = ends - starts
        best_gap = np.argmax(lengths)
        return starts[best_gap], ends[best_gap] - 1

    def find_best_point(self, start_i: int, end_i: int, ranges: np.ndarray) -> int:
        gap_slice = ranges[start_i : end_i + 1]
        max_dist = np.max(gap_slice)

        deepest_candidates = np.where(gap_slice >= 0.94 * max_dist)[0] + start_i
        deepest_idx = int(np.mean(deepest_candidates))
        center_idx = (start_i + end_i) // 2

        # 78% hacia el fondo de la recta/salida de curva para trazar línea de carrera rápida
        return int(0.68 * deepest_idx + 0.32 * center_idx)

    def scan_callback(self, data: LaserScan):
        angles = data.angle_min + np.arange(len(data.ranges)) * data.angle_increment
        fov_rad = math.radians(self.fov_deg)
        fov_mask = (angles >= -fov_rad) & (angles <= fov_rad)
        fov_indices = np.where(fov_mask)[0]

        fov_ranges = np.array(data.ranges)[fov_indices]
        fov_angles = angles[fov_indices]

        proc_ranges = self.preprocess_lidar(fov_ranges, data.angle_increment)
        proc_ranges = self.apply_safety_bubble(proc_ranges, data.angle_increment)

        start_i, end_i = self.find_max_gap(proc_ranges)
        best_i = self.find_best_point(start_i, end_i, proc_ranges)

        target_angle = fov_angles[best_i]
        target_dist = float(proc_ranges[best_i])

        self.publish_target_marker(target_dist, target_angle)

        # Ganancia proporcional reducida en rectas para evitar oscilación a 10+ m/s
        center_idx = len(proc_ranges) // 2
        front_window = proc_ranges[max(0, center_idx - 10) : min(len(proc_ranges), center_idx + 10)]
        front_dist = float(np.mean(front_window)) if len(front_window) > 0 else 1.0

        kp_dynamic = self.kp_base * (0.72 if front_dist > 6.5 else 1.0)
        raw_steer = (
            kp_dynamic * target_angle
            + self.kd_steer * (target_angle - self.prev_steer)
        )
        steering_angle = float(np.clip(raw_steer, -0.4189, 0.4189))
        self.prev_steer = target_angle

        # Perfil de velocidad exponencial basado en ángulo de giro y distancia al objetivo
        abs_steer = abs(steering_angle)
        if abs_steer < 0.055 and front_dist > 6.5:
            speed = self.v_max
        else:
            steer_factor = math.exp(-3.2 * abs_steer)
            lookahead_factor = min(1.0, max(front_dist, target_dist * 0.85) / 7.5)
            speed = self.v_min + (self.v_max - self.v_min) * steer_factor * lookahead_factor

            # Frenado preventivo si el muro frontal está muy cerca entrando rápido a horquilla
            if front_dist < 3.6:
                speed = min(speed, self.v_min + 1.1 * (front_dist / 3.6))

        self.current_speed = float(np.clip(speed, self.v_min, self.v_max))

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'ego_racecar/base_link'
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = self.current_speed
        self.drive_pub.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FollowTheGapZandvoort()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nDeteniendo el vehículo y cerrando el nodo...")
    finally:
        stop_msg = AckermannDriveStamped()
        stop_msg.drive.speed = 0.0
        stop_msg.drive.steering_angle = 0.0
        node.drive_pub.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
