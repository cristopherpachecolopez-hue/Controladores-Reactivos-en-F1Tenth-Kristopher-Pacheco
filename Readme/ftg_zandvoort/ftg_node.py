#!/usr/bin/env python3
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Point
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker


class FollowTheGapZandvoort(Node):
    def __init__(self):
        super().__init__('ftg_zandvoort_kristopher')

        # --- Parámetros configurables por consola (Ego y Robot Oponente) ---
        self.declare_parameter('is_opponent', False)
        self.declare_parameter('v_max', 8.2)
        self.declare_parameter('v_min', 2.1)
        self.declare_parameter('center_bias', 0.26)
        self.declare_parameter('lane_bias', 0.0)   # >0 abre hueco pegándose suave a la izq. en rectas

        self.is_opponent = self.get_parameter('is_opponent').get_parameter_value().bool_value
        self.v_max = self.get_parameter('v_max').get_parameter_value().double_value
        self.v_min = self.get_parameter('v_min').get_parameter_value().double_value
        self.center_bias = self.get_parameter('center_bias').get_parameter_value().double_value
        self.lane_bias = self.get_parameter('lane_bias').get_parameter_value().double_value

        # --- Márgenes de seguridad originales (sin cambios para mantener 0 colisiones) ---
        self.fov_deg = 74.0
        self.max_lidar_range = 12.0
        self.disparity_threshold = 0.24
        self.car_half_width = 0.21
        self.bubble_radius = 0.26
        self.corridor_half_width = 0.26

        self.kp_base = 0.78
        self.kd_steer = 0.15
        self.prev_target_angle = 0.0
        self.prev_steer_cmd = 0.0
        self.current_speed = 0.0

        # --- Estado del Contador de Vueltas, Cronómetro y Distancia ---
        self.start_pos = None
        self.prev_pos = None
        self.last_traj_pos = None
        self.lap_count = 0
        self.target_laps = 10
        self.lap_start_time = None
        self.lap_times = []
        self.lap_distances = []
        self.current_lap_dist = 0.0
        self.total_dist = 0.0
        self.best_lap = None
        self.left_start_zone = False
        self.zone_radius = 2.0
        self.min_lap_time = 6.5
        self.last_print_time = 0.0

        # --- Trayectoria y HUD sincronizados (inicia en Negro de alto contraste) ---
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'
        self.traj_points = []
        self.lap_colors = [
            (0.05, 0.05, 0.05),  # Vuelta 0: Negro
            (0.85, 0.05, 0.10),  # Vuelta 1: Rojo Carmesí
            (0.05, 0.20, 0.85),  # Vuelta 2: Azul Rey
            (0.55, 0.00, 0.75),  # Vuelta 3: Púrpura Oscuro
            (0.00, 0.55, 0.15),  # Vuelta 4: Verde Bosque
        ]

        if self.is_opponent:
            scan_topic = '/opp_scan'
            odom_topic = '/opp_racecar/odom'
            drive_topic = '/opp_drive'
            self.frame_id = 'opp_racecar/base_link'
        else:
            scan_topic = '/scan'
            odom_topic = '/ego_racecar/odom'
            drive_topic = '/drive'
            self.frame_id = 'ego_racecar/base_link'

        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

        if not self.is_opponent:
            self.hud_pub = self.create_publisher(Marker, '/ftg_hud_marker', 10)
            self.target_pub = self.create_publisher(Marker, '/ftg_target_marker', 10)
            self.traj_marker_pub = self.create_publisher(Marker, '/ftg_trajectory_marker', 10)
            self.path_pub = self.create_publisher(Path, '/ftg_path', 10)

        rol = "OPONENTE (Obstaculo Dinamico)" if self.is_opponent else "EGO (Kristopher Pacheco)"
        self.get_logger().info(f'Nodo FTG iniciado como {rol} | v_max={self.v_max} m/s')

    def get_current_lap_color(self):
        return self.lap_colors[self.lap_count % len(self.lap_colors)]

    def odom_callback(self, msg: Odometry):
        if self.is_opponent:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        now = time.time()

        if self.start_pos is None:
            self.start_pos = (x, y)
            self.prev_pos = (x, y)
            self.last_traj_pos = (x, y)
            self.lap_start_time = now
            self.reset_lap_trajectory(x, y, msg)
            return

        step_dist = math.hypot(x - self.prev_pos[0], y - self.prev_pos[1])
        if 0.002 < step_dist < 2.0:
            self.current_lap_dist += step_dist
            self.total_dist += step_dist
            self.prev_pos = (x, y)

        if math.hypot(x - self.last_traj_pos[0], y - self.last_traj_pos[1]) >= 0.20:
            self.last_traj_pos = (x, y)
            self.append_trajectory_point(x, y, msg)

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
            self.lap_distances.append(self.current_lap_dist)
            if self.best_lap is None or elapsed_current < self.best_lap:
                self.best_lap = elapsed_current

            completed_dist = self.current_lap_dist
            self.current_lap_dist = 0.0
            self.lap_start_time = now
            self.left_start_zone = False

            self.reset_lap_trajectory(x, y, msg)
            self.print_dashboard(elapsed_current, completed_dist, lap_completed=True)

        elif now - self.last_print_time >= 0.15:
            self.last_print_time = now
            self.print_dashboard(elapsed_current, self.current_lap_dist, lap_completed=False)

        self.publish_hud_marker(elapsed_current)

    def reset_lap_trajectory(self, x: float, y: float, odom_msg: Odometry):
        self.path_msg.poses.clear()
        self.traj_points.clear()

        start_pt = Point()
        start_pt.x = float(self.start_pos[0])
        start_pt.y = float(self.start_pos[1])
        start_pt.z = 0.05
        self.traj_points.append(start_pt)

        self.last_traj_pos = (x, y)
        self.append_trajectory_point(x, y, odom_msg)

    def append_trajectory_point(self, x: float, y: float, odom_msg: Odometry):
        pose = PoseStamped()
        pose.header = odom_msg.header
        pose.header.frame_id = 'map'
        pose.pose = odom_msg.pose.pose
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_msg.poses.append(pose)
        self.path_pub.publish(self.path_msg)

        pt = Point()
        pt.x = float(x)
        pt.y = float(y)
        pt.z = 0.05
        self.traj_points.append(pt)

        r, g, b = self.get_current_lap_color()

        traj_marker = Marker()
        traj_marker.header.frame_id = 'map'
        traj_marker.header.stamp = self.get_clock().now().to_msg()
        traj_marker.ns = 'racing_line'
        traj_marker.id = 2
        traj_marker.type = Marker.LINE_STRIP
        traj_marker.action = Marker.ADD
        traj_marker.scale.x = 0.13
        traj_marker.color.r = float(r)
        traj_marker.color.g = float(g)
        traj_marker.color.b = float(b)
        traj_marker.color.a = 1.0
        traj_marker.points = self.traj_points
        self.traj_marker_pub.publish(traj_marker)

    def publish_hud_marker(self, current_lap_time: float):
        best_str = f"{self.best_lap:.2f}s" if self.best_lap else "--.--s"
        last_str = f"{self.lap_times[-1]:.2f}s" if self.lap_times else "--.--s"
        last_dist_str = f"{self.lap_distances[-1]:.1f}m" if self.lap_distances else f"{self.current_lap_dist:.1f}m"

        r, g, b = self.get_current_lap_color()

        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'hud_kristopher'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.95
        marker.scale.z = 0.34

        marker.color.r = float(r)
        marker.color.g = float(g)
        marker.color.b = float(b)
        marker.color.a = 1.0

        marker.text = (
            f"Kristopher Pacheco - Zandvoort (Parte 2)\n"
            f"Lap: {self.lap_count}/{self.target_laps} | Time: {current_lap_time:.2f}s | Dist: {self.current_lap_dist:.1f}m\n"
            f"Last: {last_str} ({last_dist_str}) | Best: {best_str} | Speed: {self.current_speed:.1f}m/s"
        )
        self.hud_pub.publish(marker)

    def publish_target_marker(self, target_dist: float, target_angle: float):
        if self.is_opponent:
            return
        marker = Marker()
        marker.header.frame_id = self.frame_id
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

        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.1, 0.85, 0.1, 0.95
        self.target_pub.publish(marker)

    def print_dashboard(self, current_lap_time: float, lap_dist: float, lap_completed: bool):
        best_str = f"{self.best_lap:.3f} s" if self.best_lap else "--.--- s"
        last_str = f"{self.lap_times[-1]:.3f} s" if self.lap_times else "--.--- s"

        if lap_completed:
            avg_dist = float(np.mean(self.lap_distances))
            print("\n" + "=" * 78)
            print(
                f" [META] VUELTA {self.lap_count} | "
                f"Tiempo: {current_lap_time:.3f} s | "
                f"Distancia Vuelta: {lap_dist:.2f} m | "
                f"Prom. Dist: {avg_dist:.2f} m"
            )
            print("=" * 78)

        print(
            f"\r[ZANDVOORT | K. Pacheco] "
            f"Vuelta: {self.lap_count}/{self.target_laps} | "
            f"Tiempo: {current_lap_time:6.2f}s | "
            f"Dist: {self.current_lap_dist:6.1f}m | "
            f"Ultima: {last_str} | "
            f"Mejor: {best_str} | Vel: {self.current_speed:4.1f}m/s",
            end="",
            flush=True,
        )

    def apply_disparity_extender(self, ranges: np.ndarray, angle_inc: float) -> np.ndarray:
        proc = np.array(ranges, dtype=np.float32)
        proc = np.nan_to_num(proc, nan=0.0, posinf=self.max_lidar_range, neginf=0.0)

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

        if min_dist > 1.9:
            return proc_ranges

        angle_bubble = math.atan2(self.bubble_radius, max(min_dist, 0.1))
        bubble_samples = int(math.ceil(angle_bubble / angle_inc))

        start_i = max(0, min_idx - bubble_samples)
        end_i = min(len(proc_ranges), min_idx + bubble_samples + 1)
        proc_ranges[start_i:end_i] = 0.0
        return proc_ranges

    def find_max_gap(self, free_space_ranges: np.ndarray):
        max_r = float(np.max(free_space_ranges))
        threshold = max(1.30, 0.45 * max_r)
        mask = free_space_ranges > threshold

        padded = np.concatenate(([False], mask, [False]))
        starts = np.where(padded[1:] & ~padded[:-1])[0]
        ends = np.where(~padded[1:] & padded[:-1])[0]

        if len(starts) == 0:
            mask_nonzero = free_space_ranges > 0.15
            padded_nz = np.concatenate(([False], mask_nonzero, [False]))
            starts = np.where(padded_nz[1:] & ~padded_nz[:-1])[0]
            ends = np.where(~padded_nz[1:] & padded_nz[:-1])[0]
            if len(starts) == 0:
                return 0, len(free_space_ranges) - 1

        best_gap_idx = 0
        best_score = -1.0
        for g in range(len(starts)):
            s, e = starts[g], ends[g] - 1
            width = e - s + 1
            if width < 5:
                continue
            gap_depth = float(np.max(free_space_ranges[s : e + 1]))
            score = gap_depth * math.sqrt(width)
            if score > best_score:
                best_score = score
                best_gap_idx = g

        return starts[best_gap_idx], ends[best_gap_idx] - 1

    def find_best_point(self, start_i: int, end_i: int, ranges: np.ndarray, c_bias: float) -> int:
        gap_slice = ranges[start_i : end_i + 1]
        max_dist = np.max(gap_slice)

        deepest_candidates = np.where(gap_slice >= 0.93 * max_dist)[0] + start_i
        deepest_idx = int(np.mean(deepest_candidates))
        center_idx = (start_i + end_i) // 2

        d_weight = 1.0 - c_bias
        return int(d_weight * deepest_idx + c_bias * center_idx)

    def scan_callback(self, data: LaserScan):
        angles = data.angle_min + np.arange(len(data.ranges)) * data.angle_increment
        fov_rad = math.radians(self.fov_deg)
        fov_mask = (angles >= -fov_rad) & (angles <= fov_rad)
        fov_indices = np.where(fov_mask)[0]

        fov_ranges = np.array(data.ranges)[fov_indices]
        fov_angles = angles[fov_indices]
        raw_clean = np.nan_to_num(fov_ranges, nan=5.0, posinf=5.0, neginf=5.0)

        disp_ranges = self.apply_disparity_extender(fov_ranges, data.angle_increment)
        proc_ranges = self.apply_safety_bubble(disp_ranges.copy(), data.angle_increment)

        # Medir distancia libre en el corredor frontal antes de decidir el punto objetivo
        x_pts = disp_ranges * np.cos(fov_angles)
        y_pts = disp_ranges * np.sin(fov_angles)
        corridor_mask = (x_pts > 0.05) & (np.abs(y_pts) <= self.corridor_half_width)
        corridor_dist = float(np.min(disp_ranges[corridor_mask])) if np.any(corridor_mask) else self.max_lidar_range

        start_i, end_i = self.find_max_gap(proc_ranges)
        gap_max_dist = float(np.max(proc_ranges[start_i : end_i + 1]))

        # Si hay rival delante en una recta con carril lateral abierto, apuntar al centro del pasillo libre
        approaching_in_straight = (
            (not self.is_opponent)
            and gap_max_dist > 5.5
            and corridor_dist < 4.8
            and gap_max_dist > (corridor_dist + 2.0)
        )
        c_bias = 0.52 if approaching_in_straight else self.center_bias

        best_i = self.find_best_point(start_i, end_i, proc_ranges, c_bias)
        target_angle = float(fov_angles[best_i])
        target_dist = float(proc_ranges[best_i])

        # Oponente en rectas: se pega suavemente hacia la izquierda manteniendo >= 0.72 m a la pared
        if self.lane_bias > 0.0 and target_dist > 6.0 and abs(target_angle) < 0.16:
            left_wall_mask = (fov_angles >= math.radians(35.0)) & (fov_angles <= math.radians(74.0)) & (raw_clean > 0.05)
            if np.any(left_wall_mask):
                left_clearance = float(np.min(raw_clean[left_wall_mask] * np.sin(fov_angles[left_wall_mask])))
                if left_clearance > 0.72:
                    target_angle += min(self.lane_bias, (left_clearance - 0.72) * 0.18)

        self.publish_target_marker(target_dist, target_angle)

        # Confirmar modo rebase activo en recta (ángulo suave hacia el carril libre paralelo)
        overtaking_straight = (
            approaching_in_straight
            and target_dist > 5.5
            and abs(target_angle) < 0.32
        )

        # Dirección PD: respuesta ágil al iniciar el cambio de carril, estable en recta libre
        if overtaking_straight:
            kp_dynamic = self.kp_base * 1.15
        else:
            kp_dynamic = self.kp_base * (0.75 if target_dist > 6.5 else 1.06)

        raw_steer = (
            kp_dynamic * target_angle
            + self.kd_steer * (target_angle - self.prev_target_angle)
        )
        raw_steer = float(np.clip(raw_steer, -0.4189, 0.4189))
        self.prev_target_angle = target_angle

        steer_delta = np.clip(raw_steer - self.prev_steer_cmd, -0.095, 0.095)
        steering_angle = float(self.prev_steer_cmd + steer_delta)

        # Guarda lateral anti-corte original (sin cambios)
        left_side = raw_clean[(fov_angles >= math.radians(32.0)) & (fov_angles <= math.radians(72.0))]
        right_side = raw_clean[(fov_angles <= math.radians(-32.0)) & (fov_angles >= math.radians(-72.0))]

        if len(left_side) > 0 and np.min(left_side) < 0.48 and steering_angle > 0.0:
            steering_angle = 0.0
        if len(right_side) > 0 and np.min(right_side) < 0.48 and steering_angle < 0.0:
            steering_angle = 0.0

        self.prev_steer_cmd = steering_angle

        abs_steer = abs(steering_angle)
        if overtaking_straight and corridor_dist > 0.80:
            # Mantiene impulso (7.0 - 8.2 m/s) durante el cambio de carril para pasar al rival
            steer_factor = math.exp(-1.1 * abs_steer)
            dist_factor = min(1.0, max(0.0, (target_dist - 1.1) / 5.5))
            speed = self.v_min + (self.v_max - self.v_min) * steer_factor * dist_factor
        else:
            # Comportamiento exacto original para curvas y obstáculos estáticos
            if target_dist > 5.2 and abs_steer < 0.22 and corridor_dist > 1.15:
                effective_dist = target_dist
            else:
                effective_dist = min(target_dist, corridor_dist + 0.5)

            if abs_steer < 0.055 and effective_dist > 6.8:
                speed = self.v_max
            else:
                steer_factor = math.exp(-3.4 * abs_steer)
                dist_factor = min(1.0, max(0.0, (effective_dist - 1.1) / 5.7))
                speed = self.v_min + (self.v_max - self.v_min) * steer_factor * dist_factor

        if corridor_dist < 0.80:
            speed = self.v_min

        self.current_speed = float(np.clip(speed, self.v_min, self.v_max))

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = self.frame_id
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = self.current_speed
        self.drive_pub.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FollowTheGapZandvoort()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nDeteniendo vehículo...")
    finally:
        try:
            stop_msg = AckermannDriveStamped()
            stop_msg.drive.speed = 0.0
            stop_msg.drive.steering_angle = 0.0
            node.drive_pub.publish(stop_msg)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
