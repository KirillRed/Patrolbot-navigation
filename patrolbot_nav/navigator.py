#!/usr/bin/env python3
"""
PatrolNavigator Node
--------------------
A robust, reactive navigation node for ROS 2.
Features:
  - Lidar-based obstacle avoidance with 'Squeeze Mode' for narrow corridors.
  - Universal Lidar handling (supports both Front-0 and Rear-0 sensor types).
  - Multi-sensor fusion: Lidar (Primary), Rear Sonar (Safety), Bumper (Emergency).
  - Dynamic parameter configuration via Launch files.

Author: Kyrylo Redenskyi
License: MIT
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2 
import numpy as np
import importlib
import math

class PatrolNavigator(Node):
    def __init__(self):
        super().__init__('patrol_navigator')

        # --- 1. PARAMETERS ---
        self.declare_parameter('use_sim', False)
        self.declare_parameter('enable_sonar', True)
        self.declare_parameter('enable_bumper', True)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('sonar_topic', '/sonar_pointcloud2')
        self.declare_parameter('bumper_topic', '/bumper_state')
        self.declare_parameter('speed', 0.25)

        # Load Params
        self.use_sim = self.get_parameter('use_sim').value
        self.use_sonar = self.get_parameter('enable_sonar').value
        self.use_bumper = self.get_parameter('enable_bumper').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.sonar_topic = self.get_parameter('sonar_topic').value
        self.bumper_topic = self.get_parameter('bumper_topic').value
        self.speed = self.get_parameter('speed').value

        # --- 2. INTERNAL STATE ---
        self.lidar_ranges = []
        self.angle_min = 0.0
        self.angle_max = 0.0
        self.angle_increment = 0.0
        self.has_scan = False
        
        self.rear_blocked = False 
        self.bumper_hit = False
        
        # Logic States
        self.is_panicking = False
        self.squeeze_cooldown = 0  
        self.COOLDOWN_LIMIT = 10   
        self.avoid_direction = 0.0
        
        # Smoothing
        self.prev_ang_z = 0.0
        self.ANG_SMOOTH_ALPHA = 0.7  # 0.0 = No smoothing, 1.0 = Frozen

        # --- 3. PUBLISHERS ---
        if self.use_sim:
            self.pub = self.create_publisher(TwistStamped, self.cmd_vel_topic, 10)
        else:
            self.pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # --- 4. SUBSCRIBERS ---
        # Lidar
        self.create_subscription(LaserScan, self.scan_topic, self.lidar_callback, qos_profile_sensor_data)
        
        # Sonar (Optional)
        if self.use_sonar:
            self.create_subscription(PointCloud2, self.sonar_topic, self.sonar_callback, qos_profile_sensor_data)

        # Bumper (Optional & Dynamic)
        if self.use_bumper:
            try:
                # Dynamic import to avoid errors on non-Rosaria machines
                rosaria_msgs = importlib.import_module('rosaria.msg')
                BumperState = getattr(rosaria_msgs, 'BumperState')
                self.create_subscription(BumperState, self.bumper_topic, self.bumper_callback, 10)
                self.get_logger().info(f"Bumper: Connected to {self.bumper_topic}")
            except (ModuleNotFoundError, AttributeError):
                self.get_logger().warn("Bumper: 'rosaria' module not found. Disabling physical bumper logic.")
                self.use_bumper = False

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(f"PatrolNavigator Started. Sim Mode: {self.use_sim}")

    # --- CALLBACKS ---

    def lidar_callback(self, msg):
        self.has_scan = True
        self.angle_min = msg.angle_min
        self.angle_max = msg.angle_max
        self.angle_increment = msg.angle_increment
        
        ranges = np.array(msg.ranges)
        # Handle Inf/NaN
        ranges[ranges == float('inf')] = 5.0
        ranges[np.isnan(ranges)] = 5.0
        
        # Map 0.0 errors to MAX, but keep tiny valid readings (0.05)
        ranges[(ranges < 0.05) & (ranges > 0.0)] = 0.05 
        ranges[ranges == 0.0] = 5.0
        
        self.lidar_ranges = ranges

    def sonar_callback(self, msg):
        # Only processes if enabled
        self.rear_blocked = False
        for p in pc2.read_points(msg, field_names=("x", "y"), skip_nans=True):
            x, y = p[0], p[1]
            dist = np.sqrt(x**2 + y**2)
            # Rear check: x < -0.1 (Behind), dist < 0.6 (Close)
            if x < -0.1 and dist < 0.6: 
                self.rear_blocked = True
                break

    def bumper_callback(self, msg):
        # Msg type: rosaria/BumperState
        if any(msg.front_bumpers):
            self.bumper_hit = True
            self.get_logger().error("BUMPER HIT! EMERGENCY REVERSE.")
        else:
            self.bumper_hit = False

    # --- UTILS ---

    def normalize_angle(self, angle):
        """Ensures the requested angle fits within the sensor's min/max range."""
        if self.angle_min <= angle <= self.angle_max: return angle
        angle_plus_2pi = angle + (2 * np.pi)
        if self.angle_min <= angle_plus_2pi <= self.angle_max: return angle_plus_2pi
        angle_minus_2pi = angle - (2 * np.pi)
        if self.angle_min <= angle_minus_2pi <= self.angle_max: return angle_minus_2pi
        return max(self.angle_min, min(angle, self.angle_max))

    def get_sector_min(self, start_rad, end_rad):
        """
        Returns the minimum distance in a sector.
        Handles wrapping (e.g., Front sector spanning 350->10 degrees).
        """
        if not self.has_scan: return 5.0
        
        real_start = self.normalize_angle(start_rad)
        real_end   = self.normalize_angle(end_rad)
        
        start_idx = int(round((real_start - self.angle_min) / self.angle_increment))
        end_idx   = int(round((real_end   - self.angle_min) / self.angle_increment))

        start_idx = max(0, min(start_idx, len(self.lidar_ranges)))
        end_idx   = max(0, min(end_idx,   len(self.lidar_ranges)))

        if start_idx > end_idx:
            part1 = self.lidar_ranges[start_idx:]
            part2 = self.lidar_ranges[:end_idx]
            min1 = np.min(part1) if len(part1) > 0 else 5.0
            min2 = np.min(part2) if len(part2) > 0 else 5.0
            return min(min1, min2)
        else:
            if abs(end_idx - start_idx) < 2: return 5.0
            sector = self.lidar_ranges[start_idx:end_idx]
            if len(sector) == 0: return 5.0
            return np.min(sector)

    def publish_cmd(self, lin, ang):
        if self.use_sim:
            msg = TwistStamped()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.twist.linear.x = float(lin)
            msg.twist.angular.z = float(ang)
            self.pub.publish(msg)
        else:
            msg = Twist()
            msg.linear.x = float(lin)
            msg.angular.z = float(ang)
            self.pub.publish(msg)

    # --- MAIN CONTROL LOOP ---

    def control_loop(self):
        # Priority 0: PHYSICAL BUMPER
        if self.bumper_hit:
            self.publish_cmd(-0.15, 0.0) # Back up straight
            return

        if not self.has_scan: return

        # --- SENSE ---
        d_center      = self.get_sector_min(-0.17, 0.17)
        
        d_left_shoulder  = self.get_sector_min(0.35, 0.70) 
        d_right_shoulder = self.get_sector_min(-0.70, -0.35)
        
        d_front_left  = self.get_sector_min(0.17, 0.6)
        d_front_right = self.get_sector_min(-0.6, -0.17)
        
        d_side_left   = self.get_sector_min(0.6, 1.3)
        d_side_right  = self.get_sector_min(-1.3, -0.6)

        lin_x = 0.0
        ang_z = 0.0

        # --- LOGIC ---

        panic_start_dist = 0.50
        panic_end_dist   = 0.75
        shoulder_panic_limit = 0.35 # Shoulders hit walls sooner than center

        is_center_blocked = d_center < panic_start_dist
        is_shoulder_blocked = (d_left_shoulder < shoulder_panic_limit) or \
                              (d_right_shoulder < shoulder_panic_limit)

        # PRIORITY 1: EMERGENCY STOP / PANIC RECOVERY
        if self.is_panicking or is_center_blocked or is_shoulder_blocked:
            self.squeeze_cooldown = 0
            
            # CHECK EXIT CONDITION (Hysteresis)
            # We only stop panicking if center AND shoulders are clear
            if (d_center > panic_end_dist) and \
               (d_left_shoulder > shoulder_panic_limit) and \
               (d_right_shoulder > shoulder_panic_limit):
                self.is_panicking = False
            else:
                self.is_panicking = True
                
                # Determine Spin Direction
                # If we are just starting to panic, decide direction once based on clearance
                if self.avoid_direction == 0.0:
                    if self.use_sonar and self.rear_blocked:
                         self.avoid_direction = 0.8 # Forced spin if trapped
                    else:
                        # Spin AWAY from the closest shoulder/side
                        total_left = min(d_side_left, d_left_shoulder)
                        total_right = min(d_side_right, d_right_shoulder)
                        self.avoid_direction = -0.5 if total_left < total_right else 0.5

                lin_x = 0.0
                ang_z = self.avoid_direction
                self.prev_ang_z = 0.0

        # PRIORITY 2: SQUEEZE MODE
        elif d_front_left < 0.6 or d_front_right < 0.6:
            self.avoid_direction = 0.0 # Reset panic direction
            self.squeeze_cooldown = self.COOLDOWN_LIMIT
            
            lin_x = self.speed * 0.7 # Slow down
            
            # Center strictly between the two obstacles
            error = d_front_left - d_front_right
            target_ang = max(-0.2, min(0.2, error * 1.0))
            
            # Apply Smoothing
            ang_z = (self.ANG_SMOOTH_ALPHA * self.prev_ang_z + 
                     (1 - self.ANG_SMOOTH_ALPHA) * target_ang)

        # PRIORITY 3: TAIL CLEARING
        elif self.squeeze_cooldown > 0:
            self.squeeze_cooldown -= 1
            lin_x = self.speed * 0.7
            ang_z = 0.0 
            self.prev_ang_z = 0.0

        # PRIORITY 4: CRUISE
        else:
            self.avoid_direction = 0.0
            lin_x = self.speed
            
            val_l = min(d_side_left, 2.0)
            val_r = min(d_side_right, 2.0)
            
            # Loose wall following
            if val_l > 0.8 and val_r > 0.8:
                target_ang = 0.0
            else:
                target_ang = max(-0.3, min(0.3, (val_l - val_r) * 0.6))

            # Apply Smoothing
            ang_z = (self.ANG_SMOOTH_ALPHA * self.prev_ang_z + 
                     (1 - self.ANG_SMOOTH_ALPHA) * target_ang)

        self.prev_ang_z = ang_z
        self.publish_cmd(lin_x, ang_z)

def main(args=None):
    rclpy.init(args=args)
    node = PatrolNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()