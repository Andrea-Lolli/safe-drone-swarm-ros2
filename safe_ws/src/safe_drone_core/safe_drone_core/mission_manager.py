import rclpy
from rclpy.node import Node
import csv
import math
from std_msgs.msg import Bool
import os

from geometry_msgs.msg import Twist, Pose
from safe_interfaces.msg import SafeSignal
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist, Pose, Point

class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        
        # Drone id dynamic parameter
        self.declare_parameter('drone_id', 1)
        self.drone_id = self.get_parameter('drone_id').value

        # mission parameters
        self.acceptance_radius = 0.2  
        self.hover_time_sec = 0.5     #stop time for each waypoint
        
        # drone state
        self.state = 'IDLE'
        self.hover_start_time = 0.0
        self.current_rssi_readings = {}

        # listen for start command
        self.start_sub = self.create_subscription(Bool, '/swarm/start', self.start_callback, 10)
        self.get_logger().info(f"drone {self.drone_id} ready. waiting for start command...")
        
        self.current_part = 1
        self.waypoints = []
        self.current_wp_index = 0

        self.load_mission_part(self.current_part)

        #ros communication
        # dynamic topic based on drone id
        vel_topic = f'/drone{self.drone_id}/cmd_vel'
        pose_topic = f'/drone{self.drone_id}/ground_truth/pose'
        
        self.vel_publisher = self.create_publisher(Twist, vel_topic, 10)
        self.pose_subscriber = self.create_subscription(Pose, pose_topic, self.pose_callback, 10)
        radio_topic = f'/drone{self.drone_id}/radio_signal'
        self.signal_subscriber = self.create_subscription(SafeSignal, radio_topic, self.signal_callback, 10)

        # data logging
        self.current_pose = None
        
        self.results_dir = os.path.expanduser('~/safe_ws/mission_results')
        if not os.path.exists(self.results_dir):
            os.makedirs(self.results_dir, exist_ok=True)
            
        self.log_filename = os.path.join(self.results_dir, f'safe_mission_log_drone{self.drone_id}.csv')
        self.log_file = open(self.log_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        self.csv_writer.writerow(['device_id','wp_x', 'wp_y', 'wp_z', 'avg_rssi'])
        
        # control loop timer
        self.timer = self.create_timer(0.1, self.control_loop)

        #marker for rviz
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        self.timer_marker = self.create_timer(5.0, self.publish_waypoints_markers)

    def publish_waypoints_markers(self):
        if not self.waypoints:
            return

        colors = {
            1: (1.0, 0.0, 0.0),
            2: (0.0, 1.0, 0.0),
            3: (0.0, 0.5, 1.0)
        }
        r, g, b = colors.get(self.drone_id, (1.0, 1.0, 0.0))

        for i, wp in enumerate(self.waypoints):
            m = Marker()
            m.header.frame_id = "safe_world"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = f"waypoints_drone_{self.drone_id}" 
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = wp[0]
            m.pose.position.y = wp[1]
            m.pose.position.z = wp[2]
            m.scale.x = 0.3
            m.scale.y = 0.3
            m.scale.z = 0.3
            m.color.r = r
            m.color.g = g
            m.color.b = b
            m.color.a = 0.6
            
            self.marker_pub.publish(m)
            
        if len(self.waypoints) > 1:
            line_marker = Marker()
            line_marker.header.frame_id = "safe_world"
            line_marker.header.stamp = self.get_clock().now().to_msg()
            line_marker.ns = f"trajectory_drone_{self.drone_id}"
            line_marker.id = 1000 
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.scale.x = 0.05 
            
            line_marker.color.r = r
            line_marker.color.g = g
            line_marker.color.b = b
            line_marker.color.a = 0.8
            
            for wp in self.waypoints:
                p = Point()
                p.x = wp[0]
                p.y = wp[1]
                p.z = wp[2]
                line_marker.points.append(p)
                
            self.marker_pub.publish(line_marker)

    def load_mission_part(self, part_number):
        # find mission path in input folder
        base_path = os.path.expanduser('~/safe_ws/mission_paths') 
        mission_file = os.path.join(base_path, f'mission_drone{self.drone_id}_part{part_number}.txt')
        
        self.waypoints = []
        self.current_wp_index = 0
        
        if os.path.exists(mission_file):
            with open(mission_file, 'r') as f:
                for line in f:
                    parts = line.strip().split(',')
                    if len(parts) == 3:
                        self.waypoints.append([float(parts[0]), float(parts[1]), float(parts[2])])
            self.get_logger().info(f"Part {part_number} loaded from {mission_file}, ({len(self.waypoints)} wp)")
            return True
        else:
            if part_number == 1:
                self.get_logger().error(f"File {mission_file} not found, drone {self.drone_id} won't take off")
            return False

    def start_callback(self, msg):
        if msg.data == True and self.state == 'IDLE':
            if len(self.waypoints) > 0:
                self.get_logger().info(f"drone {self.drone_id}: start command recieved, take off for part {self.current_part} of the mission")
                self.state = 'FLYING'

    def pose_callback(self, msg):
        self.current_pose = msg

    def signal_callback(self, msg):
        if self.state == 'HOVERING':
            if msg.device_id not in self.current_rssi_readings:
                self.current_rssi_readings[msg.device_id] = []
            self.current_rssi_readings[msg.device_id].append(msg.rssi)

    def control_loop(self):
        if not self.current_pose or len(self.waypoints) == 0:
            return

        if self.current_wp_index >= len(self.waypoints):
            self.stop_drone()
            
            # look for next part
            next_part = self.current_part + 1
            has_next_part = self.load_mission_part(next_part)
            
            if has_next_part:
                self.current_part = next_part
                self.state = 'IDLE' # wait for new start command
                self.get_logger().info(f"drone {self.drone_id}: landed, waiting for battery chanege. waiting for /swarm/start to start part {next_part} of the mission")
            else:
                if self.state != 'COMPLETED':
                    self.get_logger().info(f"drone {self.drone_id}: mission done!")
                    self.log_file.close() # close file
                    self.state = 'COMPLETED'
            return

        current_time = self.get_clock().now().nanoseconds / 1e9

        # state machine
        if self.state == 'FLYING':
            target = self.waypoints[self.current_wp_index]
            cx, cy, cz = self.current_pose.position.x, self.current_pose.position.y, self.current_pose.position.z
            
            dx = target[0] - cx
            dy = target[1] - cy
            dz = target[2] - cz
            distance = math.sqrt(dx**2 + dy**2 + dz**2)
            
            if distance < self.acceptance_radius:
                # ignore some waypoints
                is_structural_wp = (self.current_wp_index == 0 or self.current_wp_index >= len(self.waypoints) - 2)
                
                if is_structural_wp:
                    # skip scan
                    self.current_wp_index += 1
                else:
                    # stop and scan
                    self.stop_drone()
                    self.state = 'HOVERING'
                    self.hover_start_time = current_time
                    self.current_rssi_readings = {}
            else:
                # keep flying to next waypint
                max_speed = 3.0 # mvmnt speed
                cmd = Twist()
                cmd.linear.x = min(max(dx, -max_speed), max_speed)
                cmd.linear.y = min(max(dy, -max_speed), max_speed)
                cmd.linear.z = min(max(dz, -max_speed), max_speed)
                self.vel_publisher.publish(cmd)

        elif self.state == 'HOVERING':
            self.stop_drone()
            
            if (current_time - self.hover_start_time) >= self.hover_time_sec:
                target = self.waypoints[self.current_wp_index]
                
                if len(self.current_rssi_readings) > 0:
                    for device_id, readings in self.current_rssi_readings.items():
                        avg_rssi = sum(readings) / len(readings)
                        self.csv_writer.writerow([self.current_wp_index, device_id, target[0], target[1], target[2], avg_rssi])
                else:
                    # no signal found, still mark waypoint with none value
                    self.csv_writer.writerow([self.current_wp_index, 'NONE', target[0], target[1], target[2], -100.0])
                
                # force csv save
                self.log_file.flush() 
                
                self.current_wp_index += 1
                self.state = 'FLYING'       

    def stop_drone(self):
        cmd = Twist()
        self.vel_publisher.publish(cmd)

    def __del__(self):
        if hasattr(self, 'log_file') and self.log_file:
            self.log_file.close()

def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
