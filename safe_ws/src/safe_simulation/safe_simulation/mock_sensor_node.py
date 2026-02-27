
import rclpy
from rclpy.node import Node
import math
import random

from geometry_msgs.msg import Pose 
from safe_interfaces.msg import SafeSignal
from visualization_msgs.msg import Marker

class MockSensorNode(Node):
    def __init__(self):
        # no hardcode the name here, we will assign it in the launch file
        super().__init__('mock_sensor_base')
        
        # sensor parameters
        self.declare_parameter('sensor_id', 'SAFE_EDV_000')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.0)
        self.declare_parameter('tx_power', -20.0)
        self.declare_parameter('num_drones', 10) # drones to lesten for
        
        # read parameters from start command
        self.sensor_id = self.get_parameter('sensor_id').value
        self.x = self.get_parameter('x').value
        self.y = self.get_parameter('y').value
        self.z = self.get_parameter('z').value
        self.tx_power = self.get_parameter('tx_power').value
        self.num_drones = self.get_parameter('num_drones').value
        
        # pysics parameters
        self.env_factor = 6
        self.detection_radius = 20.0  
        
        self.drone_poses = {}
        self.publishers_ = {}

        # create dynamic publishers and subscribers for the fleet
        for i in range(1, self.num_drones + 1):
            topic_signal = f'/drone{i}/radio_signal'
            self.publishers_[i] = self.create_publisher(SafeSignal, topic_signal, 10)
            
            topic_pose = f'/drone{i}/ground_truth/pose'
            self.create_subscription(Pose, topic_pose, lambda msg, d_id=i: self.pose_callback(msg, d_id), 10)

        # timers
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        self.timer_marker = self.create_timer(1.0, self.publish_sensor_marker)
        
        self.broadcast_interval = 0.1 
        self.timer_beacon = self.create_timer(self.broadcast_interval, self.broadcast_signals)
        
        self.get_logger().info(f"📡 Sensor [{self.sensor_id}] Active at ({self.x}, {self.y}, {self.z}).")

    def pose_callback(self, msg, drone_id):
        self.drone_poses[drone_id] = msg

    def broadcast_signals(self):
        for drone_id, pose in self.drone_poses.items():
            # calculate distance to this specific sensor
            dx = pose.position.x - self.x
            dy = pose.position.y - self.y
            dz = pose.position.z - self.z
            distance = math.sqrt(dx**2 + dy**2 + dz**2)

            if distance <= self.detection_radius:
                signal_msg = SafeSignal()
                signal_msg.device_id = self.sensor_id
                signal_msg.presence = 1
                signal_msg.is_war_mode = True 
                
                noise = random.uniform(-3.5, 3.5)
                rssi = self.tx_power - (10 * self.env_factor * math.log10(max(distance, 0.1))) + noise
                signal_msg.rssi = float(rssi)
                
                self.publishers_[drone_id].publish(signal_msg)

    # rviz marker
    def publish_sensor_marker(self):
        m = Marker()
        m.header.frame_id = "safe_world" 
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "sensors"
        m.id = hash(self.sensor_id) % 10000 
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.x = self.x
        m.pose.position.y = self.y
        m.pose.position.z = self.z
        m.scale.x = 0.5
        m.scale.y = 0.5
        m.scale.z = 0.1
        m.color.r = 0.0
        m.color.g = 0.5
        m.color.b = 1.0
        m.color.a = 0.8
        self.marker_pub.publish(m)

def main(args=None):
    rclpy.init(args=args)
    node = MockSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()