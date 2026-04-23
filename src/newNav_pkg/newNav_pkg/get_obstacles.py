import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tb_interfaces.msg import ObsList

from rclpy.qos import qos_profile_sensor_data

import math

# Decide your node class name
class lab2(Node):
    def __init__(self):

        # Change to have your node name
        super().__init__('get_obs')

        self.obstacle_detected = False
        self.obstacle_distance = 0.4
        self.detect_distance = 5.0

        self.PI = 3.14159265358979323846

        self.obs_pub = self.create_publisher(ObsList, '/obs', 10)

        self.stop_pub = self.create_publisher(Bool, '/stop', 10)

        self.scan_sub = self.create_subscription(LaserScan, '/robot4/scan', self.scan_callback, 10)


    def scan_callback(self,msg):
        self.obstacle_detected = False
        min_range = msg.range_min 
        max_range = msg.range_max
        front_scans = msg.ranges[200:340]

        self.too_close(front_scans, min_range, max_range)
        
        n = len(msg.ranges)
        self.get_logger().info(str(n)) 
        obs_x_list = []
        obs_y_list = []

        i = 0

        with open("obs_locs.csv", "w") as f:

            while i<n:
                scan_val = msg.ranges[i]

                if i < n/4:
                    ratio = ((n/4) - i) / (n/4)
                    theta = (-self.PI/2) * ratio
                elif i < (3*n/4):
                    ratio = (i - (n/4)) / (n/2)
                    theta = self.PI * ratio
                else:
                    ratio = (n - i) / (n/4)
                    theta = (-self.PI/2) * ratio - (self.PI/2)
                
                if (scan_val > min_range and scan_val < self.detect_distance):
                    obs_x = scan_val*math.sin(theta)
                    obs_y = scan_val*math.cos(theta)
                    obs_x_list.append(obs_x)
                    obs_y_list.append(obs_y)

                i += 1
                f.write(str(round(obs_x,3)) + "," + str(round(obs_y,3)) + "\n")

        # Publish obstacle locations
        obs = ObsList()
        obs.x_list = obs_x_list
        obs.y_list = obs_y_list
        self.obs_pub.publish(obs)

        # Publish if obstacle in close range
        stop = Bool()
        stop.data = self.obstacle_detected
        self.stop_pub.publish(stop)
                    

    def too_close(self, ranges, min, max):
        # Stop if too close to obstacle
        for scan_val in ranges:
            if (scan_val > min and scan_val < max):
                if scan_val < self.obstacle_distance:
                    self.get_logger().info("obstacle")
                    self.obstacle_detected = True

def main(args=None):
    rclpy.init(args=args)

    # Change to be your node class name
    node = lab2()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()