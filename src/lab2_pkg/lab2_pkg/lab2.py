import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

# Decide your node class name
class lab2(Node):
    def __init__(self):

        # Change to have your node name
        super().__init__('lab2')

        self.obstacle_detected = False
        self.obstacle_distance = 0.5

        self.vel_sub = self.create_subscription(Twist, '/robot4/cmd_vel_unfiltered', self.callback_vel, 10)
        self.vel_pub = self.create_publisher(Twist, 'robot4/cmd_vel_unstamped', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/robot4/scan', self.scan_callback, 10)

        #self.led_publish = self.create_publisher(LightringLeds, '/robotN/cmd_lightring', qos_profile_sensor_data)

    def callback_vel(self,msg):
        self.fwd_vel = msg.linear.x
        self.ang_vel = msg.angular.z

        if self.fwd_vel > 0.4:
            self.fwd_vel = 0.4
            self.get_logger().info("Too fast!")

        if self.obstacle_detected:
            self.fwd_vel = 0.0

        new_vel = Twist()
        new_vel.linear.x = self.fwd_vel
        new_vel.angular.z = self.ang_vel

        self.vel_pub.publish(new_vel)

    def scan_callback(self,msg):
        self.obstacle_detected = False
        min_range = msg.range_min 
        max_range = msg.range_max
        ranges = msg.ranges[200:340]
        for range in ranges:
            if (range > min_range and range < max_range):
                if range < self.obstacle_distance:
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