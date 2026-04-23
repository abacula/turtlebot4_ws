# Interface imports
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import String
from tb_interfaces.action import RobotGoal

# ROS TF Transforms
from tf_transformations import euler_from_quaternion

# Action imports
from rclpy.action import ActionServer, GoalResponse
from rclpy.action.server import ServerGoalHandle

# Threading imports
from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import ExternalShutdownException
from rclpy.callback_groups import ReentrantCallbackGroup

# Base stuff
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import math

class MapPubNode(Node):
    def __init__(self):
        super().__init__('go_to_goal')
        
        self.PI = 3.14159265358979323846

        # Current robot position
        self.x = 0
        self.y = 0
        self.ang = 0

        # Size of the robot
        self.robot_radius = 0.3

        # Threshold of close enough
        self.pos_threshold = 0.1
        self.ang_threshold = 0.1

        # How long to try getting to goal before giving up
        self.max_iteration = 1e20

        # Store our obstacle locations
        self.obstacle_space = []

        # Map data
        self.resoltuion = 0.05 # Default 5cm
        self.width = 0
        self.height = 0
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.origin_ang = 0.0
        self.max_x = 100
        self.max_y = 100

        # Subscribe to the velocity unfiltered commands
        self.pos_subscriber = self.create_subscription(Odometry, '/robot4/odom', self.callback_pos, 10)
        self.pos_subscriber

        # Subscribe to the velocity unfiltered commands
        self.map_subscriber = self.create_subscription(OccupancyGrid, '/robot4/map', self.callback_map, 10)
        self.map_subscriber

        # Velocity publisher
        self.velocity_pub = self.create_publisher(Twist, '/robot4/cmd_vel_unfiltered', 10)

        # Obstcles publisher
        self.obstacles_pub = self.create_publisher(String, 'obs', 10)

        # Bounds publisher
        self.bounds_pub = self.create_publisher(String, 'bounds', 10)

        # ACtion server
        self.go_to_goal = ActionServer(self, RobotGoal,"robot_goal",goal_callback=self.goal_callback,execute_callback=self.execute_callback)

    # Callback for pos sub
    def callback_pos(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        quaternion = msg.pose.pose.orientation

        # Angle converted from quaternion to euler
        (_,_,self.ang) = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
       

    # index in map to real x y
    def index_to_real(self,col,row):
        real_x = round((col*self.resolution) + self.origin_x,2)
        real_y = round((row*self.resoltuion) + self.origin_y,2)
        return real_x,real_y

    # get occupancy grid and find free, unknown, obstacle space
    def callback_map(self,msg):
        self.resolution = round(msg.info.resolution,3)
        # The origin of the map [m, m, rad].  This is the real-world pose of the  cell (0,0) in the map
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.origin_ang = msg.info.origin.orientation.z
        
        self.width = msg.info.width
        self.height = msg.info.height

        self.max_x, self.max_y  = self.index_to_real(self.width, self.height)

        occupancy_grid = msg.data

        self.free_space = []
        self.unknown_space = []
        self.obstacle_space = []
        
        test2 = String()
        test2.data = " Obstacles: "
        self.obstacles_pub.publish(test2)

        row = 0
        while row < self.height:
            col = 0
            while col < self.width:
                real_x,real_y = self.index_to_real(col,row)
                point = occupancy_grid[col + (row*self.width)]
                if point == -1:
                    self.unknown_space.append([real_x,real_y]) 
                elif point > 25:
                    self.obstacle_space.append([real_x,real_y]) 
                    test2 = String()
                    test2.data = str(real_x) + "," + str(real_y) + "\n"
                    self.obstacles_pub.publish(test2)   
                else:
                    self.free_space.append([real_x,real_y])         
                col += 1
            row += 1

        test = String()
        test.data = "Bounds are x: " + str(round(self.origin_x,2)) + " to " + str(round(self.max_x,2)) + "\n"
        test.data += "Bounds are y: " + str(round(self.origin_y,2)) + " to " + str(round(self.max_y,2)) + "\n"
        test.data += "Current position x: " + str(round(self.x,2)) + " y: " + str(round(self.x,2)) + "\n"
        self.bounds_pub.publish(test)


    # Goal callback
    def goal_callback(self, goal_request):
        goal = [goal_request.goal_x,goal_request.goal_y]
        min_distance = 10000
        closest_obs = [0,0]

        for obstacle in self.obstacle_space:
            distance = math.dist(goal,obstacle)
            if distance < min_distance:
                min_distance = distance
                closest_obs = obstacle
        
        test = String()
        test.data = "closest obs: " + str(closest_obs[0]) + " y: " + str(closest_obs[1]) + "\n"
        self.obstacles_pub.publish(test)
           
        if (goal[0] > self.max_x or goal[1] > self.max_y):
            self.get_logger().info("Rejected because out of bounds")
            self.get_logger().info("Bounds are x: " + str(round(self.origin_x,2)) + " to " + str(round(self.max_x,2)))
            self.get_logger().info("Bounds are y: " + str(round(self.origin_y,2)) + " to " + str(round(self.max_y,2)))
            return GoalResponse.REJECT

        if min_distance < self.robot_radius:
            self.get_logger().info("Rejected, too close to obstacle at x: " + str(closest_obs[0]) + " y: " + str(closest_obs[1]))
            return GoalResponse.REJECT
        
        self.get_logger().info("Accepted goal!")
        return GoalResponse.ACCEPT
    
    def execute_callback(self, goal_handle):
        # Make goal relative to robot current pose
        goal_x = goal_handle.request.goal_x # + self.x
        goal_y = goal_handle.request.goal_y # + self.y
        goal_theta = goal_handle.request.goal_theta

        result = RobotGoal.Result()
        feedback = RobotGoal.Feedback()


        # Base speeds
        rotation_speed = 0.25
        forward_speed = 0.4

        # Get initial distance
        err_pos = math.dist([goal_x,goal_y],[self.x,self.y])

        # For PID
        err_pos_prev = 0
        err_ang_prev = 0
        err_pos_sum = 0
        err_ang_sum = 0

        kpl = 0.4
        kdl = 0.2
        kil = 0

        kpa = 0.1
        kda = 0.02
        kia = 0

        iteration = 0

        # While not close enough
        while abs(err_pos) > self.pos_threshold:

            if iteration > self.max_iteration:
                # Set result success to true
                result.success = False
                self.get_logger().info("Timed out")
                # Set status to succeed
                goal_handle.succeed()
                # Return result
                return result
                

            # New velocity msg
            vel = Twist()
            
            vel_lin = 0.0
            vel_ang = 0.0
            
            # Calc desired angle
            desired_angle = math.atan2(goal_y - self.y, goal_x - self.x)

            # Calc ang error
            err_ang =  desired_angle - self.ang

            err_ang_sum += err_ang     
            err_pos_sum += err_pos

            # If not close enough to desired angle
            if abs(err_ang) > self.ang_threshold:
                vel_ang = kpa*err_ang + kda*(err_ang - err_ang_prev) + kia*err_ang_sum
            else:
                vel_lin = kpl*err_pos + kdl*(err_pos - err_pos_prev) + kil*err_pos_sum

            vel.linear.x = vel_lin
            vel.angular.z = vel_ang

            
            # Publish velocity
            self.velocity_pub.publish(vel)

            # Replace prev err
            err_pos_prev = err_pos
            err_ang_prev = err_ang

            # Calc new errs
            err_pos = math.dist([goal_x,goal_y],[self.x,self.y])
            iteration += 1

            # Publish feedback
            feedback.current_x = float(round(self.x,2))
            feedback.current_y = float(round(self.y,2))
            feedback.current_theta = float(round(self.ang,2))
            feedback.distance_from_goal = float(round(err_pos,2))
            goal_handle.publish_feedback(feedback)

        # Calc dif between current angle and goal angle
        goal_angle_dif = abs(self.ang - goal_theta)
        # While not close enough
        while goal_angle_dif > self.ang_threshold:
            vel = Twist()
            vel.linear.x = 0.0
            # Rotate
            vel.angular.z = rotation_speed
            # Publish 
            self.velocity_pub.publish(vel)

            # Calc dif between current angle and goal angle
            goal_angle_dif = abs(self.ang - goal_theta)

            # Publish feedback
            feedback.current_x = float(round(self.x,2))
            feedback.current_y = float(round(self.y,2))
            feedback.current_theta = float(round(self.ang,2))
            feedback.distance_from_goal= float(round(err_pos,2))
            goal_handle.publish_feedback(feedback)

        # Stop moving
        vel = Twist()
        vel.linear.x = 0.0
        vel.angular.z = 0.0
        # Publish
        self.velocity_pub.publish(vel)

        # Set result success to true
        result.success = True
        self.get_logger().info("Arrived")
        # Set status to succeed
        goal_handle.succeed()
        # Return result
        return result


def main(args=None): 
    rclpy.init(args=None)
    node = MapPubNode()

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
   
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()