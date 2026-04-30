# Interface imports
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tb_interfaces.action import RobotGoal
from tb_interfaces.msg import ObsList

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
import os

class NavPFNode(Node):
    def __init__(self):
        super().__init__('nav_pf')
        
        self.PI = 3.14159265358979323846

        # Current robot position
        self.x = 0
        self.y = 0
        self.ang = 0

        self.x_offset = 0 
        self.y_offset = 0
        self.ang_offset = 0
        self.got_offset = False

        # Size of the robot
        self.robot_radius = 0.3

        # Threshold of close enough
        self.pos_threshold = 0.02
        self.ang_threshold = 0.06
        self.goal_threshold = 0.1

        # When we care about obs for potential fields
        self.obs_threshold = 0.8

        # How long to try getting to goal before giving up
        self.max_iteration = 1e20

        # Store our obstacle locations relative to robot (robot center is 0,0)
        self.obs_space_rob_frame = []

        # Store our obstacle locations relative to world frame (world 0,0)
        self.obs_space_world_frame = []

        # Store distances from obstacles to robot
        self.obs_dist = []

        # Subscribe to the odometry (robot location?)
        self.pos_subscriber = self.create_subscription(Odometry, '/robot4/odom', self.callback_pos, 10)

        # Subscribe to the obstacle locations
        
        self.obs_subscriber = self.create_subscription(ObsList, '/obs', self.callback_obs, 10)

        # Velocity publisher
        self.velocity_pub = self.create_publisher(Twist, '/robot4/cmd_vel_unfiltered', 10)

        # Obstcles publisher
        self.obstacles_pub = self.create_publisher(String, '/closest_obs', 10)

        # Bounds publisher
        self.loc_pub = self.create_publisher(String, 'loc', 10)

        # ACtion server
        self.go_to_goal = ActionServer(self, RobotGoal,"nav_goal",goal_callback=self.goal_callback,execute_callback=self.execute_callback)

    # Callback for pos sub
    def callback_pos(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        quaternion = msg.pose.pose.orientation
         # Angle converted from quaternion to euler
        (_,_,ang) = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])   
        
        if self.got_offset is False:
            # Point where node starts is 0, 0, rotation 0
            self.x_offset = -x
            self.y_offset = -y
            self.ang_offset = -ang
            self.got_offset = True

        ang += self.ang_offset
        
        if ang < -self.PI:
            self.ang = ang + (2*self.PI)
        else:
            self.ang = ang

        # Transform
        self.x = x*math.cos(self.ang_offset) - y*math.sin(self.ang_offset) + self.x_offset
        self.y = x*math.sin(self.ang_offset) + y*math.cos(self.ang_offset) + self.y_offset

        test = String()
        test.data = "Current position x: " + str(round(self.x,2)) + " y: " + str(round(self.y,2)) + " ang: " + str(round(self.ang,4)) + "\n"
        self.loc_pub.publish(test)

    # Callback for obstacle locations
    def callback_obs(self,msg):
        self.obs_space_rob_frame = []
        self.obs_space_world_frame = []
        self.obs_dist = list(msg.d_list)
        lim  = len(msg.x_list)
        i = 0
        while i < lim:
            x_rob = msg.x_list[i]
            y_rob = msg.y_list[i]
            self.obs_space_rob_frame.append([x_rob,y_rob])

            x_world = x_rob*math.cos(self.ang) - y_rob*math.sin(self.ang) + self.x
            y_world = x_rob*math.sin(self.ang) + y_rob*math.cos(self.ang) + self.y
            self.obs_space_world_frame.append([x_world,y_world])
            
            i+=1

    
    # Goal callback
    def goal_callback(self, goal_request):
        goal = [goal_request.goal_x,goal_request.goal_y]
        min_distance = 10000
        closest_obs = [0,0]

        for obstacle in self.obs_space_world_frame:
            distance = math.dist(goal,obstacle)
            if distance < min_distance:
                min_distance = distance
                closest_obs = obstacle
        
        test = String()
        test.data = "closest obs: " + str(closest_obs[0]) + " y: " + str(closest_obs[1]) + "\n"
        self.obstacles_pub.publish(test)

        if min_distance < self.robot_radius:
            self.get_logger().info("Rejected, too close to obstacle at x: " + str(closest_obs[0]) + " y: " + str(closest_obs[1]))
            return GoalResponse.REJECT
        
        self.get_logger().info("Accepted goal!")
        return GoalResponse.ACCEPT

    def get_att_f(self, d_goal, goal_x, goal_y):
        k_att = 1.0
        att_mag = 0.5 * k_att * d_goal
        att_ang = math.atan2(goal_y - self.y, goal_x - self.x)
        f_att_x = att_mag * math.cos(att_ang)
        f_att_y = att_mag * math.sin(att_ang)

        return f_att_x, f_att_y

    def get_rep_f(self):
        k_rep = 0.8
        numObs = len(self.obs_space_world_frame)
        f_rep_x = 0
        f_rep_y = 0
        for n in range(numObs):
            d_obs = self.obs_dist[n]
            [x_obs, y_obs] = self.obs_space_world_frame[n]
           

            if d_obs < self.obs_threshold:
                rep_mag = 0.5 * k_rep * ((1/d_obs) - (1/self.obs_threshold))**2
                rep_ang = math.atan2(self.y - y_obs, self.x - x_obs)

                f_rep_x += rep_mag * math.cos(rep_ang)
                f_rep_y += rep_mag * math.sin(rep_ang)

        return f_rep_x, f_rep_y

    def pid_to_point(self, goal_x, goal_y):
        # For PID
        err_pos_prev = 0
        err_ang_prev = 0
        err_pos_sum = 0
        err_ang_sum = 0
        vel_ang_prev = 0.1

        kpl = 0.4
        kdl = 0.2
        kil = 0

        kpa = 0.1
        kda = 0.02
        kia = 0.0

        # Calc new errs
        err_pos = math.dist([goal_x,goal_y],[self.x,self.y])

        i = 0

        while abs(err_pos) > self.pos_threshold:
            self.get_logger().info("goal: " + str(goal_x) + ", " + str(goal_y) + "| err: " + str(err_pos))
            
            if i > self.max_iteration:
                break

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
                # If at the -pi to pi boundary just keep prior vel until past that point
                if abs(err_ang) > self.PI:
                    vel_ang = vel_ang_prev
                else:
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
            vel_ang_prev = vel_ang

            # Calc new errs
            err_pos = math.dist([goal_x,goal_y],[self.x,self.y])
            i += 1

    
    def execute_callback(self, goal_handle):
        # Make goal relative to robot current pose
        goal_x = goal_handle.request.goal_x 
        goal_y = goal_handle.request.goal_y 
        goal_theta = goal_handle.request.goal_theta

        result = RobotGoal.Result()
        feedback = RobotGoal.Feedback()
       
        # Get initial distance
        err_pos = math.dist([goal_x,goal_y],[self.x,self.y])
        
        iteration = 0

        # timestep for pf
        timestep = 1.0

        # While not close enough
        while err_pos > self.pos_threshold:

            if iteration > self.max_iteration:
                # Set result success to true
                result.success = False
                self.get_logger().info("Timed out")
                # Set status to succeed
                goal_handle.succeed()
                # Return result
                return result
            
            # Potential fields
            fax, fay = self.get_att_f(err_pos, goal_x, goal_y)
            frx, fry = self.get_rep_f()

            fx = fax + frx
            fy = fay + fry

            mag = math.sqrt(fx**2 + fy**2)

            fx /= mag
            fy /= mag

            dx = timestep * fx
            dy = timestep * fy

            gx = self.x + dx
            gy = self.y + dy

            self.pid_to_point(gx,gy)

            # Publish feedback
            feedback.current_x = float(round(self.x,2))
            feedback.current_y = float(round(self.y,2))
            feedback.current_theta = float(round(self.ang,2))
            feedback.distance_from_goal = float(round(err_pos,2))
            goal_handle.publish_feedback(feedback)
            
            err_pos = math.dist([goal_x,goal_y],[self.x,self.y])
            iteration += 1

        # Calc dif between current angle and goal angle
        err_ang = goal_theta - self.ang

        # While not close enough
        while abs(err_ang) > self.ang_threshold:
            vel = Twist()
            vel.linear.x = 0.0
            # Rotate
            vel.angular.z = kpa*err_ang + kda*(err_ang - err_ang_prev) + kia*err_ang_sum
            # Publish 
            self.velocity_pub.publish(vel)

            # Calc dif between current angle and goal angle
            err_ang = goal_theta - self.ang

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
    node = NavPFNode()

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