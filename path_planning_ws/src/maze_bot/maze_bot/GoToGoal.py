import rclpy
from rclpy.node import Node

#http://wiki.ros.org/geometry_msgs
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry


from math import sqrt, pow, atan2, pi
import numpy as np
import sys



class GoToGoal(Node):


    def __init__(self):
        super().__init__('goal_movement_node')
        
        #Create Subscriber with Type Message Twist
        self.pose_subs = self.create_subscription(Odometry, '/odom',self.pose_callback, 10)

        #Create Publisher for the Wheel Speed
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.go_to_goal_fnc)

        #Pose of the robot
        self.robot_pose = Point()
        #Pose of the goal
        self.goal_pose = Point()

        self.vel_msg = Twist()

    def pose_callback(self, data):
        self.robot_pose.x = data.pose.pose.position.x
        self.robot_pose.y = data.pose.pose.position.y
        Quaternion = data.pose.pose.orientation
 
        (roll, pitch, yaw) = self.euler_from_quaternion(Quaternion)
        self.robot_pose.z = yaw

    def go_to_goal_fnc(self):
        """
            Here the calculations of errors are made
        """
        
        self.goal_pose.x = float(sys.argv[1])
        self.goal_pose.y = float(sys.argv[2])


        e_distance = sqrt( pow(self.goal_pose.x - self.robot_pose.x,2) + pow(self.goal_pose.y - self.robot_pose.y,2))
        e_angle = atan2(self.goal_pose.y - self.robot_pose.y, self.goal_pose.x - self.robot_pose.x) + (-pi)

        real_e_angle = e_angle - self.robot_pose.z 

        if abs(real_e_angle) > 0.1:
            self.vel_msg.angular.z = real_e_angle
            self.vel_msg.linear.x = 0.0
        else:
            self.vel_msg.angular.z = 0.0
            self.vel_msg.linear.x = e_distance

        self.vel_pub.publish(self.vel_msg)

        msg = "e_d: {:.2f}, e_angle: {:.2f}".format(e_distance, e_angle)
        self.get_logger().info(msg)

    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args)

    gotogoal = GoToGoal()

    rclpy.spin(gotogoal)
    gotogoal.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


