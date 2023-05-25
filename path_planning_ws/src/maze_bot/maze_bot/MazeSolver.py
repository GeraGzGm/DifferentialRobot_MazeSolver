import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import cv2
import numpy as np

from Menus.Method_Selector import Method_Selector
from .BotLocalization import Bot_Localizer
from .BotMapping import Mapping
from .BotPathPlanning import Bot_PathPlanner

class MazeSolver(Node):
    def __init__(self):
        super().__init__("maze_solving_node")
        self.velocity_publisher = self.create_publisher(Twist,'/cmd_vel',10)
        self.videofeed_subscriber = self.create_subscription(Image,'/upper_camera/image_raw',self.get_video_feed_cb,10)

        self.timer = self.create_timer(0.2, self.maze_solving)
        self.bridge = CvBridge()

        self.vel_msg = Twist()

        self.localizer_BOT = Bot_Localizer()
        self.mapping = Mapping()
        self.path_planner = Bot_PathPlanner()
        self.Method_Selector = Method_Selector()

        self.frame = []
        self.Done = False


    def get_video_feed_cb(self,data):
        frame = self.bridge.imgmsg_to_cv2(data,'bgr8')
        
        self.frame = frame

        #cv2.imshow("CameraView", frame)
        #cv2.waitKey(1)

    def maze_solving(self):
        #self.Method_Selector.win.update()

        if self.Method_Selector.Method == "":
            self.Method_Selector.run()
        
        else:
            frame_display = self.frame.copy()
            self.localizer_BOT.Localize_bot(self.frame, frame_display)

            if not self.Done:
                self.mapping.Graphify(self.localizer_BOT.Occupancy_grid)
                start = self.mapping.Graph.start
                end = self.mapping.Graph.end
                maze = self.mapping.maze
                
                method = self.Method_Selector.Method

                self.path_planner.find_path_nd_display(self.mapping.Graph.graph, start, end , maze ,method)

                self.Done = True

        self.vel_msg.linear.x = 0.1
        self.vel_msg.angular.z = 0.1

        self.velocity_publisher.publish(self.vel_msg)


def main(args =None):
    rclpy.init()
    node_obj = MazeSolver()
    rclpy.spin(node_obj)
    rclpy.shutdown()


if __name__ == '__main__':
    main()