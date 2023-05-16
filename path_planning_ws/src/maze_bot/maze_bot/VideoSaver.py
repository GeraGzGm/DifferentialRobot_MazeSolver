import rclpy

from rclpy.node import Node

from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import cv2
import numpy as np
import os

class VideoSaver(Node):
    def __init__(self):
        super().__init__("video_saver_node")
        self.videofeed_subscriber = self.create_subscription(Image,'/upper_camera/image_raw',self.Process_Data,10)

        vid_path = os.path.join(os.getcwd(), "output.avi")
        
        self.out = cv2.VideoWriter(vid_path, cv2.VideoWriter_fourcc('M','J','P','G'),30, (1200,700))

        #Bridge between ROS and CV2 to conver ros image to opencv data
        self.bridge = CvBridge()
        

        self.sat_view = np.zeros((100,100))

    def Process_Data(self,data):
        """Convert frame to cv2 

        bridge.imgmsg_to_cv2(data,'bgr8'): Perform conversion from ros to opencv
        self.out.write: Write frame into output

        Args:
            data (_type_): _description_
        """

        frame = self.bridge.imgmsg_to_cv2(data,'bgr8')
        self.out.write(frame)
        
        cv2.imshow("sat_view", frame)
        cv2.waitKey(1)



def main(args =None):
    rclpy.init()
    node_obj =VideoSaver()
    rclpy.spin(node_obj)
    rclpy.shutdown()


if __name__ == '__main__':
    main()