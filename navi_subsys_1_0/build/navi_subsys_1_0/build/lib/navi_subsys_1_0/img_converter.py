
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from sensor_msgs.msg import Image, CompressedImage

"""Converts a compressed image to an umcompressed image 22.03.2021 j.mahler

Subscribes: cam1_compressed [CompressedImage.msg]

"""
class Image_Converter(Node):

    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            CompressedImage,
            'cam1',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Image received')


def main(args=None):
    rclpy.init(args=args)

    image_subscriber = Image_Converter()

    rclpy.spin(image_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


"""

#!/usr/bin/env python

""" """Converts a compressed image to an umcompressed image 22.03.2021 j.mahler

Subscribes: cam1_compressed [CompressedImage.msg]
Publishes: cam1_uncompressed [Image.msg]

This node uses opencv to convert an compressed image from e.g. a unity sensor to a standard ROS image.msg for further processing.
This is done via the CvBridge package.
Also look at http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
"""
"""
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

class image_converter:
    
    def __init__(self):

        # Publish image in ros uncompressed Image.msg
        self.image_pub = rospy.Publisher("cam1/uncompressed",Image,queue_size=10)

        # Object to exchange images between ROS and OpenCV
        self.bridge = CvBridge()
        
        # Subscribe to the unity jpeg compressedImage.msg
        self.image_sub = rospy.Subscriber("cam1/compressed",CompressedImage,self.callback)
    
           # Write node desription to ros console
        rospy.loginfo("Node: img_converter.py.")
        rospy.loginfo("Description: Converts a compressed image to an umcompressed image.")
        rospy.loginfo("Subscribes: cam1/compressed [CompressedImage.msg]")
        rospy.loginfo("Publishes: cam1/uncompressed [Image.msg]\n")
        
    def callback(self,data):
     
        
        try:
        # Convert Unity compressed image to opencv matrix
            cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
    
        # Show image in opencv
        #cv2.imshow("opencv Image", cv_image)
        # Wait for closing till key pressed, otherwise closed with node shutdown
        #cv2.waitKey(3)


        try:
          # convert the image to image.msg and publish
          self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
          rospy.loginfo("Raw image published.")
        except CvBridgeError as e:
            rospy.logerr(e)
    
        
            

def main(args):
    # Set up ROS init
    rospy.init_node('img_converter', anonymous=True)
    
    # Instance of the image converter
    ic = image_converter()
    
    # Repeat node
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.logwarn("Node is shutted down.")
  
    # after closing ROS close all opencv windows
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
"""