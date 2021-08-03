"""
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
"""
#!/usr/bin/env python
import cv2
import time
import rclpy
import math
import numpy as np
import matplotlib.pyplot as plt
from cv_bridge import CvBridge

from sensor_msgs.msg import CompressedImage
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

# ROS custom message
from gnc_interfaces.msg import ImageSensor

from datetime import datetime

class get_image(Node):
    """ Extract lines in an image

            :param (subscribes): topic cam1/pixel [ImageSensor.msg] 
            :return (publishes): 
        This ROS2 node extracts lines with the Hough algorithm.
        20.05.2021 j.mahler"""
    
    def __init__(self):
        """ Initialize image data subscription

                :param - 
                :return -
            Set up node with topic and console output."""
        
        super().__init__('image_subscriber')
                
        # Info to show in terminal
        self.get_logger().info("Node: get_image")
        self.get_logger().info("Description: Subscribes to the pixel data of cam1.")
        self.get_logger().info("Subscribes:  cam1\pixel [ImageSensor.msg]")
        self.get_logger().info("Publishes: <None>\n")
        
        self.onstart = 0
        
        self.subscription = self.create_subscription(
            ImageSensor,
            'cam1/pixel',
            self.listener_callback,
            10)
        # prevent unused variable warning
        self.subscription  

    def read_image(self,msg):
        """ Read an image from file

                :param -
                :return image[numpy.array (M,N,3)]
            No further details"""
        
        
        try:
            imgsensor = cv2.imread(msg.datapath)
        except:
            self.errorcustom = True
            self.get_logger().error("Error in method read_image")
        
        if imgsensor is None:
            self.errorcustom = True
            self.get_logger().warning("Read image is of type None.")
        
        return imgsensor
        
    def lines_extraction(self, imgsensor):
        """ Extract lines from an image

                :param  imgsensor [numpy.array (M,N,3)]
                :return edges3 [numpy.array (M,N,1)]
                :return lines [numpy.ndarray (ρ,θ)]
            Sets up a greyscale image, use Canny to detect edges and hough to extract lines"""
        
        if (self.errorcustom == True):
            return None, None
        
        # Canny edge detection coefficents.
        # first threshold for the hysteresis procedure. (170)
        threshold1 = 170
        # second threshold for the hysteresis procedure. (260as)
        threshold2 = 260

        # Hough line detection coefficents.
        # Distance resolution of the accumulator in pixels. (1.1)
        rhohough = 1.1
        # Angle resolution of the accumulator in radians. (math.pi/180)
        thetahough = math.pi/180
        # Accumulator threshold parameter. Only those lines are returned that get enough votes ( >threshold ). (190)
        threshhough = 190
        
        # Turn image to grey
        self.grey = cv2.cvtColor(imgsensor, cv2.COLOR_BGR2GRAY)

        # Edge detection. variables to adjust defined in class
        self.edges = cv2.Canny(self.grey, threshold1, threshold2)
        # Change from one to three channel image to write colored information
        self.edges3 = cv2.cvtColor(self.edges, cv2.COLOR_GRAY2BGR)

        # Line detection. Variables to adjust defined in the class. 
        # Type of return is numpy.ndarray
        self.lines = cv2.HoughLines( self.edges, rhohough, thetahough, threshhough)

        # Check if lines are detected and print to edge image
        if self.lines is not None:
            
            # Iterate through the found lines    
            for line in self.lines:
                
                # Convert straights from polar to two points representation
                # ρ is the distance from the coordinate origin (0,0) (top-left corner of the image). θ is the line rotation angle in radians 
                rho,theta = line[0]
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a*rho
                y0 = b*rho
                
                # Set up two points two draw line
                x1 = int(x0 + 1000*(-b))
                y1 = int(y0 + 1000*(a))
                x2 = int(x0 - 1000*(-b))
                y2 = int(y0 - 1000*(a))
                cv2.line(self.edges3,(x1,y1),(x2,y2),(0,0,255),2)
        
        return  self.edges3, self.lines
    
    def visualization(self, edgeimage): 
        """ Visualize greyscale image with detected lines

                :param  edgeimage [numpy.array (M,N,3)]
                :return -
  
            Uses interactive matplot plot"""

        if (self.errorcustom == True):
            return
        
        self.onstart += 1
        
        # Clear plot
        plt.clf()
            
        plt.imshow(edgeimage)
            
        # Draw new data on existing matplot figure
        plt.draw()
        plt.pause(0.00000001)
    
    def publish_lines(self, edgelines):
        """ Publish the extracted lines 

                :param  edgelines [numpy.ndarray (ρ,θ)]
                :return -
  
            No further details"""

        if (self.errorcustom == True):
            return 

        # Instantiate a new custom message type FloatArray
        #self.transmitlines = FloatArray()
        
        # Instantiate list to store the lines rho and theta seperated
        self.nprho = []
        self.nptheta = []
        
        # Check if lines are detected
        if edgelines is not None:
            
            # Write that lines were found to message
            #self.transmitlines.info = "Lines are detected" 
            
            # Convert numpy array of rho and theta line values of image object into python list
            for line in self.lines:
                
                # Get the first line of lines (however there should be only one line)
                rho,theta = line[0]
                
                # Append rho and theta to list
                self.nprho.append(rho)
                self.nptheta.append(theta)
    
            # Write rho and theta to message
            #self.transmitlines.rho = self.nprho
            #self.transmitlines.theta = self.nptheta

            self.get_logger().info("Total of lines detected: " + str(int(len(self.nprho)))+ "\n")
        
        else:
            # Write that no lines were found to message
            #self.transmitlines.info = "No lines are detected" 
            self.nprho = None
            self.nptheta = None
            
            # Warn that no line is detected  
            self.get_logger().warning("No lines are detected"+ "\n")
        
        # Publish message
        #self.image_pub.publish(self.transmitlines)
    
    def listener_callback(self,msg):
        """ Call methods for line extraction

                :param msg [ImageSensor.msg]
                :return - 
            Called upon new received imagedata."""
        # Get the time same as in Unity ([us])to measure transmittance difference
        t0 = datetime(2021, 7, 1)
        now = datetime.utcnow()
        seconds = (now - t0).total_seconds()
        
        ticks = int(seconds * 10**7)
        ticks_reduced= ticks - int(int(ticks /10000000000) * 10000000000) 
        ticks_reduced = int(ticks_reduced/10)
        self.get_logger().info("ROS time since 2021.7.1: " + str(ticks_reduced))
        self.get_logger().info("Unity time since 2021.7.1 " + str(msg.time))
        duration = ticks_reduced- msg.time 
        
        hs = open("latency_transmittance.csv","a")
        hs.write("transmittance_image[us]; " + str(duration)+ "\n")
        # Write methods execution time to file
        
        hs = open("latency_foxy_2.csv","a")

        
        
        # Set to true if an error occured
        self.errorcustom = False
        
        self.get_logger().info("Unity time since start: " + str(msg.time))
        
        timestart = time.time_ns()
        img = self.read_image(msg)
        timeend = time.time_ns()
        hs.write("get_image-read_image() [us]; " + str(timeend-timestart) + "\n")
        
        timestart = time.time_ns()
        edgeimg, edgelin = self.lines_extraction(img)
        timeend = time.time_ns()
        hs.write("get_image-lines_extraction() [us]; " + str(timeend-timestart)+ "\n")
        
        timestart = time.time_ns()
        self.visualization(edgeimg)
        timeend = time.time_ns()
        hs.write("get_image-visualization) [us]; " + str(timeend-timestart)+ "\n")
        
        timestart = time.time_ns()
        self.publish_lines(edgelin)
        timeend = time.time_ns()
        hs.write("get_image-publish_lines() [us]; " + str(timeend-timestart) + "\n")
        
def main(args=None):
    """ ROS2 main cycle

        :param -
        :return -
    No further details"""

    rclpy.init(args=args)

    # Interactive matplotlib window
    plt.ion()

    image_subscriber = get_image()

    rclpy.spin(image_subscriber)
    
    plt.close()
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
