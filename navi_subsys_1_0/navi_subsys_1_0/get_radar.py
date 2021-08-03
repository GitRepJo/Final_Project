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
import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
import math
from pyapril import caCfar as cf
import time
from datetime import datetime

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from matplotlib.colors import ListedColormap, LinearSegmentedColormap

# ROS custom message
from gnc_interfaces.msg import RadarSensor

class get_radar(Node):
    """ Filter a radar image with SO_CFAR

            :param (subscribes): topic radar_img [RadarSensor.msg] 
            :return (publishes): 
        This ROS2 node filters radar data with the OS-CFAR algorithm.
        08.06.2021 j.mahler"""
        
    def __init__(self):
        """ Initialize radardata subscription

                :param - 
                :return -
            Set up node with topic and console output."""

        super().__init__('radar1_img')
        
        # Info to show in terminal
        self.get_logger().info("Node: get_radar.")
        self.get_logger().info("Description: Subscribes to the image data of radar1.")
        self.get_logger().info("Subscribes: radar1/img [RadarSensor.msg]")
        self.get_logger().info("Publishes: <None>\n")
        
        self.publisher_marker = self.create_publisher(MarkerArray,'/radar1/marker',10)
        
        self.subscription = self.create_subscription(
            RadarSensor, 'radar1/img', self.listener_callback, 10)

        self.subscription  # prevent unused variable warning
    
    def set_marker(self, min, max):
        """ Create a ROS cube marker message 

                    :param - center (x,y,z)
                        center of the cube
                    :param - scale (x,y,z) 
                        extension of the cube ( 1= 1m)
                    :return - marker[visualization_msgs/Marker]
                        defines the boundingbox with a center and extension from this center(scale)
                No further details"""
        
        # Set up Marker message
        marker = Marker()
        marker.ns = '/radar1'
        
        # Create a unique id based on the time (of existance of the node) <- not sure about this
        currenttime = get_radar.get_clock(self).now().to_msg()
        marker.id = int(currenttime.nanosec/1000)
        
        marker.header.frame_id = 'map'
        marker.header.stamp = get_radar.get_clock(self).now().to_msg()
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        
        # Calculate xyz extension based on cluster extension
        marker.scale.x = max[0] - min[0]
        marker.scale.y = max[1] - min[1]
        marker.scale.z = max[2] - min[2]
        
        # Calculate position based on half way from cluster min value to cluster max value
        marker.pose.position.x = (min[0] + max[0]) / 2
        marker.pose.position.y = (min[1] + max[1]) / 2
        marker.pose.position.z = (min[2] + max[2]) / 2

        return marker
    
    def fixed_threshold(self,x, threshold):
        
        peak_idx  = []
        num_cells = np.size(x)
        
        for i,data in enumerate(x):
            if x[i] > threshold: 
                peak_idx.append(i)
        
        return peak_idx, num_cells
    
    def detect_peaks(self, x, num_train, num_guard, rate_fa):
        """
        Detect peaks with CFAR algorithm.
        
        :param x [int16[]]
            reflectance data of the radar 
        :param num_train [int]
            Number of training cells.
        :param num_guard [int] 
            Number of guard cells.
        :param rate_fa[float] 
            False alarm rate. 
        :return peak_idx [np.array((int) n)]
            index of found signals
        :return num_cells [int]
            Total number of cells
        
        See http://tsaith.github.io/detect-peaks-with-cfar-algorithm.html
        """
        num_cells = np.size(x)
        num_train_half = round(num_train / 2)
        num_guard_half = round(num_guard / 2)
        num_side = num_train_half + num_guard_half
    
        alpha = num_train*(rate_fa**(-1/num_train) - 1) # threshold factor
        
        peak_idx = []
        for i in range(num_side, num_cells - num_side):
            
            if i != i-num_side+np.argmax(x[i-num_side:i+num_side+1]): 
                continue
            
            sum1 = np.sum(x[i-num_side:i+num_side+1])
            sum2 = np.sum(x[i-num_guard_half:i+num_guard_half+1]) 
            p_noise = (sum1 - sum2) / num_train 
            threshold = alpha * p_noise
            
            if x[i] > threshold: 
                peak_idx.append(i)
        
        peak_idx = np.array(peak_idx, dtype=int)
        
        return peak_idx, num_cells

    def visualize_cluster(self,data, cellsPerBurst,cellExtension,resolution):
        """ Write radar image to interactive matlabplot

                :param data [bytearray()]
                    radar reflection data for all cells in 360°, first values for burst at 0°...
                :param cellsPerBurst [int]
                :param cellExtension [int]
                :param resolution [int] 
                    encoding of the intensities in bit, max value is 8.
                :return -   
            
            Write in polar coordinates that resemble the burst"""
        
        # Clear plot
        plt.clf()
        
         # Calculate number of bursts
        burstPerRotation = len(data)/cellsPerBurst

        # Create Grid to map intensities to
        thetarot = np.arange(0, 2*np.pi + (2*np.pi/burstPerRotation), 2*np.pi/burstPerRotation)
        radrot = np.arange(0, cellsPerBurst*cellExtension, cellExtension)
        allrad,alltheta = np.meshgrid(radrot,thetarot)
        # Add an extra zero burst that is not visualized but necessary to obtain a 
        radzeros = np.zeros(cellsPerBurst)
        int = np.append(data,radzeros) 
        # Convert intensities to colors
       
        # Match matrix size
        color = int.reshape(alltheta.shape)
    
        # Set up custom color map from black to white
        N = 256
        vals = np.ones((N, 4))
        vals[:, 0] = np.linspace(256/256, 0/256, N)
        vals[:, 1] = np.linspace(256/256, 0/256, N)
        vals[:, 2] = np.linspace(256/256, 0/256, N)
        newcmp = ListedColormap(vals)
        
        # Set up plot
        ax2 = plt.axes(projection = 'polar')
        ax2.set_theta_zero_location('N')
        ax2.pcolormesh(alltheta,allrad, color, cmap= newcmp,vmin = 0,vmax= 255)
        ax2.pcolormesh(alltheta,allrad, color, cmap= 'Greys',vmin = 0 ,vmax= 200)
        
        # Draw new data on existing matplot figure
        plt.draw()
        plt.pause(0.00000000001)   

        self.get_logger().info("Burst per rotation: "+ str(burstPerRotation))
        self.get_logger().info("Cell extension: " + str(cellExtension)+ "\n")  
    
    def bounding_box(self, signalidx, cellnr, cellsPerBurst, cellExtension):
        """ Create bounding boxes of radarsignal

                :param - signal [np.array((int) n)]
                    index of detected signals
                :param cellnr [int]
                    total number of cells
                :param - cellsPerBurst [int] 
                    cells in one burst
                :param - cellExtension [int]
                    max cell extension
                :return - box [visualization_msgs/Marker[]]
                    bounding boxes
            No further information
            """
        # To Save all marker of this batch
        markerArray = MarkerArray()
        
        for counter,signal in enumerate(signalidx):
         
            # Stepdistance in degree for one turn
            degPerBurst = 360/(cellnr/cellsPerBurst)
   
            # According burst number to the signal peak index
            burstNumber = np.ceil(signalidx[counter]/cellsPerBurst)
          
            # According theta for signal peak index
            theta = burstNumber * degPerBurst
         
            # Cell number of the signal peak index relative to one burst
            burstcellnr = signalidx[counter] - (burstNumber - 1) * cellsPerBurst
            
            radius = burstcellnr * cellExtension
            
            # Convert to cartesian coordinates
            x = float(radius  * np.cos(math.radians(theta)))
            y = float(radius  * np.sin(math.radians(theta)))
            
            z = 0.0
            
            cellmin = (x - cellExtension ,y - 2* cellExtension,z)
            cellmax = (x + 2 * cellExtension, y + cellExtension , z + 0.2)
            
            cubemarker = self.set_marker(cellmin,cellmax)
            markerArray.markers.append(cubemarker)
        
        return markerArray
    
    def publish(self, marker):
        """ Publish MarkerArray

                :param marker [visualization_msgs/Marker[]]
                :return - 

            No further details"""
        
        # Set up/publish a dummy MarkerArray to delete all previous marker
        emptyMarker = Marker()
        emptyMarker.action = emptyMarker.DELETEALL
        emptyMarkerArray = MarkerArray()
        emptyMarkerArray.markers.append(emptyMarker)
        self.publisher_marker.publish(emptyMarkerArray)
        
        # Publish the calculated markerarray
        self.publisher_marker.publish(marker)
    
    def listener_callback(self, msg):
        """ Call radar data methods

                :param msg [Radar.msg]
                :return - 
            Called upon new received radardata."""
        
        # Get the time same as in Unity to measure transmittance difference
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
        hs.write("transmittance_radar[us]; " + str(duration)+ "\n")

        
        cellext = 4 
        
        peaks, totalcells = self.detect_peaks(msg.data, 100, 2, 0.1)
   

        # Write methods execution time to file
        hs = open("latency_foxy_2.csv","a")

        timestart = time.time_ns()
        self.visualize_cluster(msg.data, msg.nrcell, cellext, msg.signallevel)
        timeend = time.time_ns()
        hs.write("get_radar-visualize_cluster() [us]; " + str(timeend-timestart)+ "\n")
        
        timestart = time.time_ns()
        radarmarker = self.bounding_box(peaks, totalcells, msg.nrcell, cellext)
        timeend = time.time_ns()
        hs.write("get_radar-bounding box() [us]; " + str(timeend-timestart)+ "\n")

        timestart = time.time_ns()
        self.publish(radarmarker)
        timeend = time.time_ns()
        hs.write("get_radar-publish() [us]; " + str(timeend-timestart)+ "\n")
        
def main(args=None):
    """ ROS2 main cycle

            :param -
            :return -
        No further details"""
        
    rclpy.init(args=args)

    oscfarFilter = get_radar()
    
    # Interactive matplotlib window
    plt.ion()
    plt.show()
    
    rclpy.spin(oscfarFilter)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    oscfarFilter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()