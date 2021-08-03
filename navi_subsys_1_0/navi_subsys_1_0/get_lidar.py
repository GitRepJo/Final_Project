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
from mpl_toolkits.mplot3d import Axes3D
import time
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from rclpy.duration import Duration
from datetime import datetime

# Dbscan
#from sklearn.datasets import make_blobs
from sklearn.preprocessing import StandardScaler
from dbscan import DBSCAN

# ROS custom message
from gnc_interfaces.msg import XYZLidar

class get_lidar(Node):
    """ Cluster lidar 3d data

            :param (subscribes): topic lidar1_xyz [XYZLidar.msg] 
            :return (publishes): 
        This ROS2 node clusters 3d data points with the dbscan algorithm.
        04.05.2021 j.mahler"""
        
    def __init__(self):
        """ Initialize lidardata subscription

                :param - 
                :return -
            Set up node with topic and console output."""

        super().__init__('lidar1_xyz')
        
        # Info to show in terminal
        self.get_logger().info("Node: get_lidar.")
        self.get_logger().info("Description: Subscribes to the xyz data of lidar1.")
        self.get_logger().info("Subscribes: lidar1/xyz [XYZLidar.msg]")
        self.get_logger().info("Publishes: <None>\n")
        
        self.publisher_marker = self.create_publisher(MarkerArray,'/lidar1/marker',10)
        
        self.subscription = self.create_subscription(
            XYZLidar, 'lidar1/xyz', self.listener_callback, 10)

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
        marker.ns = '/lidar1'
        
        # Create a unique id based on the time (of existance of the node) <- not sure about this
        currenttime = get_lidar.get_clock(self).now().to_msg()
        marker.id = int(currenttime.nanosec/1000)
        
        marker.header.frame_id = 'map'
        marker.header.stamp = get_lidar.get_clock(self).now().to_msg()
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
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
    
    def dbscan(self, coordx,coordy,coordz):

        """ Apply DBSCAN (Density-based spatial clustering of applications with noise

                :param coordy,coordy,coordz [np.float np.array(n,1)]
                :return points [np.float np.array(n,3)]
                    one 3D point is one row
                :return labels [np.int32 np.array(n,1)] 
                    cluster ID in same order of points
                :return core_samples_mask [np.bool np.array(n,1)]
                    mask core point of according cluster      
            Use the DBSCAN algorithm as described in https://pypi.org/project/dbscan/."""
    

        # Check for data integrety
        if len(coordx) != len(coordy) or len(coordx) != len(coordz):
            self.get_logger().error("Length of x,y and z are not matching, some 3d points are thus not defined.")

        # For the pointcloud data, one 3D point is one column
        points =  np.empty((len(coordx),0), float)
        
        points = np.append(points, np.array([coordx]).transpose(), axis=1)
        points = np.append(points, np.array([coordy]).transpose(), axis=1)
        points = np.append(points, np.array([coordz]).transpose(), axis=1)
        
        labels, core_samples_mask = DBSCAN(points, eps=1, min_samples=10)
        
        return points, labels, core_samples_mask

    def bounding_box(self, pointcloud, cluster, resolution):
        """ Create bounding boxes of lidarcluster

                :param - pointcloud [np.float np.array(n,3)]
                    one 3D point is one row
                :param - cluster [np.int32 np.array(n,1)] 
                    cluster id of 3d points 
                :param - resolution
                    max cell extension, counts for x,y,z
                :return - box [visualization_msgs/Marker[]]
                    bounding boxes
            Split up cluster that are larger than specified resolution"""
        
        # To Save all marker of this batch
        markerArray = MarkerArray()

        clustervalues = set(cluster)
        
        # Iterate through clusternumbers
        for k in clustervalues:
            
            # Get all values of the current clusternumber
            clustermask = (cluster == k)
            clusterpoints = (pointcloud[clustermask])
            
            # Disregard noise
            if k == -1:
                continue
                
            # Find cluster extension
            xyzmaxindex = np.argmax(clusterpoints, axis = 0)
            xyzminindex = np.argmin(clusterpoints, axis = 0)
            
            xmax = clusterpoints[xyzmaxindex[0], 0]
            ymax = clusterpoints[xyzmaxindex[1], 1]
            zmax = clusterpoints[xyzmaxindex[2], 2]

            xmin = clusterpoints[xyzminindex[0], 0]
            ymin = clusterpoints[xyzminindex[1], 1]
            zmin = clusterpoints[xyzminindex[2], 2]

            clustermax = (xmax, ymax, zmax)
            clustermin = (xmin, ymin, zmin)
            
            diff = (xmax - xmin, ymax - ymin, zmax - zmin)

            maxdiff = max(diff)

            # Cluster can be converted directly into an marker message
            if (maxdiff <= resolution):
                
                cubemarker = self.set_marker(clustermax,clustermin)
                
                markerArray.markers.append(cubemarker)
                
            # cluster has to be split up to suffice resolution request
            else:
                
                # Calculate number of partitions to suffice resolution requirement
                # With ceiling, the next higher number of partitions is used
                cubedivisionx  = int(np.ceil(float(diff[0]/resolution)))
                cubedivisiony  = int(np.ceil(float(diff[1]/resolution)))
                cubedivisionz  = int(np.ceil(float(diff[2]/resolution)))                    
                
                # Access ranges of sub cluster. Subcluster has dimension resolution*resolution*resolution
                for xres in range(cubedivisionx):
                    # Use the x clustermin/max value for bounding box min/max, as they are below resolution
                    if diff[0] < resolution:
                        boxminx = clustermin[0]
                        boxmaxx = clustermax[0]
                    # Calculate new bounding box min/max based on resolution
                    else:
                        boxminx = clustermin[0] + (resolution * xres) 
                        boxmaxx = boxminx + resolution
                    
                    for yres in range(cubedivisiony):
                        if diff[1] < resolution:
                            boxminy = clustermin[1]
                            boxmaxy = clustermax[1]
                        else:
                            boxminy = clustermin[1] + (resolution * yres) 
                            boxmaxy = boxminy + resolution
                        
                        for zres in range(cubedivisionz):
                            if diff[2] < resolution:
                                boxminz = clustermin[2]
                                boxmaxz = clustermax[2]
                            else:
                                boxminz = clustermin[2] + (resolution * zres)
                                boxmaxz = boxminz + resolution
                            
                            # Dimensions of new box
                            boxmin = (boxminx,boxminy,boxminz)
                            boxmax = (boxmaxx,boxmaxy,boxmaxz)
                            
                            valinbox = False
                            
                            # Is a x,y,z cluster value in the box? If not, there is no object, thus do not create a marker.
                            for indx,xbox in enumerate(clusterpoints[:,0]):
                                if clusterpoints[indx,0] > boxmin[0] and clusterpoints[indx,0] < boxmax[0]:
                                    if clusterpoints[indx,1] > boxmin[1] and clusterpoints[indx,1] < boxmax[1]:
                                        if clusterpoints[indx,2] > boxmin[2] and clusterpoints[indx,2] < boxmax[2]:
                                            valinbox = True
                            
                            # Check if point in box is found
                            if valinbox == True:   
                                cubemarker = self.set_marker(boxmax,boxmin)
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

    def visualize_cluster(self,pointCoord,clusterID,mask):
        """ Write cluster to interactive matlabplot

                :param pointCoord [np.float np.array(n,3)]
                :param clusterID [np.int32 np.array(n,1)]
                    cluster ID in same order of pointCoord
                :param mask [np.bool np.array(n,1)] 
                    mask core point of according cluster 
                :return -   
            
            No further details"""
        # Clear plot
        plt.clf()
        
        points = pointCoord
        cluster = clusterID
        center = mask

        # Get number of elements with distinction
        dQuantity = len(set(cluster)) 
        # Noise marked by -1 is not considered a cluster
        clusterQuantity = dQuantity - (1 if -1 in cluster else 0)
        noiseQuantity = list(cluster).count(-1)
        clusterValues = set(cluster)

        self.get_logger().info("Number of clusters found: "+ str(clusterQuantity))
        self.get_logger().info("Points discarded as noise: " + str(noiseQuantity)+ "\n")
       
        # One color for each cluster
        colors = [plt.cm.Spectral(each)
            for each in np.linspace(0, 1, len(clusterValues))]
        
        # 3d plot is required
        ax = plt.axes(projection ='3d')
        ax.azim = 180
        ax.elev = 60    
        ax.xlabel = "x [x global Unity3d Coordinatesystem]"
        ax.ylabel = "y [y global Unity3d Coordinatesystem]"
        ax.zlabel = "z [z global Unity3d Coordinatesystem]"
        ax.set_title('Cluster detected:' + str(clusterQuantity) + "\n Points discarded as noise:" + str(noiseQuantity))

        for k, col in zip(clusterValues, colors):
            class_member_mask = (cluster == k)
            clusterpoints = points[class_member_mask]
            
            #Plot noise lightgrey
            if k == -1:
                ax.scatter(clusterpoints[:,0],clusterpoints[:,1],clusterpoints[:,2], c="lightgrey", alpha = 0.1)  
            # Plot cluster in different color
            else: 
                ax.scatter(clusterpoints[:,0],clusterpoints[:,1],clusterpoints[:,2], color= col)   
            
       
        # Draw new data on existing matplot figure
        plt.draw()
        plt.pause(0.00000000001)     

    def listener_callback(self, msg):
        """ Call lidardata clustering methods

                :param msg [XYZLidar.msg]
                :return - 
            Called upon new received lidardata."""
        # Get the time same as in Unity to measure transmittance difference
        t0 = datetime(2021, 7, 1)
        now = datetime.utcnow()
        seconds = (now - t0).total_seconds()
        
        ticks = int(seconds * 10**7)
        ticks_reduced= ticks - int(int(ticks /10000000000) * 10000000000) 
        ticks_reduced = int(ticks_reduced/10)
        # Ticks reduced are in useconds
        self.get_logger().info("ROS time since 2021.7.1: " + str(ticks_reduced))
        self.get_logger().info("Unity time since 2021.7.1 " + str(msg.time))
        duration = ticks_reduced- msg.time 
        
        hs = open("latency_transmittance.csv","a")
        hs.write("transmittance_lidar[us]; " + str(duration)+ "\n")
        
        lidar1 = msg
        
        x = lidar1.x
        y = lidar1.y
        z = lidar1.z

        # Resolution of boundingbox (extension) in meters
        boundingboxres = 5
        
        # Write methods execution time to file
        hs = open("latency_foxy_2.csv","a")
        
        self.get_logger().info("Unity time since start" + str(lidar1.time))
        
        timestart = time.time_ns()
        lidarPoints, cluster, cores = self.dbscan(x,y,z)
        timeend = time.time_ns()
        hs.write("get_lidar-dbscan() [ms/us]; " + str(timeend-timestart)+ "\n")

        timestart = time.time_ns()
        self.visualize_cluster(lidarPoints, cluster, cores)
        timeend = time.time_ns()
        hs.write("get_lidar-visualize_cluster() [us]; " + str(timeend-timestart)+ "\n")

        timestart = time.time_ns()
        lidarmarker = self.bounding_box(lidarPoints, cluster, boundingboxres)
        timeend = time.time_ns()
        hs.write("get_lidar-bounding_boxes() [us]; " + str(timeend-timestart)+ "\n")

        timestart = time.time_ns()
        self.publish(lidarmarker)
        timeend = time.time_ns()
        hs.write("get_lidar-publish() [us]; " + str(timeend-timestart)+ "\n")
        
def main(args=None):
    """ ROS2 main cycle

            :param -
            :return -
        No further details"""
        
    rclpy.init(args=args)

    dbscanCluster = get_lidar()
    
    # Interactive matplotlib window
    plt.ion()
    plt.show()
    
    rclpy.spin(dbscanCluster)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    dbscanCluster.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()