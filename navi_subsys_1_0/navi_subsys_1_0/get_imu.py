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
import time
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime

# ROS custom message
from gnc_interfaces.msg import IMUSensor

class get_imu(Node):
    """ Subscribe and visualize IMU data

            :param (subscribes): topic imu1/accvel [IMUSensor.msg] 
            :return (publishes): 
        Visualizes acceleration x,y,z and the angular velocity x,y,z
        31.05.2021 j.mahler"""
        
    def __init__(self):
        """ Initialize imudata subscription

                :param - 
                :return -
            Set up node with topic and console output."""

        super().__init__('get_imu')
        
        # Info to show in terminal
        self.get_logger().info("Node: get_imu.")
        self.get_logger().info("Description: Subscribes to the acceleration and angular velocity of imu1.")
        self.get_logger().info("Subscribes: imu1/accvel [IMUSensor.msg]")
        self.get_logger().info("Publishes: <None>\n")
        
        # Array for plotting acceleration and angular velocity
        self.arracx, self.arracy, self.arracz = [0] * 50,[0] * 50,[0] * 50
        self.arrangx, self.arrangy, self.arrangz = [0] * 50,[0] * 50,[0] * 50
        self.arrtime = [0] * 50

        # only active in first cycles
        self.onstart = True

        self.subscription = self.create_subscription(
            IMUSensor, 'imu1/accvel', self.listener_callback, 10)

        self.subscription  # prevent unused variable warning
    
    def visualize_data(self,accvel):
        """ Write acceleration and angular velocity to interactive matlabplot

                :param accelx,y,z angularvelx,y,z [IMUSensor.msg]
                :return -   
            
            No further details"""

        # Make dataset with array length values to plot all at once in graph
        self.arracx.append(accvel.accelx) 
        self.arracy.append(accvel.accely) 
        self.arracz.append(accvel.accelz)

        self.arracx.pop(0)
        self.arracy.pop(0) 
        self.arracz.pop(0)

        self.arrangx.append(accvel.angularvelx) 
        self.arrangy.append(accvel.angularvely) 
        self.arrangz.append(accvel.angularvelz)  

        self.arrangx.pop(0) 
        self.arrangy.pop(0) 
        self.arrangz.pop(0)
        
        self.arrtime.append(accvel.time)

        self.arrtime.pop(0)
        
        # Delete previous plot, otherwise legend is printed multiple times
        plt.clf()
        
          # 10 seconds on x-axis displayed 
        if accvel.time >= 10:
            plt.xlim(accvel.time -5,accvel.time)
         
        # Acceleration
        plt.plot(self.arrtime,self.arracx,'r',linewidth = '2', label ="Acceleration x")
        plt.plot(self.arrtime,self.arracy, 'b',linewidth = '2', label = "Acceleration y")
        plt.plot(self.arrtime,self.arracz, 'g',linewidth = '2', label = "Acceleration z")
        # Angular velocity
        plt.plot(self.arrtime,self.arrangx,'y',linewidth = '0.25',marker='o', linestyle='dashed',label ="Angular Velocity x")
        plt.plot(self.arrtime,self.arrangy, 'black',linewidth = '0.25',marker='o',linestyle='dashed', label = "Angular Velocity y")
        plt.plot(self.arrtime,self.arrangz, 'orange',linewidth = '0.25',marker='o',linestyle='dashed', label = "Angular Velocity z")
        
        plt.legend()
        
        plt.pause(0.00000000001)     

    def listener_callback(self, msg):
        """ Call lidardata clustering methods

                :param msg [IMUSensor.msg]
                :return - 
            Called upon new received lidardata."""
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
        hs.write("transmittance_imu[us]; " + str(duration)+ "\n")
        self.get_logger().info("Unity time since start: "+str(msg.time))
        self.get_logger().info("Acceleration     x: "+str(msg.accelx)+" y: "+ str(msg.accely)+" z: "+ str(msg.accelz))
        self.get_logger().info("Angular Velocity x: "+str(msg.angularvelx)+" y: "+ str(msg.angularvely)+" z: "+ str(msg.angularvelz) +"\n")
        
        # Write methods execution time to file
        hs = open("latency_foxy_2.csv","a")
        
        timestart = time.time_ns()
        self.visualize_data(msg)
        timeend = time.time_ns()
        hs.write("get_imu-visualize_data() [us]; " + str(timeend-timestart)+ "\n")
        
def main(args=None):
    """ ROS2 main cycle

            :param -
            :return -
        No further details"""
        
    rclpy.init(args=args)

    IMUData = get_imu()
    
    # Interactive matplotlib window
    #plt.ion()
    plt.show()
   
    
    rclpy.spin(IMUData)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    IMUData.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()