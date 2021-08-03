/*
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
*/

using System.Diagnostics;
using UnityEngine;
using System;
namespace RosSharp.RosBridgeClient
{
    public class ROSPublishXYZ : UnityPublisher<MessageTypes.GncInterfaces.XYZLidar>
    /// <summary> This class publishes data attained from a lidarsensor to a subsequent rosbridge2.
    /// It utilizes a custom messafe XYZLidar, defined in the ROS workspace in the package GncInterfaces  </summary>
    
    {
        [SerializeField]
        private SensorManager ROSManagerObj;

        private MessageTypes.GncInterfaces.XYZLidar message  = new MessageTypes.GncInterfaces.XYZLidar();



        private void Awake()
        {

            if (ROSManagerObj == null)
            {
                if (gameObject.GetComponent<SensorManager>() == null)
                {
                    UnityEngine.Debug.LogError("No SensorManager object is found. A SensorManager is necessary for sensor data exchange.");
                }
                else
                {
                    ROSManagerObj = gameObject.GetComponent<SensorManager>();
                }
            }
        }
        
        protected override void Start()
        { 
            base.Start();
        }

        private void SetMessage()
        {

            // Switch y and z as that is commonly used outside of Unity3d
            message.x = ROSManagerObj.PointCloud1.xcoord;
            message.y = ROSManagerObj.PointCloud1.zcoord;
            message.z = ROSManagerObj.PointCloud1.ycoord;
        

            ROSManagerObj.PointCloud1.newdata = false;
            // Calculate ticks from beginning of date
            DateTime centuryBegin = new DateTime(2021, 7, 1);   
            DateTime currentDate = DateTime.UtcNow;

            long elapsedTicks = currentDate.Ticks - centuryBegin.Ticks;
            TimeSpan elapsedSpan = new TimeSpan(elapsedTicks);
            
            // Reduce the time span so that it fits into an integer32
            // Ommit MSB's of Ticks, as they are not relevant to measure short time intervalls
            long reducedTicks = elapsedSpan.Ticks - (elapsedSpan.Ticks/10000000000 * 10000000000);
            // Ommit LSB as it is zero due to resolution
            long ticks = reducedTicks/ 10;
            message.time =  Convert.ToInt32(ticks);
        }

        
        private void Update()
        {
            if (ROSManagerObj.PointCloud1.newdata == true)
            {
                ProcessTime.StartTime();
                SetMessage();
                ProcessTime.EndTime("ROSPublishXYZ-SetMessage");
                ProcessTime.StartTime();
                Publish(message);
                ProcessTime.EndTime("ROSPublishXYZ-Publish()");
            }
        }
    }
}
