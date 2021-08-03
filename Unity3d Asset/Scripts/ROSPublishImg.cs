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

using UnityEngine;
using System;
using System.Drawing;
using System.Collections.Generic;
using System.Diagnostics;
namespace RosSharp.RosBridgeClient
{
    /// <summary>  This class publishes data attained from a lidarsensor to a subsequent rosbridge2 </summary>
    /// <remarks> - It utilizes a custom message XYZLidar to send data </remarks>
    public class ROSPublishImg : UnityPublisher<MessageTypes.GncInterfaces.ImageSensor>

    {
        [SerializeField]
        private SensorManager ROSManagerObj;
        private MessageTypes.GncInterfaces.ImageSensor message  = new MessageTypes.GncInterfaces.ImageSensor();


        /// <summary>  This method searches for an object and class SensorManager </summary>
        /// <remarks> -  </remarks>
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

        /// <summary>  This method sets the data to be published  </summary>
        /// <remarks> - Data is set according to the types defined in the SensorManager  </remarks>
        private void SetMessage()
        {
            //message.data = ROSManagerObj.ImageCam1.data;
            
            // Byte arrays can not be transmitted with rosbridge, use short 
          
            //var shorts = Array.ConvertAll(ROSManagerObj.ImageCam1.data, b => (short)b);
            //message.doublebytedata = shorts;
            
            // Ticks since century begins

            message.format= ROSManagerObj.ImageCam1.format;
            message.datapath = ROSManagerObj.ImageCam1.datapath;
            
            ROSManagerObj.ImageCam1.newdata = false;
            
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

        /// <summary>  This method publishes an image</summary>
        /// <remarks> - Only publishes when there is new data found  </remarks>
        private void Update()
        {   
            if (ROSManagerObj.ImageCam1.newdata == true)
            {
                ProcessTime.StartTime();   
                SetMessage();
                ProcessTime.EndTime("ROSPublishImg-SetMessage()"); 
                ProcessTime.StartTime(); 
                Publish(message);
                ProcessTime.EndTime("ROSPublishImg-Publish()"); 
               
            }
        }
        
    }
}
