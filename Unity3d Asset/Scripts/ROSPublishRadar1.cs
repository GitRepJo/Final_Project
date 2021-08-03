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
using System.Diagnostics;
namespace RosSharp.RosBridgeClient

{
    public class ROSPublishRadar1 : UnityPublisher<MessageTypes.GncInterfaces.RadarSensor>
    /// <summary> This class publishes data attained from a radarsensor to a subsequent rosbridge2.
    /// It utilizes a custom message RadarSensor, defined in the ROS workspace in the package GncInterfaces  </summary>
    
    {
        [SerializeField]
        private SensorManager ROSManagerObj;

        private MessageTypes.GncInterfaces.RadarSensor message  = new MessageTypes.GncInterfaces.RadarSensor();



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

            message.data = Array.ConvertAll(ROSManagerObj.RadarImage1.data, b => (short)b);
            message.signallevel = ROSManagerObj.RadarImage1.levelsignal;
            message.nrcell = ROSManagerObj.RadarImage1.numcell;
            
            ROSManagerObj.RadarImage1.newdata = false;
            
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
            if (ROSManagerObj.RadarImage1.newdata == true)
            {   
                
                ProcessTime.StartTime();   
                SetMessage();
                ProcessTime.EndTime("ROSPublishRadar1-SetMessage()");
                
                //ProcessTime.StartTime();   
                Publish(message);
                ProcessTime.EndTime("ROSPublishRadar1-Publish()");
            }
        }
        
    }
}
