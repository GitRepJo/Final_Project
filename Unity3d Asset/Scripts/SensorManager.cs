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

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Diagnostics;
using System.Threading;
using System;
using System.IO;


public class ProcessTime
{
    static int startTime = 0;

    /// <remarks> For debugging purposes, place before process to measure time.
    /// Turn off by setting SensorManager.measureTime to false</remarks>    
    public static void StartTime()
	{

        
        if (SensorManager.measureTime == true)
        {
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
            // Resolution in us
            startTime =  Convert.ToInt32(ticks);
        }
        
        /* // Deprecated measurement of time, results time format not clear
        
        {
            // Get current time stamp
            double timestamp = Stopwatch.GetTimestamp();
		    startTime = 1_000_000_000.0 * timestamp / Stopwatch.Frequency;
        }
        */
	}
    /// <remarks> For debugging purposes, place after process to measure time.
    /// Turn off by setting SensorManager.measureTime to false </remarks>    
    public static void EndTime(string processName)
    {
        if (SensorManager.measureTime == true & startTime != 0)
        {
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
            int endTime =  Convert.ToInt32(ticks);
            
            // Calculate time span
            int intervall =  endTime - startTime;
            
            // Write to file
            using (StreamWriter sw =  File.AppendText(Application.dataPath + "/latency_unity_2.csv"))
            {
                sw.WriteLine(processName+ " [us] ; " + intervall);
            }
        }
        /* // Deprecated measurement of time, results time format not clear
        {
            // Get current time stamp
            double timestamp = Stopwatch.GetTimestamp();
            double endTime = 1_000_000_000.0 * timestamp / Stopwatch.Frequency;
        
            // Calculate time span
            long intervall = ((long) endTime - (long) startTime);
            
            // Write to file
            using (StreamWriter sw =  File.AppendText(Application.dataPath + "/latency_unity_2.csv"))
            {
                sw.WriteLine(processName+ " ; " + intervall);
            } 
            
        } */
    }   
}

/// <summary>  Declare the datatypes of the IMIU sensors data  </summary>
/// <remarks> - Shall be used for dataexchange of IMU sensor and ros publisher  </remarks>
public class IMUData

    {
        public float accelx;
        public float accely;
        public float accelz;
        public float angularvelx;
        public float angularvely;
        public float angularvelz;
        public bool newdata;
    }

/// <summary>  Declare the datatypes of the lidar sensors data  </summary>
/// <remarks> - Shall be used for dataexchange of lidar sensor and ros publisher  </remarks>
public class LidarData

    {
        public float[] xcoord;
        public float[] ycoord;
        public float[] zcoord;
        public bool newdata;
    }

/// <summary>  Declare the datatypes of the radar sensors data  </summary>
/// <remarks> - Shall be used for dataexchange of radar sensor and ros publisher  </remarks>
public class RadarData

    {
        // acceptable values are jpeg or png
        public int numcell;
        public int levelsignal;
        public byte[] data;
        public bool newdata;

    }
/// <summary>  Declare the datatypes of the image sensors data  </summary>
/// <remarks> - Shall be used for dataexchange of image sensor and ros publisher  </remarks>
public class ImageData

    {
        // acceptable values are jpeg or png
        public string format;
        public string datapath;
        public bool newdata;
        public byte[] data;

    }

/// <summary>  This class contains the datatypes for the sensor data  </summary>
/// <remarks> - Every type of data of the sensors that is to be exchanged is defined here   </remarks>
public class SensorManager : MonoBehaviour

{
    public static bool measureTime = true;
    public LidarData PointCloud1 = new LidarData();
    public ImageData ImageCam1 = new ImageData();
    public IMUData IMU1 = new IMUData();
    public RadarData RadarImage1 = new RadarData();
    
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
    }
}


