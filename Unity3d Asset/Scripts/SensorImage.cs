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
using System.IO;

/// <summary>   This class writes data from an image sensor in a system jpeg file. </summary>
/// <remarks> - Shall be used in conjunction with a SensorManager to tell ROS2 information about the saved file.  </remarks>
public class SensorImage : MonoBehaviour
{
    private GameObject SensorManagerObj;
    private SensorManager SensorManagerScript;
    private bool saveToSensorManager = false; 
    public Camera ImageCamera;
    public int resolutionWidth = 640;
    public int resolutionHeight = 480;
    [Range(0, 100)]
    public int qualityLevel = 50;
    private Texture2D texture2D;
    private Rect rect;
    private float fixedUpdateFreq;
    private int triggerCounter;
    
    // (FixedUpdate executed every 0.02sec at default)
    [Tooltip("FixedUpdates per saving (FixedUpdate executed every 0.02sec at default)")]
    public int saveFreq = 1;



    private void Awake()
        {            
            // Find a SensorManager to access its data structure and save the lidarpointcloud

            // Find SensorManager object
            if (GameObject.Find("SensorManager") == null)
            {
                Debug.LogWarning("SensorManager object is not found. A SensorManager is necessary for sensor data exchange.\n "
                    + "Option: FileFormat-> Sensor Manager is currently not executable.");
                saveToSensorManager = false;
            }

            else
            {
                SensorManagerObj = GameObject.Find("SensorManager");

                // Find SensorManager script
                if (SensorManagerObj.GetComponent<SensorManager>() == null)
                {
                    Debug.LogWarning("SensorManager script is not found. A SensorManager is necessary for sensor data exchange.\n "
                    + "Option: FileFormat-> Sensor Manager is currently not executable.");
                    saveToSensorManager = false;
                }
                
                else
                {
                    SensorManagerScript = SensorManagerObj.GetComponent<SensorManager>();
                    saveToSensorManager = true;
                }
            }           
        }

    void Start()
        {
            InitializeGameObject();
            Camera.onPostRender += UpdateImage;
               // Is 50 by default. Time fixedDeltaTime is 0.02 when scaled with default value 1.
            fixedUpdateFreq = 1 / Time.fixedDeltaTime;
        }
    private void UpdateImage(Camera _camera)
        {
             ProcessTime.StartTime();   
            if (saveToSensorManager == true)
            {
                if (texture2D != null && _camera == this.ImageCamera)
                {
                    texture2D.ReadPixels(rect, 0, 0);
                    
                    byte[] Imagejpg = texture2D.EncodeToJPG(qualityLevel);
                    
                    // Image is saved to Operating system and then called from ROS2 because the connection from ROSBridge to ROS2
                    // corrupts the image file in the curren development status of the libraries.
                    File.WriteAllBytes(Application.dataPath + "/ImageSensor.jpeg", Imagejpg);
                    SensorManagerScript.ImageCam1.data = texture2D.EncodeToJPG(qualityLevel);
            
                    SensorManagerScript.ImageCam1.format = "jpeg";
                    SensorManagerScript.ImageCam1.newdata = true;
                    SensorManagerScript.ImageCam1.datapath = Application.dataPath + "/ImageSensor.jpeg";
                    
                    saveToSensorManager = false;
                    ProcessTime.EndTime("SensorImage-UpdateImage()");   
                }
            }
        }

    private void InitializeGameObject()
        {
            texture2D = new Texture2D(resolutionWidth, resolutionHeight, TextureFormat.RGB24, false);
            rect = new Rect(0, 0, resolutionWidth, resolutionHeight);
            ImageCamera.targetTexture = new RenderTexture(resolutionWidth, resolutionHeight, 24);
        }

    private void FixedUpdate()
    {
        if (triggerCounter % (fixedUpdateFreq / saveFreq) == 0 && triggerCounter != 0)
        {
            saveToSensorManager = true;
            triggerCounter = 0;
        }
        triggerCounter++;
    }

}
