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
using UnityEngine.UI;
using TMPro;

// Set the camera that is rendered to screen from a dropdownmenu.
// Assign the according cameras in the inspector.
// To add or change cameras make sure to change the gameobject name and the item name in the list to avoid confusion.

public class UIChooseCam : MonoBehaviour
{   
    // Variable for mode of view, look below at items in list for further information. 
    public GameObject cam1;
    public GameObject cam2;
    public GameObject cam3;
    public GameObject cam4;
    public GameObject cam5;

    // Name as shown in the dropdownmenu
    public string name_cam1 = "Description of the view as in the dropdownpanel";
    public string name_cam2 = "Description of the view as in the dropdownpanel";
    public string name_cam3 = "Description of the view as in the dropdownpanel";
    public string name_cam4 = "Description of the view as in the dropdownpanel";
    public string name_cam5 = "Description of the view as in the dropdownpanel";

    // Start is called before the first frame update
    void Start()
    {
        //new GameObject() cam1;
        // Get the dropdown object of the gameobject the script is attached to.
        var dropdown = transform.GetComponent<TMP_Dropdown>();
        // Delete the options in the Inspector because this script is used.
        dropdown.options.Clear();
      
        // Append new items to list
        List<string> items = new List<string>();
        items.Add("Perspective "+name_cam1);
        items.Add("Perspective "+name_cam2);
        items.Add("Perspective "+name_cam3);
        items.Add("Perspective "+name_cam4);
        items.Add("Perspective "+name_cam5);       
        

        // Fill dropdown with items
        foreach (var item in items)
        {
            dropdown.options.Add(new TMP_Dropdown.OptionData() {text = item}); 
        }

        
        // Start with first value
        dropdown.value = 0;
         

        // Iniital execution
        DropdownItemSelected(dropdown);
        
        // Listen for value changed
        dropdown.onValueChanged.AddListener(delegate {DropdownItemSelected(dropdown);});

    }
    
    // Cases which are selected according to user selection.
    // Set the Camera to the value choosen to active
    void DropdownItemSelected(TMP_Dropdown dropdown)
    {
        int index = dropdown.value;
        
       
        

        switch (index)
       {
            case 0:
                cam1.SetActive(true);
                cam2.SetActive(false);
                cam3.SetActive(false);
                cam4.SetActive(false);
                cam5.SetActive(false);
                Debug.Log("Camera " + name_cam1 + " has been selected.");
                break;
            case 1:
                cam1.SetActive(false);
                cam2.SetActive(true);
                cam3.SetActive(false);
                cam4.SetActive(false);
                cam5.SetActive(false);
                Debug.Log("Camera " + name_cam2 + " has been selected.");
                break;
            case 2:
                cam1.SetActive(false);
                cam2.SetActive(false);
                cam3.SetActive(true);
                cam4.SetActive(false);
                cam5.SetActive(false);
                Debug.Log("Camera " + name_cam3 + " has been selected.");
                break;
            case 3:
                cam1.SetActive(false);
                cam2.SetActive(false);
                cam3.SetActive(false);
                cam4.SetActive(true);
                cam5.SetActive(false);
                Debug.Log("Camera " + name_cam4 + " has been selected.");
                break;
            case 4:
                cam1.SetActive(false);
                cam2.SetActive(false);
                cam3.SetActive(false);
                cam4.SetActive(false);
                cam5.SetActive(true);
                Debug.Log("Camera " + name_cam5 + " has been selected.");
                break;
        
       }
    }
}
