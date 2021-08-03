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

public class UIChooseRes : MonoBehaviour
{   
    // Object for initial text on drop down menu
    private TMPro.TMP_Text initialtext;

    // To wait for resolution confirmation
    private IEnumerator coroutine;

    // Button for resolution confirmation
    public GameObject Confirm;

    // Button to confirm resolution choice
    public Button clickresolution;

    // save confirmation of user 
    public bool resconf;

    // Start is called before the first frame update
    void Start()
    {
       
        
        // Set Confirmation button for resolution to inactive, not used yet
        Confirm.SetActive(false);
        
        // Get the dopdown object of the gemeobject the script is attached to.
        var dropdown = transform.GetComponent<TMP_Dropdown>();

        // Add listener to confirm button click event in case resolution is confirmed
        clickresolution.onClick.AddListener(delegate {ConfirmClicked(dropdown);});

        // Delete the options in the Inspector because this script is used.
        dropdown.options.Clear();

        // Set initial screen resolution to smallest res possible
        Screen.SetResolution(640, 360, false);

        // Append new items to list
        List<string> items = new List<string>();
        items.Add("nHD 640 x 360");
        items.Add("HD 1366 x 768");
        items.Add("FHD 1920 x 1080");
        items.Add("4K UHD 3840 x 2160");
        items.Add("HD Fullscreen");
        items.Add("FHD Fullscreen");
        items.Add("UHD Fullscreen");
        
        // Fill dropdown with items
        foreach (var item in items)
        {
            dropdown.options.Add(new TMP_Dropdown.OptionData() {text = item}); 
        }
        
        // Start with lowest resolution
        Debug.Log("Resolution set to nHD 640 x 360.");
        Screen.SetResolution(640, 360, false);
        
        // Listen for value changed
        dropdown.onValueChanged.AddListener(delegate {DropdownItemSelected(dropdown);}); 

    }
    // Sets the new resolution
    void DropdownItemSelected(TMP_Dropdown dropdown)
    {
        // Disable resolution confirmation button
        Confirm.SetActive(false);
        
        // Set user confirm value to false
        resconf = false;
        
        // Cases for resolution selection
        int index = dropdown.value;
        
        switch (index)
        {
            case 0:
                Debug.Log("Resolution set to nHD 640 x 360.");
                Screen.SetResolution(640, 360, false);
                break;
            case 1:
                Debug.Log("Resolution set to HD 1366 x 768.");
                Screen.SetResolution(1366, 768, false);
                break;
            case 2:
                Debug.Log("Resolution set to FHD 1920 x 1080.");
                Screen.SetResolution(1920, 1080, false);
                break;
            case 3:
                Debug.Log("Resolution set to UHD 3840 x 2160.");
                Screen.SetResolution(3840, 2160, false);
                break;
            case 4:     
                Debug.Log("Resolution set to HD Fullscreen.");
                Screen.SetResolution(1366, 768, true);
                break;
            case 5:     
                Debug.Log("Resolution set to FHD Fullscreen.");
                Screen.SetResolution(1920, 1080, true);
                break;
            case 6:     
                Debug.Log("Resolution set to UHD Fullscreen.");
                Screen.SetResolution(3840, 2160, true);
                break;
        }
        
        // New Coroutine object
        coroutine = ConfirmRes(dropdown);
        
        // Start Coroutine to wait for user input
        StartCoroutine(coroutine);

    }  
    // Wait 5 seconds 
    private IEnumerator ConfirmRes(TMP_Dropdown dropdown)
    {
        // Allow user to select Confirm button
        Confirm.SetActive(true);
        resconf = false;
        
        // Wait seconds for user input
        yield return new WaitForSeconds(5);

        // If no user confirmation after seconds, set back resolution to lowest possible value
        if (resconf == false)
        {
            // Reset resolution to case 0
            dropdown.value = 0;
        }
        // Let confirm button disappear
        Confirm.SetActive(false);
        
    }
     
    void ConfirmClicked(TMP_Dropdown dropdown)
    {
        // Resolution selection is confirmed.
        resconf = true;
        // Let confirm button disappear after clicked
        Confirm.SetActive(false);
    }
}
