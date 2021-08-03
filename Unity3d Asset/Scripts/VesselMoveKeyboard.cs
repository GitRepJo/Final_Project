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
using TMPro;

public class VesselMoveKeyboard : MonoBehaviour
{

    // Update is called once per frame
    void Update() 
    {

            //x translation ----------------------------------

            // Get the forward and backward motion, W and S are used, they seem to be the custom keys
            float zDirection = -Input.GetAxis("Vertical");

            // Lower speed by factor & change direction by 
            zDirection = - zDirection / 2;

            // Only the x value is used because other axis are not required for the use case
            Vector3 moveDirection = new Vector3(0.0f, 0.0f, zDirection);

            // transoform the position of the object according to the keyboard values
            transform.Translate(moveDirection);

            // z rotation ----------------------------------

            // Get the right and left motion, A and D are the custom keys here
            float yRotation = Input.GetAxis("Horizontal");

            // Lower rotation by factor
            yRotation = yRotation / 1;

            // Only z value is used and written to vector
            Vector3 rotateDirection = new Vector3(0.0f, yRotation, 0.0f);

            transform.Rotate(rotateDirection);
  
    }
   

}
