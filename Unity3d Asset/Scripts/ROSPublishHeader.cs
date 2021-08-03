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
namespace RosSharp.RosBridgeClient
{
    public class ROSPublishHeader : UnityPublisher<MessageTypes.GncInterfaces.HeaderTest>
    /// <summary> This class publishes a Header message to test support in ROS2 </summary>
    
    {


        private MessageTypes.GncInterfaces.HeaderTest message  = new MessageTypes.GncInterfaces.HeaderTest();

        
        protected override void Start()
        {
           
            base.Start();
            
        }


        private void SetMessage()
        {
            message.header.frame_id= "Headertest";
            message.header.Update();
            
        }

        
        private void Update()
        {
       
            
            SetMessage();
            Publish(message);
            
        }
        
    }
}
