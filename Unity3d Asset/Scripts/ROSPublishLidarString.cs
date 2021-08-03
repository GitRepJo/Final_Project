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
    public class ROSPublishLidarString : UnityPublisher<MessageTypes.Std.String>
    {
        public string messageData;

        private MessageTypes.Std.String message;

        protected override void Start()
        {
            base.Start();
            // Write the Ascii literals to the string
            WriteMessage();
            // Initialize the data that is to be send with the string that contains this data
            InitializeMessage();
        }

        private void InitializeMessage()
        {
           message = new MessageTypes.Std.String
            {
                data = messageData
            };
        }
        private void WriteMessage()
        {
            // This string is read by a python interpreter, mark newlines with pythons </n>
            messageData = "This is supposed to be followed by a pointlcloud that results out of a raycast. /r"
            + "386.703 71.2376 365.0522 /r"
            + "386.7031 71.92661 365.0522 /r";
                    }
        private void Update()
        {
            message.data = messageData;
            Publish(message);
        }
    }
}
