#  Copyright 2020 Unity Technologies
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

import rclpy
import socket
import re

from .communication import RosReceiver
from .client import ClientThread


class UnityService(RosReceiver):
    """
    Class to register a ROS service that's implemented in Unity.
    """

    def __init__(self, topic, service_class, tcp_server, queue_size=10):
        """

        Args:
            topic:         Topic name to publish messages to
            service_class: The message class in catkin workspace
            queue_size:    Max number of entries to maintain in an outgoing queue
        """
        strippedTopic = re.sub("[^A-Za-z0-9_]+", "", topic)
        node_name = f"{strippedTopic}_service"
        RosReceiver.__init__(self, node_name)

        self.topic = topic
        self.node_name = node_name
        self.service_class = service_class
        self.tcp_server = tcp_server
        self.queue_size = queue_size

        self.service = self.create_service(self.service_class, self.topic, self.send)

    def send(self, request, response):
        """
        Connect to TCP endpoint on client, pass along message and get reply
        Args:
            data: message data to send outside of ROS network

        Returns:
            The response message
        """
        return self.tcp_server.send_unity_service(self.topic, self.service_class, request)

    def unregister(self):
        """

        Returns:

        """
        self.destroy_node()
