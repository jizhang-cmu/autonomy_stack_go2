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
import time
import threading
import json

from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rclpy.serialization import serialize_message

from .client import ClientThread
from .thread_pauser import ThreadPauser

# queue module was renamed between python 2 and 3
try:
    from queue import Queue
    from queue import Empty
except:
    from Queue import Queue
    from Queue import Empty


class UnityTcpSender:
    """
    Sends messages to Unity.
    """

    def __init__(self, tcp_server):
        # super().__init__(f'UnityTcpSender')

        self.sender_id = 1
        self.time_between_halt_checks = 5
        self.tcp_server = tcp_server

        # Each sender thread has its own queue: this is always the queue for the currently active thread.
        self.queue = None
        self.queue_lock = threading.Lock()

        # variables needed for matching up unity service requests with responses
        self.next_srv_id = 1001
        self.srv_lock = threading.Lock()
        self.services_waiting = {}

    def send_unity_info(self, text):
        if self.queue is not None:
            command = SysCommand_Log()
            command.text = text
            serialized_bytes = ClientThread.serialize_command("__log", command)
            self.queue.put(serialized_bytes)

    def send_unity_warning(self, text):
        if self.queue is not None:
            command = SysCommand_Log()
            command.text = text
            serialized_bytes = ClientThread.serialize_command("__warn", command)
            self.queue.put(serialized_bytes)

    def send_unity_error(self, text):
        if self.queue is not None:
            command = SysCommand_Log()
            command.text = text
            serialized_bytes = ClientThread.serialize_command("__error", command)
            self.queue.put(serialized_bytes)

    def send_ros_service_response(self, srv_id, destination, response):
        if self.queue is not None:
            command = SysCommand_Service()
            command.srv_id = srv_id
            serialized_header = ClientThread.serialize_command("__response", command)
            serialized_message = ClientThread.serialize_message(destination, response)
            self.queue.put(b"".join([serialized_header, serialized_message]))

    def send_unity_message(self, topic, message):
        if self.queue is not None:
            serialized_message = ClientThread.serialize_message(topic, message)
            self.queue.put(serialized_message)

    def send_unity_service_request(self, topic, service_class, request):
        if self.queue is None:
            return None

        thread_pauser = ThreadPauser()
        with self.srv_lock:
            srv_id = self.next_srv_id
            self.next_srv_id += 1
            self.services_waiting[srv_id] = thread_pauser

        command = SysCommand_Service()
        command.srv_id = srv_id
        serialized_header = ClientThread.serialize_command("__request", command)
        serialized_message = ClientThread.serialize_message(topic, request)
        self.queue.put(b"".join([serialized_header, serialized_message]))

        # rospy starts a new thread for each service request,
        # so it won't break anything if we sleep now while waiting for the response
        thread_pauser.sleep_until_resumed()

        response = deserialize_message(thread_pauser.result, service_class.Response())
        return response

    def send_unity_service_response(self, srv_id, data):
        thread_pauser = None
        with self.srv_lock:
            thread_pauser = self.services_waiting[srv_id]
            del self.services_waiting[srv_id]

        thread_pauser.resume_with_result(data)

    def get_registered_topic(self, topic):
        if topic in self.tcp_server.publishers_table:
            return self.tcp_server.publishers_table[topic]
        elif topic in self.tcp_server.subscribers_table:
            return self.tcp_server.subscribers_table[topic]
        elif topic in self.tcp_server.ros_services_table:
            return self.tcp_server.ros_services_table[topic]
        elif topic in self.tcp_server.unity_services_table:
            return self.tcp_server.unity_services_table[topic]
        else:
            return None

    def send_topic_list(self):
        if self.queue is not None:
            topic_list = SysCommand_TopicsResponse()
            topics_and_types = self.tcp_server.get_topic_names_and_types()
            topic_list.topics = [item[0] for item in topics_and_types]
            for i in topics_and_types:
                node = self.get_registered_topic(i[0])
                if len(i[1]) > 1:
                    if node is not None:
                        self.tcp_server.get_logger().warning(
                            "Only one message type per topic is supported, but found multiple types for topic {}; maintaining {} as the subscribed type.".format(
                                i[0], self.parse_message_name(node.msg)
                            )
                        )
                topic_list.types = [
                    item[1][0].replace("/msg/", "/")
                    if (len(item[1]) <= 1)
                    else self.parse_message_name(node.msg)
                    for item in topics_and_types
                ]
            serialized_bytes = ClientThread.serialize_command("__topic_list", topic_list)
            self.queue.put(serialized_bytes)

    def start_sender(self, conn, halt_event):
        sender_thread = threading.Thread(
            target=self.sender_loop, args=(conn, self.sender_id, halt_event)
        )
        self.sender_id += 1

        # Exit the server thread when the main thread terminates
        sender_thread.daemon = True
        sender_thread.start()

    def sender_loop(self, conn, tid, halt_event):
        s = None
        local_queue = Queue()

        # send a handshake message to confirm the connection and version number
        handshake_metadata = SysCommand_Handshake_Metadata()
        handshake = SysCommand_Handshake(handshake_metadata)
        local_queue.put(ClientThread.serialize_command("__handshake", handshake))

        with self.queue_lock:
            self.queue = local_queue

        try:
            while not halt_event.is_set():
                try:
                    item = local_queue.get(timeout=self.time_between_halt_checks)
                except Empty:
                    # I'd like to just wait on the queue, but we also need to check occasionally for the connection being closed
                    # (otherwise the thread never terminates.)
                    continue

                # print("Sender {} sending an item".format(tid))

                try:
                    conn.sendall(item)
                except Exception as e:
                    self.tcp_server.logerr("Exception {}".format(e))
                    break
        finally:
            halt_event.set()
            with self.queue_lock:
                if self.queue is local_queue:
                    self.queue = None

    def parse_message_name(self, name):
        try:
            # Example input string: <class 'std_msgs.msg._string.Metaclass_String'>
            names = (str(type(name))).split(".")
            module_name = names[0][8:]
            class_name = names[-1].split("_")[-1][:-2]
            return "{}/{}".format(module_name, class_name)
        except (IndexError, AttributeError, ImportError) as e:
            self.tcp_server.logerr("Failed to resolve message name: {}".format(e))
            return None


class SysCommand_Log:
    def __init__(self):
        text = ""


class SysCommand_Service:
    def __init__(self):
        srv_id = 0


class SysCommand_TopicsResponse:
    def __init__(self):
        topics = []
        types = []


class SysCommand_Handshake:
    def __init__(self, metadata):
        self.version = "v0.7.0"
        self.metadata = json.dumps(metadata.__dict__)


class SysCommand_Handshake_Metadata:
    def __init__(self):
        self.protocol = "ROS2"
