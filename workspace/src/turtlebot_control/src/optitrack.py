#!/usr/bin/env python3

from geometry_msgs.msg import PoseWithCovarianceStamped
import json
import paho.mqtt.client as mqtt
import sys
import time
import rospy


class OptiTrack:
    def __init__(self, config):
        self.config = config
        self.client_id = config["server"]["client_id"]
        self.password = config["server"]["password"]
        self.username = config["server"]["username"]
        self.url_server = config["server"]["url"]
        self.tcp_port = config["server"]["tcp_port"]
        self.keepalive_client = config["server"]["keepalive_client"]
        self.bind_address_client = config["server"]["bind_address_client"]
        self.node: Node

    def start(self):
        rospy.loginfo(f"Starting with client_id: {self.client_id}")
        self.connect()

    def stop(self):
        rospy.loginfo(f"Stopping service with client_id: {self.client_id}")
        data = {"connection_type": "tcp", "client_id": self.client_id}
        topic = "all_clients/cobots-service-server/client_disconnected"
        self.publish(topic, json.dumps(data))
        self.client.disconnect()
        sys.exit(0)

    def connect(self):
        self.client = mqtt.Client(self.client_id)
        self.client.username_pw_set(self.username, self.password)
        self.client.enable_logger()

        self.client.on_connect = self.onConnect
        self.client.on_disconnect = self.onDisconnect
        self.client.on_message = self.onMessage
        self.client.on_log = self.onLog

        data = {"connection_type": "tcp", "client_id": self.client_id}
        topic = "all_clients/cobots-service-server/client_disconnected"
        self.client.will_set(topic, json.dumps(data), 0, False)

        while True:
            try:
                self.client.connect(
                    self.url_server,
                    self.tcp_port,
                    self.keepalive_client,
                    self.bind_address_client,
                )
                self.client.loop_start()
                break
            except Exception as e:
                rospy.loginfo(f"Tentative de connexion au serveur: {str(e)}")
                time.sleep(5)

    # The callback for when the client receives a CONNACK response from the server.
    def onConnect(self, client, userdata, flags, rc):
        msg = {
            0: "Connection successful.",
            1: "Connection refused - incorrect protocol version",
            2: "Connection refused - invalid client identifier",
            3: "Connection refused - server unavailable",
            4: "Connection refused - bad username or password",
            5: "Connection refused - not authorised",
        }
        rospy.loginfo(msg.get(rc, f"Code {rc} currently unused"))

        if rc != 0:
            self.client.reconnect()

        self.client.subscribe(self.client_id + "/#")
        self.client.subscribe("all_clients/cobots-service-client/#")

        data = {
            "connection_type": "tcp",
            "client_id": self.client_id,
            "commands": self.config["commands"],
            "styles": self.config["styles"],
        }
        topic = "all_clients/cobots-service-server/client_connected"
        self.publish(topic, json.dumps(data))

    def onDisconnect(self, client, userdata, rc):
        rospy.loginfo(f"Disconnected from server with code {rc}")
        self.client.loop_stop()
        self.connect()

    # The callback for when a PUBLISH message is received from the server.
    def onMessage(self, client, userdata, msg):
        array_topic = msg.topic.split("/")
        if len(array_topic) > 0:
            client_id = array_topic[0]
            if client_id != "$SYS":
                if client_id == self.client_id or client_id == "all_clients":
                    if len(array_topic) == 3:
                        service = array_topic[1]
                        if len(array_topic) > 2:
                            command = array_topic[2]
                            self.publishMessageToService(
                                service, command, str(msg.payload, "utf-8")
                            )
                        else:
                            rospy.loginfo(f"Topic action '{msg.topic}' does not exist")
                    elif len(array_topic) == 2:
                        topic_command = array_topic[1]
                        if topic_command in self.config["commands"]:
                            service = self.config["commands"][topic_command]["service"]
                            command = self.config["commands"][topic_command]["command"]
                            self.publishMessageToService(
                                service, command, str(msg.payload, "utf-8")
                            )
                        else:
                            rospy.loginfo(
                                f"Topic '{msg.topic}' has no command '{topic_command}'"
                            )
                    else:
                        rospy.loginfo(
                            f"Service or command '{msg.topic}' does not exist"
                        )
                else:
                    rospy.loginfo(
                        f"The client_id for topic '{msg.topic}' does not match the configuration '{self.client_id}'"
                    )
        else:
            rospy.loginfo(f"The client_id for topic '{msg.topic}' does not exist")

    def onLog(self, client, userdata, level, buf):
        # le call back n'est pas appelé. Problème similaire:
        # https://www.eclipse.org/forums/index.php/m/1849551/?srch=on_log+not+triggered#msg_1849551
        pass

    def publish(self, topic, data):
        self.client.publish(topic, data)

    # ----------------------------------------------------------------------------------------------------------------------------------------------------
    # Commandes du service
    # ----------------------------------------------------------------------------------------------------------------------------------------------------
    def serverIsStartedCommand(self, service, command, data):
        data = {
            "connection_type": "tcp",
            "client_id": self.config["server"]["client_id"],
        }
        topic = "all_clients/cobots-service-server/client_connected"
        self.publish(topic, json.dumps(data))

    def getPositionOptitrackReply(self, data):
        if data is str:
            rospy.loginfo(data)
        else:
            rospy.loginfo(json.dumps(data))

    # #----------------------------------------------------------------------------------------------------------------------------------------------------
    # # Récéption des commandes depuis le cobots-service-server
    # #----------------------------------------------------------------------------------------------------------------------------------------------------

    def publishMessageToService(self, service, command, data):
        data_json = json.loads(data)
        if service == "cobots-service-client":
            if command == "server_is_started":
                self.serverIsStartedCommand(service, command, data_json)
            else:
                rospy.loginfo(f"Cannot execute command '{command}': {data}")
                reply_data = {
                    "reply_request": data_json["reply_request"],
                    "status": "error",
                    "data": {
                        "error": "Le robot "
                        + self.client_id
                        + ", dans le service "
                        + service
                        + ", ne peut executer la commande "
                        + command
                        + ": "
                        + data
                    },
                }
                self.publish(data_json["reply_request"]["reply_topic"], reply_data)
        elif service == "turtlebot":
            if command == "position_event":
                self.node.send(data)
        else:
            rospy.loginfo(f"Could not send command '{command}' to service '{service}'")


class Node:
    def __init__(self):
        rospy.init_node("optitrack_relay_node")
        self.__publisher = rospy.Publisher(
            "/optitrack", PoseWithCovarianceStamped, queue_size=1
        )
        self.rate = rospy.Rate(10)
        config = json.load(open("/workspace/config.json"))
        self.optitrack = OptiTrack(config)
        self.optitrack.node = self
        self.optitrack.start()

    def send(self, data):
        data_json = json.loads(data)
        position = data_json["data"]["position"]
        orientation = data_json["data"]["orientation"]
        pose = PoseWithCovarianceStamped()
        pose.pose.pose.position.x = position["x"]
        pose.pose.pose.position.y = position["y"]
        pose.pose.pose.position.z = position["z"]
        pose.pose.pose.orientation.x = orientation["x"]
        pose.pose.pose.orientation.y = orientation["y"]
        pose.pose.pose.orientation.z = orientation["z"]
        pose.pose.pose.orientation.w = orientation["w"]
        pose.header.frame_id = 'scan_base'
        time = rospy.Time.now()
        pose.header.stamp.secs = time.secs
        pose.header.stamp.nsecs = time.nsecs
        self.__publisher.publish(pose)


    def loop(self):
        data = {
            "reply_request": {
                "reply_topic": "turtlebot/position_event",
                "uuid": 1,
                "service": "",
            },
            "data": {"timeout": 1000, "client_id": "turtlebot"},
        }
        while not rospy.is_shutdown():
            self.optitrack.publish("optitrack/get-position-optitrack", json.dumps(data))
            self.rate.sleep()


if __name__ == "__main__":
    node = None
    try:
        node = Node()
        node.loop()
    except rospy.ROSInterruptException:
        if node is not None:
            node.optitrack.stop()
