#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField

from rclpy.qos import qos_profile_system_default


import paho.mqtt.client as mqtt

import json

from pcd_serializable import PCDSerializable

mqttc = None

node = None
sub = None


# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, reason_code, properties):
    

    # print(f"Connected with result code {reason_code}")
    node.get_logger().info(str(reason_code))
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    # client.subscribe("$SYS/#")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    pass
    # print(msg.topic+" "+str(msg.payload))

def callback(data: PointCloud2):
    # rospy.loginfo('I heard data with length: %s', len(data.data))

    # Transfer point cloud data to a serializable format
    data_serializable = PCDSerializable(data)

    # Create a JSON string for data transfer
    data_json_string = json.dumps(data_serializable.__dict__)

    # Publish the data to a bridge topic
    if mqttc is not None:
        mqttc.publish('bridge/pointcloud2', data_json_string)
    
def listener(args=None):
    global mqttc
    global node
    global sub

    rclpy.init(args=args)

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    node = Node('pcd_bridge_connector')


    mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    mqttc.on_connect = on_connect
    mqttc.on_message = on_message

    mqttc.connect("localhost", 1883, 60)

    mqttc.loop_start()

    sub = node.create_subscription(PointCloud2, "camera/points", callback, qos_profile_system_default)

    # spin() simply keeps python from exiting until this node is stopped
    rclpy.spin(node)

    mqttc.loop_stop()

if __name__ == '__main__':
    listener()
