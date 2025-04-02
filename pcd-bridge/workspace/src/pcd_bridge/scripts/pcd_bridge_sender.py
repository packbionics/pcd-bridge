#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2, PointField

import paho.mqtt.client as mqtt

import json

from pcd_serializable import PCDSerializable

mqttc = None



# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, reason_code, properties):
    

    # print(f"Connected with result code {reason_code}")
    rospy.loginfo(str(reason_code))
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
        mqttc.publish('bridge/pointcloud', data_json_string)
    
def listener():
    global mqttc

    mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    mqttc.on_connect = on_connect
    mqttc.on_message = on_message

    mqttc.connect("localhost", 1883, 60)

    mqttc.loop_start()

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('pcd_bridge', anonymous=True)

    rospy.Subscriber("points", PointCloud2, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    mqttc.loop_stop()

if __name__ == '__main__':
    listener()
