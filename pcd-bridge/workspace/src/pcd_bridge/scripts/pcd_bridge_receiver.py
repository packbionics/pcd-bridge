#!/usr/bin/env python
import rclpy
from sensor_msgs.msg import PointCloud2

import paho.mqtt.client as mqtt

import json

mqttc = None
global_msg = None

class PCDSerializable:
    def __init__(self, pc2):
        self.data = None
        if pc2 is not None:
            self.data = pc2.data.hex()

    def from_json_string(json_string):
        json_data = json.loads(json_string)
        pcd_serial = PCDSerializable(None)
        pcd_serial.data = json_data['data']

        return pcd_serial
    
    def to_pcd2(self):
        pcd2 = PointCloud2()
        data = bytes.fromhex(self.data)


# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, reason_code, properties):
    

    # print(f"Connected with result code {reason_code}")
    rospy.loginfo(str(reason_code))
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("bridge/pointcloud")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    global global_msg
    global_msg = msg
    print('I heard data with length: %s' % len(global_msg.payload))


# def callback(data: PointCloud2):
#     if global_msg is not None:
#         rospy.loginfo('I heard data with length: %s', len(global_msg))
    
    
def listener():
    global mqttc

    mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    mqttc.on_connect = on_connect
    mqttc.on_message = on_message

    mqttc.connect("localhost", 1883, 60)

    mqttc.loop_forever()

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('pcd_bridge', anonymous=True)

    rospy.Publisher("points", PointCloud2, callback)

    # # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()

    # mqttc.loop_stop()

if __name__ == '__main__':
    listener()
