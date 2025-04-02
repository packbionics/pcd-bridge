import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from sensor_msgs.msg import PointCloud2, PointField

import paho.mqtt.client as mqtt

import json

from pcd_serializable import PCDSerializable



node = None
publisher = None 

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, reason_code, properties):
    global node
    global publisher

    print(f"Connected with result code {reason_code}")
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("bridge/pointcloud")

    node = Node('pcd_bridge_receiver')
    publisher = node.create_publisher(PointCloud2, 'points', qos_profile_system_default)    

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    print('data from: ', msg.topic)
    
    serial_pcd = PCDSerializable.from_json_string(msg.payload)
    pcd2 = serial_pcd.to_pcd2()
    publisher.publish(pcd2)


def main(args=None):
    mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    mqttc.on_connect = on_connect
    mqttc.on_message = on_message

    mqttc.connect("localhost", 1883, 60)

    rclpy.init(args=args)

    # Blocking call that processes network traffic, dispatches callbacks and
    # handles reconnecting.
    # Other loop*() functions are available that give a threaded interface and a
    # manual interface.
    mqttc.loop_forever()


if __name__ == '__main__':
    main()
