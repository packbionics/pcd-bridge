import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from sensor_msgs.msg import PointCloud2, PointField

import paho.mqtt.client as mqtt

import json


class PCDSerializable:
    def __init__(self, pc2):
        self.height = 0
        self.width = 0

        self.is_bigendian = False
        self.point_step = 0
        self.row_step = 0

        self.fields = []

        self.data = None
        if pc2 is not None:
            self.height = pc2.height
            self.width = pc2.width

            self.fields = []
            for field in pc2.fields:
                field_data = {
                    "name": field.name,
                    "offset": field.offset,
                    "datatype": field.datatype,
                    "count": field.count
                }
                self.fields.append(field_data)

            self.is_bigendian = pc2.is_bigendian
            self.point_step = pc2.point_step
            self.row_step = pc2.row_step

            self.data = pc2.data.hex()

    def from_json_string(json_string):
        json_data = json.loads(json_string)
        pcd_serial = PCDSerializable(None)

        pcd_serial.height = json_data['height']
        pcd_serial.width = json_data['width']

        pcd_serial.fields = json_data['fields']

        pcd_serial.is_bigendian = json_data['is_bigendian']
        pcd_serial.point_step = json_data['point_step']
        pcd_serial.row_step = json_data['row_step']

        pcd_serial.data = json_data['data']

        return pcd_serial
    
    def to_pcd2(self):
        pcd2 = PointCloud2()

        pcd2.header.frame_id = 'map'

        pcd2.height = self.height
        pcd2.width = self.width

        for field in self.fields:
            point_field2 = PointField()

            point_field2.name = field['name']
            point_field2.offset = field['offset']
            point_field2.datatype = field['datatype']
            point_field2.count = field['count']

            pcd2.fields.append(point_field2)

        pcd2.is_bigendian = self.is_bigendian
        pcd2.point_step = self.point_step
        pcd2.row_step = self.row_step

        pcd2.data = bytes.fromhex(self.data)

        return pcd2


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
