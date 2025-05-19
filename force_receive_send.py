#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist
from sensor_msgs import point_cloud2
import numpy as np
import paho.mqtt.client as mqtt
import time

# Define the broker address and port
BROKER_ADDRESS = "172.22.247.120"
BROKER_PORT = 1883

class ForceManager:
    def __init__(self):
        rospy.init_node('force_manager', anonymous=True)

        self.rate = rospy.Rate(0.1)  # 1 Hz (1 message per second)

        # Publisher
        self.force_detect_pub = rospy.Publisher('/force_detect', , queue_size=10)

        # Create an MQTT client instance
        self.client = mqtt.Client()
        
        # Attach callback functions
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        self.receive_message()
        

    ##### MQTT client #####
    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print("Connected to Mosquitto broker!")
            client.subscribe("robot/arrival")
            print("Waiting for message ....")
        else:
            print(f"Failed to connect, return code {rc}")

    def on_message(self, client, userdata, msg):
        print(f"Message received: Topic = {msg.topic}, Payload = {msg.payload.decode()}")
        self.object = msg.payload.decode()
        print("Received object name: ", self.object)

        # Disconnect client after receiving message
        self.client.loop_stop()

    def receive_message(self):
        print(f"Connecting to broker {BROKER_ADDRESS}:{BROKER_PORT}...")
        self.client.connect(BROKER_ADDRESS, BROKER_PORT, 60)

        # Start loop in a background thread
        self.client.loop_start()
        print("Listening for messages...")

        # Wait for a message to be received
        while self.object is None:
            time.sleep(0.1)

        print("Message received. Exiting receive_message.")

    def send_message(self):
        print(f"Connecting to broker {BROKER_ADDRESS}:{BROKER_PORT}...")
        self.client.connect(BROKER_ADDRESS, BROKER_PORT, 60)

        # Start loop in a background thread
        self.client.loop_start()

        # Publish a message
        print("Publishing message...")
        self.client.publish("robot/arrival", "Grasp done")

        # Wait for the message to be sent
        time.sleep(1)

        # Stop the loop and disconnect
        self.client.loop_stop()
        self.client.disconnect()
        print("Message sent and client disconnected.")



    def run(self):
        # rate = rospy.Rate(1)  # 1 Hz (1 message per second)
        # while not rospy.is_shutdown():

        self.receive_message()
        
        rospy.spin()

if __name__ == "__main__":
    try:
        controller = ForceManager()
        controller.run()
        # rospy.spin()
    except rospy.ROSInterruptException:
        pass
