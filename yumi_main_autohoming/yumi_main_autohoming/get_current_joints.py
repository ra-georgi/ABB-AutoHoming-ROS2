#!/usr/bin/env python3
import socket
import struct
import yaml
import math

import rclpy
from rclpy.node import Node

import os
from ament_index_python.packages import get_package_share_directory

class SocketNode(Node):
    def __init__(self):
        super().__init__('socket_node')
        # Reserve a port on your computer
        port = 1025
        Joint_values_L = self.get_joint_values(port)

        port = 1026
        Joint_values_R =self.get_joint_values(port)
    
        self.update_yaml(Joint_values_L,Joint_values_R)        


    def update_yaml(self,Joint_values_L,Joint_values_R):
        # Path to the YAML file
        # file_path = "/home/reuben/moveit_ws/src/yumi_main_autohoming/urdf/initial_positions.yaml"
        description_dir = get_package_share_directory("yumi_main_autohoming")
        file_path=os.path.join(description_dir, "config", "initial_positions.yaml")        


        # Define the new float values for the joints
        updated_values = {
            "gripper_l_joint": 0.1,
            "gripper_r_joint": 0.2,
            "yumi_joint_1_l": math.radians(Joint_values_L[0]),
            "yumi_joint_1_r": math.radians(Joint_values_R[0]),
            "yumi_joint_2_l": math.radians(Joint_values_L[1]),
            "yumi_joint_2_r": math.radians(Joint_values_R[1]),
            "yumi_joint_3_l": math.radians(Joint_values_L[2]),
            "yumi_joint_3_r": math.radians(Joint_values_R[2]),
            "yumi_joint_4_l": math.radians(Joint_values_L[3]),
            "yumi_joint_4_r": math.radians(Joint_values_R[3]),
            "yumi_joint_5_l": math.radians(Joint_values_L[4]),
            "yumi_joint_5_r": math.radians(Joint_values_R[4]),
            "yumi_joint_6_l": math.radians(Joint_values_L[5]),
            "yumi_joint_6_r": math.radians(Joint_values_R[5]),
            "yumi_joint_7_l": math.radians(Joint_values_L[6]),
            "yumi_joint_7_r": math.radians(Joint_values_R[6]),
        }

        # Read, modify, and write the YAML file
        with open(file_path, "r") as file:
            data = yaml.safe_load(file)

        # Update the float values in the YAML content
        data["initial_positions"].update(updated_values)

        # Write the updated content back to the YAML file
        with open(file_path, "w") as file:
            yaml.dump(data, file, default_flow_style=False)

        print("Updated the YAML file with new float values.")  

    def get_joint_values(self,port):
        # Create a socket object    
        s = socket.socket()
        print("Socket successfully created")

        # Bind the socket to the port
        s.bind(('', port))
        print(f"Socket bound to port {port}")

        # Put the socket into listening mode
        s.listen(5)
        print("Socket is listening")

        Joint_values = [0.0] * 7  

        while True:
            # Establish connection with client
            c, addr = s.accept()
            print('Got connection from', addr)

            # Receive data from the client (maximum 1024 bytes at a time)
            data = c.recv(28)

            # Decode the raw bytes
            try:
                Joint_values = struct.unpack('<7f', data)  # Little-endian
                # print(f"Decoded (little-endian): {Joint_values[0]}")
            except struct.error as e:
                print(f"Error decoding float: {e}")

            # print(f"Received from client: {data}")

            # Optionally, send a response back to the client
            c.send('Message received'.encode())

            # Close the connection with the client
            c.close()

            # Uncomment this line to handle only one client and exit the loop
            break     

        return Joint_values

def main():
    rclpy.init()
    node = SocketNode()
    # rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
