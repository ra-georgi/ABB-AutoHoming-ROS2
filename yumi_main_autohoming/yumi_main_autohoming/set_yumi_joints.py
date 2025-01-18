import rclpy
from rclpy.node import Node
from moveit_msgs.msg import RobotTrajectory
import numpy as np
import socket
import struct


class TrajectorySubscriber(Node):
    def __init__(self):
        super().__init__('trajectory_subscriber')
        self.subscription = self.create_subscription(
            RobotTrajectory,
            'robot_trajectory',
            self.trajectory_callback,
            10
        )
        self.get_logger().info('Trajectory subscriber node initialized.')
        self.joint_positions_L = None  # To store the joint positions
        self.joint_positions_R = None  # To store the joint positions
        self.joint_positions = None

    def trajectory_callback(self, msg):
        # Extract joint positions from the trajectory message
        joint_positions_list = [
            list(point.positions) for point in msg.joint_trajectory.points
        ]
        # Convert to a numpy array
        self.joint_positions = np.degrees(np.array(joint_positions_list))

        self.get_logger().info(f"Received trajectory with {len(joint_positions_list)} points.")
        self.get_logger().info(f"Joint positions saved as numpy array with shape {self.joint_positions.shape}.")
        
        # Stop spinning once the message is processed
        # rclpy.shutdown()

    def send_joint_values(self, port):
        # Create a socket object
        s = socket.socket()
        self.get_logger().info(f"Socket successfully created")

        # Connect to the server
        #Stanza IP
        s.connect(('10.103.141.81', port))      
        self.get_logger().info(f"Connected to server on port {port}")

        if port == 1025:
            self.get_logger().info(f"Left arm AH in progress")
            # Iterate through joint angles in the numpy array
            for joint_set in self.joint_positions_L:
                # Encode the joint values into 7 floats (28 bytes)
                data = struct.pack('<7f', *joint_set)

                # Send the encoded joint values
                s.send(data)
                self.get_logger().info(f"Sent Joint Values: {joint_set}")

                # Optionally, wait for acknowledgment from the server
                response = s.recv(1024).decode()
                self.get_logger().info(f"Server Response: {response}")
        else:
            # Iterate through joint angles in the numpy array
            for joint_set in self.joint_positions_R:
                # Encode the joint values into 7 floats (28 bytes)
                data = struct.pack('<7f', *joint_set)

                # Send the encoded joint values
                s.send(data)
                self.get_logger().info(f"Sent Joint Values: {joint_set}")

                # Optionally, wait for acknowledgment from the server
                response = s.recv(1024).decode()
                self.get_logger().info(f"Server Response: {response}")            

        # Close the socket connection
        s.close()
        print("Connection closed.")        

def main(args=None):
    rclpy.init(args=args)
    node = TrajectorySubscriber()

    while node.joint_positions is None:
        rclpy.spin_once(node)
    node.joint_positions_L = node.joint_positions
    node.joint_positions = None
    while node.joint_positions is None:
        rclpy.spin_once(node)
    node.joint_positions_R = node.joint_positions
    node.send_joint_values(1025)
    node.send_joint_values(1026)    

    # # Save the joint positions to a file after the node stops spinning
    # if node.joint_positions is not None:
    #     np.save("joint_positions.npy", node.joint_positions)
    #     node.get_logger().info("Joint positions saved to 'joint_positions.npy'.")

    rclpy.shutdown()

if __name__ == '__main__':
    main()
