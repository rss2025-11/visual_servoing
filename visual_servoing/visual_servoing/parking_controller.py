#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np

from vs_msgs.msg import ConeLocation, ParkingError
from ackermann_msgs.msg import AckermannDriveStamped

class ParkingController(Node):
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
    def __init__(self):
        super().__init__("parking_controller")

        self.declare_parameter("drive_topic", "/drive")
        DRIVE_TOPIC = self.get_parameter("drive_topic").value # set in launch file; different for simulator vs racecar

        self.drive_pub = self.create_publisher(AckermannDriveStamped, DRIVE_TOPIC, 10)
        self.error_pub = self.create_publisher(ParkingError, "/parking_error", 10)

        self.create_subscription(ConeLocation, "/relative_cone", 
            self.relative_cone_callback, 1)

        self.parking_distance = .75 # meters; try playing with this number!
        self.relative_x = 0
        self.relative_y = 0

        self.get_logger().info("Parking Controller Initialized")

        self.CAR_LENGTH = 0.5 # TODO: CHANGE BASED ON MEASUREMENT
        self.ANGLE_TOL = 0.1 # about 5 degrees
        self.DRIVE_SPEED = 0.7
    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        drive_cmd = AckermannDriveStamped()

        #################################
        # Use pure pursuit to park in front of the cone
        # Use module 3 to locate the position of the cone
        drive_cmd.header.stamp = self.get_clock().now().to_msg()
        # TODO: THIS DOES NOT HANDLE THE CASE WHERE RELATIVE_Y IS VERY SMALL
        angle_to_goal = np.arctan(self.relative_y/self.relative_x)
        distance = (self.relative_x**2 + self.relative_y**2)**(1/2)
        # if we are already too close, stop all together
        
        # compute lookahead distance
        look_ahead = np.sqrt(self.relative_x**2 + self.relative_y**2)
        look_ahead = min(2.0, look_ahead)

        angle = np.arctan(2*self.CAR_LENGTH*np.sin(angle_to_goal)/look_ahead)

        if (distance <= self.parking_distance):
            if abs(angle_to_goal) < self.ANGLE_TOL:
                drive_cmd.drive.speed = 0.0
                self.drive_pub.publish(drive_cmd)
                return
            else: # If you are too close to the cone but not aligned
                drive_cmd.drive.speed = -self.DRIVE_SPEED
                drive_cmd.drive.steering_angle = -angle


        else:
            if look_ahead == 2.0:
                self.get_logger().info(f"Lookahead is reduced to 2 | angle = {angle}")
            # self.get_logger().info("finished computing")
            drive_cmd.drive.steering_angle = angle
            drive_cmd.drive.speed = self.DRIVE_SPEED # TODO: MAKE A PARAMETER
        # YOUR CODE HERE
        # Use relative position and your control law to set drive_cmd

        #################################

        self.drive_pub.publish(drive_cmd)
        self.error_publisher()

    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = ParkingError()

        #################################

        # YOUR CODE HERE
        # Populate error_msg with relative_x, relative_y, sqrt(x^2+y^2)

        #################################
        
        self.error_pub.publish(error_msg)

def main(args=None):
    rclpy.init(args=args)
    pc = ParkingController()
    rclpy.spin(pc)
    rclpy.shutdown()

if __name__ == '__main__':
    main()