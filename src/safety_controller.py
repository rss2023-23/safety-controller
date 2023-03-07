#!/usr/bin/env python

import numpy as np
import sensor_msgs.point_cloud2 as pc2
import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan, PointCloud2
from ackermann_msgs.msg import AckermannDriveStamped
import laser_geometry.laser_geometry as lg
import math
from visualization_tools import *


class SafetyController:
    # ROS Parameters
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic", "/scan")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")

    LEFT_SCAN_STARTING_INDEX = 45
    LEFT_SCAN_ENDING_INDEX = 100
    RIGHT_SCAN_STARTING_INDEX = 0
    RIGHT_SCAN_ENDING_INDEX = 55
    MAX_WALL_DISTANCE = 3
    VELOCITY = rospy.get_param("wall_follower/velocity", 1)
    DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance", 1)



    def __init__(self):
        # Subscribe to LIDAR Sensor
        rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.on_lidar_scan)

        # Publish Car Actions
        self.car_publisher = rospy.Publisher(
            self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)

        # Handle Laser Geometry Projection
        self.laser_projector = lg.LaserProjection()
        self.laser_projection_publisher = rospy.Publisher(
            "laser_projection", PointCloud2, queue_size=1)

        self.wall_line_publisher = rospy.Publisher(
            "wall_line", Marker, queue_size=1)

        self.last_control = 0

    def on_lidar_scan(self, lidar_data):
        """
        lidar_data:
        Number Samples: 100
        Min Angle: -2.355
        Angle Increment: 0.047575756
        """
        # Get lidar data of followed wall
        # wall_data = self.make_wall_data(lidar_data) # now we want smallest value (20cm)
        smallest_value = np.min(self.wall_data(lidar_data))

        if smallest_value <= 0.2:
            self.adjust_drive()

    def wall_data(self, lidar_data):
        """
        Gets smallest value from data

        """
        wall_data = lidar_data[20:81]
        """
        This represents the "forward facing" ranges, so it won't stop when something comes up from behind

        """ 

        return wall_data

    def adjust_drive(self):
        car_action_stamped = AckermannDriveStamped()

        # Make header
        car_action_stamped.header.stamp = rospy.Time.now()
        car_action_stamped.header.frame_id = "world"

        # Make command
        car_action = car_action_stamped.drive
        car_action.steering_angle = 0
        car_action.steering_angle_velocity = 0
        car_action.speed = -0.5
        car_action.acceleration = 0
        car_action.jerk = 0

        # Publish command
        self.car_publisher.publish(car_action_stamped)



if __name__ == "__main__":
    rospy.init_node('safety_controller')
    wall_follower = SafetyController()
    rospy.spin()
