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
    SCAN_TOPIC = rospy.get_param("safety_controller/scan_topic", "/scan")
    DRIVE_TOPIC = rospy.get_param("safety_controller/drive_topic", "/vesc/low_level/ackermann_cmd_mux/input/safety")

    # Tunable Parameters
    SCAN_STARTING_INDEX = rospy.get_param("safety_controller/scan_starting_index", 50)
    SCAN_ENDING_INDEX = rospy.get_param("safety_controller/scan_ending_index", 50)
    DANGER_THRESHOLD = rospy.get_param("safety_controller/danger_threshold", 0.2)

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

    def on_lidar_scan(self, lidar_data):
        """
        lidar_data:
        Number Samples: 100
        Min Angle: -2.355
        Angle Increment: 0.047575756
        """
        # Get lidar data of collision zone
        collision_zone_data = self.make_collision_zone_data(lidar_data)
    
        # Visualize collision zone points
        wall_projection = self.laser_projector.projectLaser(collision_zone_data)
        self.laser_projection_publisher.publish(wall_projection)

        # Get Collision Zone Data
        min = np.min(collision_zone_data)
        average = np.average(collision_zone_data)

        rospy.loginfo("Average: " + average + " Min: " + min)
        

        if average <= self.DANGER_THRESHOLD:
            self.stop_car()

    def make_collision_zone_data(self, lidar_data):
        """
        Mutates lidar_data to only contain the lidar data in the collision zone of the car

        Collision Zone: #TODO HERE
        """
        lidar_data.ranges = lidar_data.ranges[self.SCAN_STARTING_INDEX:self.SCAN_ENDING_INDEX]
        lidar_data.angle_min = lidar_data.angle_min + lidar_data.angle_increment * float(self.SCAN_STARTING_INDEX)
        lidar_data.angle_max = lidar_data.angle_min + lidar_data.angle_increment * float(self.SCAN_ENDING_INDEX-1)

        return lidar_data

    def stop_car(self):
        car_action_stamped = AckermannDriveStamped()

        # Make header
        car_action_stamped.header.stamp = rospy.Time.now()
        car_action_stamped.header.frame_id = "world"

        # Make command
        car_action = car_action_stamped.drive
        car_action.steering_angle = 0
        car_action.steering_angle_velocity = 0
        car_action.speed = -0.1
        car_action.acceleration = 0
        car_action.jerk = 0

        # Publish command
        self.car_publisher.publish(car_action_stamped)


if __name__ == "__main__":
    rospy.init_node('safety_controller')
    wall_follower = SafetyController()
    rospy.spin()
