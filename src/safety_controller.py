#!/usr/bin/env python2

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
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic", "/vesc/ackermann_cmd_mux/input/navigation")

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
        wall_data = self.make_wall_data(lidar_data)

        # Convert wall distances to coordinate points
        wall_projection = self.laser_projector.projectLaser(wall_data)
        self.laser_projection_publisher.publish(wall_projection)
        wall_points = pc2.read_points_list(wall_projection)

        # Estimate wall line from coordinate points
        slope, intercept = self.get_wall_line(lidar_data, wall_points)

        # Visualize wall line
        x = [-2, 2]
        y = [x[0]*slope + intercept, x[1]*slope + intercept]
        VisualizationTools.plot_line(
            x, y, self.wall_line_publisher, frame="/laser")

        # Estimate Distance from Wall
        distance = abs(intercept) / math.sqrt(1 + slope**2)
        print("Distance: ", distance)

        # Follow Wall
        # Positive error means steer away

        """
        error = -self.SIDE * (self.DESIRED_DISTANCE - distance)
        control = self.KP*error + self.KD * \
            self.VELOCITY*math.sin(self.last_control) # Substitute with slope cos(arctan(slope))
        self.last_control = control
        self.drive(control, self.VELOCITY)
        """

    def make_wall_data(self, lidar_data):
        """
        Mutates lidar_data to only contain the lidar data facing the followed wall

        Left Wall: > 20 degrees; > 0.349066 radians; Index : 57-100
        Right Wall: < -20 degrees; < -0.349066 radians; Index 0:43
        """
        wall_to_follow = self.side_to_direction[self.SIDE]

        if wall_to_follow == "left":
            lidar_data.ranges = lidar_data.ranges[self.LEFT_SCAN_STARTING_INDEX:self.LEFT_SCAN_ENDING_INDEX]
            lidar_data.angle_min = lidar_data.angle_min + \
                lidar_data.angle_increment * \
                float(self.LEFT_SCAN_STARTING_INDEX)
        else:
            lidar_data.ranges = lidar_data.ranges[self.RIGHT_SCAN_STARTING_INDEX:self.RIGHT_SCAN_ENDING_INDEX]
            lidar_data.angle_max = lidar_data.angle_min + \
                lidar_data.angle_increment * \
                float(self.RIGHT_SCAN_ENDING_INDEX-1)

        return lidar_data

    def get_wall_line(self, lidar_data, wall_points):
        ranges = lidar_data.ranges

        x_coords = [wall_points[i].x for i in range(
            len(wall_points)) if ranges[i] <= self.MAX_WALL_DISTANCE]
        y_coords = [wall_points[j].y for j in range(
            len(wall_points)) if ranges[j] <= self.MAX_WALL_DISTANCE]
        return np.polyfit(x_coords, y_coords, 1)

    def drive(self, steering_angle, speed, steering_angle_velocity=0, acceleration=0, jerk=0):
        car_action_stamped = AckermannDriveStamped()

        # Make header
        car_action_stamped.header.stamp = rospy.Time.now()
        car_action_stamped.header.frame_id = "world"

        # Make command
        car_action = car_action_stamped.drive
        car_action.steering_angle = steering_angle
        car_action.steering_angle_velocity = steering_angle_velocity
        car_action.speed = speed
        car_action.acceleration = acceleration
        car_action.jerk = jerk

        # Publish command
        self.car_publisher.publish(car_action_stamped)



if __name__ == "__main__":
    rospy.init_node('safety_controller')
    wall_follower = SafetyController()
    rospy.spin()
