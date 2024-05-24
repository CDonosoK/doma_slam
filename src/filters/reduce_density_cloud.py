#! /usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

class ReduceDencity:
    def __init__(self):
        rospy.init_node('reduce_noise_filter')

        self.point_cloud_data = None
        self.max_distance = 20
        self.voxel_grid = 3

        self.point_cloud_subscriber = rospy.Subscriber('/unilidar/cloud', PointCloud2, self.point_cloud_callback)
        self.point_cloud_filtered_publisher = rospy.Publisher('/sensor/cloud/points_filtered', PointCloud2, queue_size=1)

        rospy.on_shutdown(self.shutdownhook)

    def shutdownhook(self):
        # Just to handle shutdown more gracefully
        rospy.loginfo("Shutting down downsample filter.")

    def point_cloud_callback(self, msg):
        self.point_cloud_data = np.array(list(pc2.read_points(msg)))
        self.point_cloud_data = self.point_cloud_data[::self.voxel_grid]
        self.point_cloud_data = self.point_cloud_data[
            np.linalg.norm(
                self.point_cloud_data[:,:3], axis=1
            ) < self.max_distance
        ]

        self.point_cloud_data = pc2.create_cloud_xyz32(
            msg.header,
            self.point_cloud_data[:,:3]
        )

        self.point_cloud_filtered_publisher.publish(self.point_cloud_data)


if __name__ == '__main__':
    ReduceDencity()
    rospy.spin()
