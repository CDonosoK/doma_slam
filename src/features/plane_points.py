#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

def point_cloud_callback(msg):
    # Convert PointCloud2 message to numpy array
    points = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)

    # Convert points to a NumPy array
    points = np.array(list(points))

    # Ground segmentation using RANSAC
    num_points = points.shape[0]
    num_iterations = 20
    threshold_distance = 0.02
    best_inliers = []

    for _ in range(num_iterations):
        # Randomly sample 3 points
        indices = np.random.choice(num_points, 3, replace=False)
        sample_points = points[indices]

        # Fit a plane to the sampled points
        v1 = sample_points[1] - sample_points[0]
        v2 = sample_points[2] - sample_points[0]
        normal = np.cross(v1, v2)
        normal /= np.linalg.norm(normal)

        # Compute distances from all points to the plane
        distances = np.abs(np.dot(points - sample_points[0], normal))

        # Find inliers based on distance threshold
        inlier_indices = np.where(distances < threshold_distance)[0]

        # Update best inliers if current set is larger
        if len(inlier_indices) > len(best_inliers):
            best_inliers = inlier_indices

    # Extract ground points
    ground_points = points[best_inliers]

    # Create a new PointCloud2 message
    header = msg.header
    header.frame_id = 'unilidar_lidar'  # Replace 'base_link' with your desired frame ID
    ground_msg = point_cloud2.create_cloud_xyz32(header, ground_points)

    # Publish the ground points to a new topic
    pub = rospy.Publisher('/ground_points', PointCloud2, queue_size=10)
    pub.publish(ground_msg)


def main():
    rospy.init_node('ground_segmentation_node')
    rospy.Subscriber('/sensor/cloud/points_filtered', PointCloud2, point_cloud_callback)
    rospy.spin()

if __name__ == '__main__':
    main()