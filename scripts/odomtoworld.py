#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped  # Import PoseStamped
import tf
import numpy as np

# Global variable to store the initial transformation
initial_transformation = None

# Publisher for the transformed pose
transformed_pose_pub = rospy.Publisher('transformed_pose', PoseStamped, queue_size=10)

def initial_transformation_callback(data):
    global initial_transformation
    position = data.pose.position
    orientation = data.pose.orientation
    rotation_matrix = tf.transformations.quaternion_matrix([orientation.x, orientation.y, orientation.z, orientation.w])

    # Create 4x4 transformation matrix
    initial_transformation = np.array([
        [rotation_matrix[0][0], rotation_matrix[0][1], rotation_matrix[0][2], position.x],
        [rotation_matrix[1][0], rotation_matrix[1][1], rotation_matrix[1][2], position.y],
        [rotation_matrix[2][0], rotation_matrix[2][1], rotation_matrix[2][2], position.z],
        [0, 0, 0, 1]
    ])

def pose_callback(data):
    global initial_transformation
    if initial_transformation is None:
        rospy.logwarn("Initial transformation not yet received")
        return

    # Check if there are any poses in the path
    if len(data.poses) > 0:
        # Access the most recent pose (last element in the array)
        pose = data.poses[-1].pose
        rospy.loginfo("Received pose: x={}, y={}, z={}".format(
            pose.position.x, pose.position.y, pose.position.z))

        # Convert quaternion to rotation matrix
        quaternion = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        rotation_matrix = tf.transformations.quaternion_matrix(quaternion)

        # Create 4x4 transformation matrix from the odom frame
        transformation_odom = np.array([
            [rotation_matrix[0][0], rotation_matrix[0][1], rotation_matrix[0][2], pose.position.x],
            [rotation_matrix[1][0], rotation_matrix[1][1], rotation_matrix[1][2], pose.position.y],
            [rotation_matrix[2][0], rotation_matrix[2][1], rotation_matrix[2][2], pose.position.z],
            [0, 0, 0, 1]
        ])

        # Apply the initial transformation
        transformed_pose_matrix = np.dot(initial_transformation, transformation_odom)

        # Extract the transformed position and orientation
        transformed_position = transformed_pose_matrix[:3, 3]
        transformed_orientation = tf.transformations.quaternion_from_matrix(transformed_pose_matrix)

        # Create a PoseStamped message
        transformed_pose_msg = PoseStamped()
        transformed_pose_msg.header.stamp = rospy.Time.now()
        transformed_pose_msg.header.frame_id = "world"
        transformed_pose_msg.pose.position.x = transformed_position[0]
        transformed_pose_msg.pose.position.y = transformed_position[1]
        transformed_pose_msg.pose.position.z = transformed_position[2]
        transformed_pose_msg.pose.orientation.x = transformed_orientation[0]
        transformed_pose_msg.pose.orientation.y = transformed_orientation[1]
        transformed_pose_msg.pose.orientation.z = transformed_orientation[2]
        transformed_pose_msg.pose.orientation.w = transformed_orientation[3]

        # Publish the transformed pose
        transformed_pose_pub.publish(transformed_pose_msg)

        rospy.loginfo("Transformed pose: x={}, y={}, z={}".format(
            transformed_pose_msg.pose.position.x,
            transformed_pose_msg.pose.position.y,
            transformed_pose_msg.pose.position.z))
    else:
        rospy.logwarn("No poses received in the path")

def listener():
    rospy.init_node('pose_transformer', anonymous=True)
    # rospy.Subscriber('/lio_sam/mapping/pose_odomTo_map', PoseWithCovarianceStamped, initial_transformation_callback)
    rospy.Subscriber('/lio_sam/mapping/pose_odomTo_map', PoseStamped, initial_transformation_callback)  # Corrected to PoseStamped
    rospy.Subscriber('/lio_sam/mapping/path', Path, pose_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
