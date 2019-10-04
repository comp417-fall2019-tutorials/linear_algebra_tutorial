#!/usr/bin/env python

import cv2
import math
import os
import numpy as np
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import numpy as np

K = np.array([[119.17577090035485, 0.0, 100.5], [0.0, 119.17577090035485, 100.5], [0.0, 0.0, 1.0]])
K_inv = np.linalg.inv(K)


class CameraView:
    GAZEBO_PLANE_SIZE = (30.0, 30.0)

    def __init__(self, reef_plane_image):
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.on_odom)
        self.image_true_subscriber = rospy.Subscriber('/camera/image_true', Image, self.on_image_true)
        self.image_generated_pub = rospy.Publisher('/camera/generated', Image, queue_size=1)
        self.image_difference_pub = rospy.Publisher('/camera/difference', Image, queue_size=1)
        self.reef_plane_image = reef_plane_image
        self.cv_bridge = CvBridge()
        self.pose = (0, 0, 0)

    def get_seabed_to_duck_camera_transform(self, x_duck, y_duck, theta_duck):

        CAMERA_X_OFFSET = 0.2
        SEABED_Z_DEPTH = 2
        half_plane_size = self.GAZEBO_PLANE_SIZE[0]/2

        # TODO: Complete matrix transformations
        # -----------------------------------------------
        T_camera_offset = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ])
        # Can use np.cos(rad), np.sin(rad)
        R_duck_rot = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ])
        T_seabed_to_duck = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ])
        # END TODO
        # -----------------------------------------------
        return np.matmul(T_seabed_to_duck, np.matmul(R_duck_rot, T_camera_offset))

    def on_odom(self, odom):
        pos = odom.pose.pose.position
        orientation_q = odom.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.pose = pos.x, pos.y, yaw

    def get_sample_points(self, img_true):
        n_rows = img_true.shape[1]
        n_cols = img_true.shape[0]
        sample_points = np.meshgrid(np.arange(0, n_rows), np.arange(0, n_cols))
        samples_x = sample_points[0].flatten()
        samples_y = sample_points[1].flatten()
        sample_points = np.transpose(np.concatenate((np.expand_dims(samples_x, 1), np.expand_dims(samples_y, 1)), axis=1))
        return np.concatenate((sample_points, np.expand_dims(np.ones(sample_points.shape[1]), 0)), axis=0)

    def on_image_true(self, img_input):
        F = self.get_seabed_to_duck_camera_transform(self.pose[0], self.pose[1], self.pose[2])
        img_true = self.cv_bridge.imgmsg_to_cv2(img_input, "bgr8")
        img_generated = np.zeros_like(img_true)

        sample_points = self.get_sample_points(img_true)
        cam_coords = np.matmul(K_inv, sample_points)

        # Add back in z depth
        cam_coords = cam_coords*2.0
        cam_coords[2, :] = cam_coords[2, :] * -1
        cam_coords = np.concatenate((cam_coords, np.expand_dims(np.ones(cam_coords.shape[1]), 0)), axis=0)

        # Pixel to gazebo scale matrix
        S = np.eye(4)
        S[0,0] = self.reef_plane_image.shape[1]/self.GAZEBO_PLANE_SIZE[1]
        S[1,1] = self.reef_plane_image.shape[0]/self.GAZEBO_PLANE_SIZE[0]

        # Camera frame rotation matrix
        R_camera_frame = np.array([
            [np.cos(-np.pi / 2), -np.sin(-np.pi / 2), 0, 0],
            [np.sin(-np.pi / 2), np.cos(-np.pi / 2), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ])

        reef_plane_coords = np.matmul(S, (np.matmul(F, np.matmul(R_camera_frame, cam_coords))))

        for reef_plane_coord, img_coord in zip(np.transpose(reef_plane_coords), np.transpose(sample_points)):
            r = self.reef_plane_image.shape[0]-int(reef_plane_coord[1])-1
            c = int(reef_plane_coord[0])
            reef_plane_px = self.reef_plane_image[r, c]
            img_generated[img_generated.shape[0]-int(img_coord[1])-1, int(img_coord[0]), :] = reef_plane_px

        img_difference = np.zeros_like(img_true)
        img_difference[:, :, :] = np.abs(img_true.astype(int) - img_generated.astype(int))
        self.image_generated_pub.publish(self.cv_bridge.cv2_to_imgmsg(img_generated, "bgr8"))
        self.image_difference_pub.publish(self.cv_bridge.cv2_to_imgmsg(img_difference, "bgr8"))


if __name__ == '__main__':

    rospy.init_node('camera_view_node', anonymous=True)
    rospy.loginfo("Camera View Node Starting")

    pkg_dir = os.popen('rospack find linear_algebra_tutorial').read().rstrip()
    reef_plane_path = os.path.join(pkg_dir, 'materials', 'reef_plane.png')
    reef_plane_image = cv2.imread(reef_plane_path)

    camera_view_worker = CameraView(reef_plane_image)

    while not rospy.is_shutdown():
        rospy.spin()
