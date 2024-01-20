#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
## from tf.transformations import euler_from_quaternion
import numpy as np
import cv2

# Load camera matrix and distortion coefficients from the saved file
calibration_data = np.load('/home/janhavi/AiPhile_cal/calib_data/MultiMatrix.npz')
camera_matrix = calibration_data['camMatrix']
dist_coeffs = calibration_data['distCoef']
r_vectors = calibration_data['rVector']
t_vectors = calibration_data['tVector']

def aruco_pose_callback(msg):
    # Extract translation component (X, Y, Z)
    print("yes")
    translation = msg.pose.position

    # Calculate distance using the loaded camera matrix and distortion coefficients
    '''image_points = np.array([[translation.x, translation.y]])
    marker_size = 0.1
    object_points = np.array([[0, 0, 0],
                          [marker_size, 0, 0],
                          [0, marker_size, 0],
                          [marker_size, marker_size, 0]], dtype=np.float32)
    # Assuming the marker size in meters

    _, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)'''

    ##distance = tvec[2][0]  # Z component of the translation vector
    ##print("distance: ",distance)
    print("x: ",translation.x)
    print("y: ",translation.y)
    print("z: ",translation.z)
    ##rospy.loginfo("Distance to Aruco marker: {:.2f} meters".format(distance))

def main():
    rospy.init_node('aruco_distance_calculator', anonymous=True)
    rospy.Subscriber('/aruco_single/pose', PoseStamped, aruco_pose_callback)
    rospy.spin()

if __name__ == '__main__':
    main()




