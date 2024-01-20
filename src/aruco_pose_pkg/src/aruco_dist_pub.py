#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
import numpy as np

def webcam_publisher():
    
    # Initialize ROS node
    rospy.init_node('webcam_publisher', anonymous=True)
    rate = rospy.Rate(10)  # Adjust the rate as needed

    # Initialize OpenCV and CvBridge
    cap = cv2.VideoCapture(2)
    bridge = CvBridge()
    # Create a publisher for the image
    image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)

    # Create a publisher for Aruco marker pose
    aruco_pose_pub = rospy.Publisher('/aruco_single/pose', PoseStamped, queue_size=10)

    # Define Aruco dictionary and parameters
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters_create()

    while not rospy.is_shutdown():
        # Capture frame from webcam
        ret, frame = cap.read()

        if ret:
            # Convert the OpenCV image to a ROS image message
            ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")

            # Detect Aruco markers in the frame
            corners, ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=parameters)
            print(corners)

            if ids is not None and len(ids) > 0:
                print(ids)
                # Get the first detected marker's ID
                marker_id = ids[0][0]

                # Get the rotation and translation vectors for the first marker
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.1, camera_matrix, dist_coeffs)

                # Create a PoseStamped message
                pose_msg = PoseStamped()
                pose_msg.header.stamp = rospy.Time.now()
                pose_msg.header.frame_id = "camera_frame"  # Replace with your camera frame ID

                # Set position (X, Y, Z)
                pose_msg.pose.position.x = tvec[0][0][0]
                pose_msg.pose.position.y = tvec[0][0][1]
                pose_msg.pose.position.z = tvec[0][0][2]

                # Set orientation (using a default quaternion)
                pose_msg.pose.orientation.x = 0.0
                pose_msg.pose.orientation.y = 0.0
                pose_msg.pose.orientation.z = 0.0
                pose_msg.pose.orientation.w = 1.0

                # Publish the Aruco marker pose
                aruco_pose_pub.publish(pose_msg)

            # Publish the image
            image_pub.publish(ros_image)
        
            cv2.imshow("Webcam Image", frame)
            cv2.waitKey(1)

        rate.sleep()

    # Release the webcam when the node is shutdown
    cap.release()

if __name__ == '__main__':
    try:
        calibration_data = np.load('/home/janhavi/AiPhile_cal/calib_data/MultiMatrix.npz')
        camera_matrix = calibration_data['camMatrix']
        dist_coeffs = calibration_data['distCoef']
        r_vectors = calibration_data['rVector']
        t_vectors = calibration_data['tVector']
        webcam_publisher()
    except rospy.ROSInterruptException:
        pass
