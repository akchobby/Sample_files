import cv2
import rosbag 
import rospy
import numpy as np
import multiprocessing as mp


from cv_bridge import CvBridge


def display_distorted(data):
    
    if data[0]:
        for topic, msg, t in bag.read_messages(cam_topic_name):
            img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            cv2.imshow('original', img)
            cv2.waitKey(1)
    else:
        for topic, msg, t in bag.read_messages(cam_topic_name):
            img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            img_undistorted = cv2.fisheye.undistortImage(img, data[1], data[2], Knew=data[3])

            cv2.imshow('undistorted', img_undistorted)
            cv2.waitKey(1)

cam_topic_name = "camera_45_deg/fisheye1/image_raw"
cam_info_topic_name = "camera_45_deg/fisheye1/camera_info"

# right scale still to be figured, 0.7 seems high tunnel effect prevalent
scale= 0.7

bridge = CvBridge()
bag = rosbag.Bag("localization_camera_45_deg_2021-06-09-15-07-31_7min.bag")

for topic, msg, t in bag.read_messages(cam_info_topic_name):
    distortion_coeffs = np.array(list(msg.D))[:-1]
    camera_matrix = np.array(msg.K).reshape(3,3)
    break

Knew = camera_matrix.copy()
Knew[(0,1),(0,1)] = scale * Knew[(0,1),(0,1)]
print(Knew[(0,1),(0,1)])


data_distorted = [True]
data_undistorted = [False, camera_matrix, distortion_coeffs, Knew]

# VM curently set only to 1 cpu , hence fully not used
p = mp.Pool() # mp.Pool(4)
p.map(display_distorted, [ data_undistorted])  # p.map(display_distorted, [data_distorted, data_undistorted])





