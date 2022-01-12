#!/usr/bin/env python

""" rectifier.py: Rectifies fish camera images from rosbags 
                  and generates a new bag from it """
                  
__author__ = "Anil Kumar Chavali trial commit"

import cv2
import re
import sys
import rosbag 
import rospy
import numpy as np
import multiprocessing as mp


from cv_bridge import CvBridge

def add_img_topic_names(data, pattern):
    data["image_topics"] = {}
    topics = rosbag.Bag(data["filename"]).get_type_and_topic_info()[1].keys()
    for topic in topics:
        if re.search(pattern, topic):
            data["image_topics"][topic] = []




def display_data(data):
    
    bridge = CvBridge()
    if data["simple_display"]:

        for topic, msg, t in rosbag.Bag(data["filename"]).read_messages(data["image_topics"].keys()):
            img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            cv2.imshow('original', img)
            cv2.waitKey(1)
    else:
        for topic, msg, t in rosbag.Bag(data[1]).read_messages(data["image_topics"].keys()):

            img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')  
            img_undistorted = cv2.fisheye.undistortImage(img,
                                                        D=data["image_topics"][topic][0], 
                                                        K=data["image_topics"][topic][1],
                                                        Knew=data["image_topics"][topic][2])

            cv2.imshow('undistorted', img_undistorted)
            cv2.waitKey(1)

def rectify_bag(data):

    bridge = CvBridge()
    outbag_name = data["filename"].split(".")[0].strip() + "_rectified_imgs.bag"

    cam_info_topic_names = ["/".join(name.split("/")[:-1] + ["camera_info"])for name in data["image_topics"].keys()]

    with rosbag.Bag(outbag_name, 'w') as outbag:
        for topic, msg, t in rosbag.Bag(data["filename"]).read_messages():

            if topic in data["image_topics"].keys():

                img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

                img_undistorted = cv2.fisheye.undistortImage(img,
                                                            D=data["image_topics"][topic][0], 
                                                            K=data["image_topics"][topic][1],
                                                            Knew=data["image_topics"][topic][2])


                msg = bridge.cv2_to_imgmsg(img_undistorted, encoding='passthrough')
            
            # camera info for rectified image, changed as well
            if topic in cam_info_topic_names:
                img_key = "/".join(topic.split("/")[:-1] + ["image_raw"])

                msg.D = [0.0, 0.0, 0.0, 0.0, 0.0]
                msg.K = data["image_topics"][img_key][2].reshape(9).tolist()

                tmp = list(msg.P)
                tmp[0] = msg.K[0]
                tmp[4] = msg.K[3]
                msg.P = tmp

            outbag.write(topic, msg, t) 

def convert_camera_info(data):

    cam_info_topic_names = ["/".join(name.split("/")[:-1] + ["camera_info"])for name in data["image_topics"].keys()]

    for topic, msg, t in rosbag.Bag(data["filename"]).read_messages():

        if topic in cam_info_topic_names:
            
            distortion_coeffs = np.array(list(msg.D))[:-1]
            camera_matrix = np.array(msg.K).reshape(3,3)
            Knew = camera_matrix.copy()
            Knew[(0,1),(0,1)] = data["scale"] * Knew[(0,1),(0,1)] # hard coded scale value

            img_key = "/".join(topic.split("/")[:-1] + ["image_raw"])
            data["image_topics"][img_key] = [distortion_coeffs,camera_matrix, Knew]
            cam_info_topic_names.remove(topic)
            
        # to prevent unecessary looping
        if len(cam_info_topic_names) < 1:
            break


    for key in data["image_topics"].keys():
        assert len(data["image_topics"][key]) == 3, "Camera data not associated with {} topic".format(key)


def main():
    """
    data dict params:

    simple_display: for simple or undistrot and display 
    original_rosbag: path to rosbag
    image_topics: list of image topic names to rectify --> make it dict ? such that data is associated with topic name



    """
    
    data = {"simple_display": True, 
            "filename": "localization_camera_45_deg_2021-06-09-15-07-31_7min.bag",
            "scale": 0.7}
    
    # give pattern to match and topics will be added
    add_img_topic_names(data, r"\/image_raw$")

    if sys.argv[1].lower() == "true":
        print("Generating rectified image bag ...")
        # saving camera info in data
        convert_camera_info(data)

        # undistortion of images and correction of camera info
        rectify_bag(data)

    if sys.argv[2].lower() == "true":
        print("Displaying images ...")
        # display results from rectified bag
        data_rectified = data.copy()
        data_rectified["filename"] = data_rectified["filename"].split(".")[0].strip() + "_rectified_imgs.bag"        

    
        # Multiprocessing to allow quicker simultaneous display, 
        # use killall python to abort need to add SIGINT capture still
        my_settings = [data, data_rectified]
        p = mp.Pool(6) 
        results = p.map(display_data, my_settings)
    


if __name__=="__main__":
    main()







