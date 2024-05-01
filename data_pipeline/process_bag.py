import os
import rosbag
import numpy as np
import parser

import rosbag
import cv2
import pandas as pd
from sensor_msgs.msg import Image
from cv_bridge import CvBridge



src_dir_list = ['route1/forward', 'route1/back', 'route2/forward','route2/back']
# src_dir_list = [ 'route3/forward', 'route3/back']
bag_topics = ["/camera/cam3/image_raw"]

def save_images_from_rosbag(rosbag_files, bag_topics, output_directory):

    gps_data = []
    camera_file_names = []
    files = os.listdir(rosbag_files)
    for rosbag_file in files:
        bridge = CvBridge()
        bag = rosbag.Bag(f"{rosbag_files}/{rosbag_file}" , 'r')
        default_camera_file = ""

        for topic, msg, t in bag.read_messages(topics=bag_topics):
            if "image_raw" in topic:
                cam_num = int(topic[11]) # Magic index at which camera number is mentioned in the topic string
                cam = "cam"+str(cam_num)

                try:
                    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                    # You can process the image here if needed
                    # For example, you can resize it or apply other OpenCV operations
                    # cv_image = cv2.resize(cv_image, (new_width, new_height))

                    # Construct the output file name based on the timestamp
                    file_name = f"{t.to_nsec()}.png"
                    # if cam_num == default_camera:
                    #     default_camera_file = file_name
                    camera_file_names.append(file_name)
                    output_path = f"{output_directory}/{file_name}"

                    # Save the image to the output directory
                    cv2.imwrite(output_path, cv_image)

                    print(f"Saved image: {output_path}")
                except Exception as e:
                    print(f"Error processing image: {e}")
            if "gps_data" in topic:
                data = msg.data
                if data[0] != 0.0:
                    gps_data.append(data)

        bag.close()
    return  camera_file_names, gps_data

# def read_bag_files(src_dir_list, out_dir):
#     for dir in src_dir_list:
#         files = os.listdir((dir))
#         files = [dir+"/"+file for file in files]
#         output_directory = "/home/ur10/mnt_pt/2023-27-09/" + dir.split("/")[5]+"/images/"+dir.split(("/"))[6]
#         output_images_path = "/home/ur10/mnt_pt/2023-27-09/" + dir.split("/")[5]+"/images_path/"+dir.split("/")[6]
#         print(f"Reading the data from {dir} and saving into {output_directory}, saving the file name and coord into {output_images_path}")
#         camera_files_names, gps_data  = save_images_from_rosbag(files, bag_topics, output_directory, 3)
#         new_gps_data = []
#         for i in range(1200):
#             if i % 10 == 0:
#                 if gps_data:
#                     new_gps_data.append(gps_data.pop(0))
#                 else:
#                     new_gps_data.append((new_gps_data[-1]))
#             else:
#                 new_gps_data.append(new_gps_data[0])

#         image_data = pd.DataFrame({'image_name':camera_files_names, 'coordinates':new_gps_data[:len(camera_files_names)]})
#         file_save_path = f"{output_images_path}/queryImagesPath.txt" # This will change according to reference query data.
#         image_data.to_csv(path_or_buf= file_save_path, index=False, header=None, sep=" ")




if __name__ == "__main__":
    # rosbag_file = "/home/ur10/ros_collection/ouster_ros1"  # Replace with your ROS bag file path
    rosbag_file = "/media/ur10/IDD-3/test1_cam"
    image_topic = "/camera/cam3/image_raw"  # Replace with your image topic
    output_directory = "saved_images"  # Replace with your desired output directory
    capture_data_day = "2023-27-09/"
    # src_dir_list = ["/home/ur10/mnt_pt/"+capture_data_day+dir_list for dir_list in src_dir_list]
    # read_bag_files( src_dir_list, "")
    src_dir_list = ["/media/ur10/IDD-3/test1_cam"]
    save_images_from_rosbag(rosbag_file, bag_topics, output_directory)
