import rosbag
from sensor_msgs.msg import Image
#from cv_bridge import CvBridge
import cv2
import os
import numpy as np
import sys
import pandas as pd

# Define the path to the ROS bag file
# bag_file = '_2023-09-25-06-35-28_0.bag'

# # Define the directory where you want to save the images
# output_directory = 'output_images/'

# Create the output directory if it doesn't exist
# os.makedirs(output_directory, exist_ok=True)

# Create a CvBridge object to convert ROS Image messages to OpenCV images
#bridge = CvBridge()

def imgmsg_to_cv2(img_msg):
    if img_msg.encoding != "bgr8":
        rospy.logerr("This Coral detect node has been hardcoded to the 'bgr8' encoding.  Come change the code if you're actually trying to implement a new camera")
    dtype = np.dtype("uint8") # Hardcode to 8 bits...
    dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
    image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), # and three channels of data. Since OpenCV works with bgr natively, we don't need to reorder the channels.
                    dtype=dtype, buffer=img_msg.data)
    # If the byt order is different between the message and the system.
    if img_msg.is_bigendian == (sys.byteorder == 'little'):
        image_opencv = image_opencv.byteswap().newbyteorder()
    return image_opencv

# src_dir_list = ['route1/forward', 'route1/back', 'route2/forward','route2/back']
src_dir_list = [ 'route2/forward', 'route2/back']
bag_topics = ["/camera/cam0/image_raw", "/camera/cam1/image_raw", "/camera/cam2/image_raw", "/camera/cam3/image_raw",
              "/camera/cam4/image_raw", "/camera/cam5/image_raw", "/gps_data/gps/lat_lon"]
camera_file_names_0 = []

def save_images_from_rosbag(rosbag_files, bag_topics, output_directory, default_camera):

    gps_data = []
    camera_file_names = {'cam0':[],'cam1':[],'cam2':[],'cam3':[],'cam4':[],'cam5':[]}
    # rosbag_files = ['forward_2023-09-21-13-55-23_5.bag']
    print(rosbag_files)
    for rosbag_file in rosbag_files:

        # bridge = CvBridge()
        bag = rosbag.Bag(rosbag_file, 'r')
        default_camera_file = ""
        file_names = []
        for topic, msg, t in bag.read_messages(topics=bag_topics):
            if "image_raw" in topic:
                cam_num = int(topic[11]) # Magic index at which camera number is mentioned in the topic string
                cam = "cam"+str(cam_num)

                try:
                    cv_image = imgmsg_to_cv2(msg)
                    # You can process the image here if needed
                    # For example, you can resize it or apply other OpenCV operations
                    # cv_image = cv2.resize(cv_image, (new_width, new_height))

                    # Construct the output file name based on the timestamp
                    file_name = f"{t.to_nsec()}.png"
                    # if cam_num == default_camera:
                    #     default_camera_file = file_name
                    camera_file_names[cam].append(file_name)
                    output_path = f"{output_directory}/{cam}/{file_name}"

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

def read_bag_files(src_dir_list, out_dir):
    print(src_dir_list)
    for dir in src_dir_list:
        print(dir)
        files = os.listdir((dir))
        # print(dir)
        new_files = []
        for file in files:
            if os.path.isdir(dir+"/"+file):
                a =1
            else:
                # files.remove(file)
                new_files.append(dir+"/"+file)
        # files = [dir+"/"+file for file in files]
        # print(new_files)
        output_directory = "/media/ur10/IDD-3/2023-10-16_modified/" + dir.split("/")[4]+"/imges/"+dir.split(("/"))[5]
        output_images_path = "/media/ur10/IDD-3/2023-10-16//" + dir.split("/")[4]+"/images_path/"+dir.split("/")[5]
        # print(f"Reading the data from {dir} and saving into {output_directory}, saving the file name and coord into {output_images_path}")
        camera_files_names, gps_data  = save_images_from_rosbag(new_files, bag_topics, output_directory, 3)
        print(gps_data)
        new_gps_data = []
        print(f"Completed the processing now saving the related names...")
        base_file_num = min(len(camera_files_names['cam0']),len(camera_files_names['cam1']),len(camera_files_names['cam2']),len(camera_files_names['cam3']),len(camera_files_names['cam4']),len(camera_files_names['cam5']))
        for i in range((base_file_num)):
            if i % 10 == 0:
                if gps_data:
                    print('adding new coords')
                    new_gps_data.append(gps_data.pop(0))
                else:
                    new_gps_data.append((new_gps_data[-1]))
            else:
                new_gps_data.append(new_gps_data[0])

        image_data = pd.DataFrame({'cam0':camera_files_names['cam0'][:base_file_num-1],'cam1':camera_files_names['cam1'][:base_file_num-1],'cam2':camera_files_names['cam2'][:base_file_num-1],'cam3':camera_files_names['cam3'][:base_file_num-1],'cam4':camera_files_names['cam4'][:base_file_num-1],'cam5':camera_files_names['cam5'][:base_file_num-1], 'coordinates':new_gps_data[:base_file_num-1]})
        file_save_path = f"{output_images_path}/queryImagesPath.txt" # This will change according to reference query data.
        image_data.to_csv(path_or_buf= file_save_path, index=False, header=None, sep=" ")


if __name__ == "__main__":
    rosbag_file = "your_rosbag.bag"  # Replace with your ROS bag file path
    image_topic = "/camera/image_raw"  # Replace with your image topic
    output_directory = "/media/ur10/IDD-3/2023-10-16/"  # Replace with your desired output directory
    capture_data_day = "/media/ur10/IDD-3/2023-10-16/"
    src_dir_list = [capture_data_day+dir_list for dir_list in src_dir_list]
    read_bag_files( src_dir_list, "")
    # save_images_from_rosbag(rosbag_file, bag_topics, output_directory)





# Put the method under image and sequence level.
# Find the matrices.
# Find the github implementation.
# Transformer based and CNN based.
# Compare a common metric for bot the retrival and sequence matching method.
# Data collection after a week can be staggered across weeks.