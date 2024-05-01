# import os
# import numpy as np
# import pandas as pd

# src_path = ["/home/ur10/mnt_pt/media1/reference_data/route1/images/back/cam0/","/home/ur10/mnt_pt/media1/reference_data/route1/images/back/cam1/", "/home/ur10/mnt_pt/media1/reference_data/route1/images/back/cam2/", "/home/ur10/mnt_pt/media1/reference_data/route1/images/back/cam3/", "/home/ur10/mnt_pt/media1/reference_data/route1/images/back/cam4/", "/home/ur10/mnt_pt/media1/reference_data/route1/images/back/cam5/"]
# cam_files = []
# dir = os.walk(src_path)
# for sub_dir in src_path:
# 	files = []
# 	# print('hre')
# 	print(sub_dir)
# 	image_files = os.listdir(sub_dir)
# 	# print(image_files)
# 	for file in image_files:
# 		print('here')
# 		files.append(file)
# 	cam_files.append(files)

# # stacked_files = np.column_stack((cam_files[0], cam_files[1], cam_files[2], cam_files[3], cam_files[4], cam_files[5]))
# print(cam_files)
# image_data = pd.DataFrame({'cam0':cam_files[0][:65],'cam1':cam_files[1][:65],'cam2':cam_files[2][:65],'cam3':cam_files[3][:65],'cam4':cam_files[4][:65],'cam5':cam_files[5][:65], 'coords':65*[(17.493770856, 78.35127406)]})
# file_save_path = "sample.txt"
# image_data.to_csv(path_or_buf= file_save_path, index=False, header=None, sep=" ")
import rosbag
from sensor_msgs.msg import Image
#from cv_bridge import CvBridge
import cv2
import os
import numpy as np
import sys
# Define the path to the ROS bag file
bag_file = '_2023-09-25-06-35-28_0.bag'

# Define the directory where you want to save the images
output_directory = 'output_images/'


# Create the output directory if it doesn't exist
os.makedirs(output_directory, exist_ok=True)

# Create a CvBridge object to convert ROS Image messages to OpenCV images
bridge = CvBridge()


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

# Open the ROS bag file
with rosbag.Bag(bag_file, 'r') as bag:
    for topic, msg, t in bag.read_messages():
        if topic == '/your/image/topic':  # Replace with the actual topic containing image messages
            try:
                # Convert the ROS Image message to an OpenCV image
                cv_image = bridge.imgmsg_to_cv2(msg)
                
                # Get the nanosecond timestamp
                timestamp_ns = msg.header.stamp.to_nsec()
                
                # Define the image file name based on the timestamp
                image_filename = os.path.join(output_directory, f'image_{timestamp_ns}.jpg')
                
                # Save the image
                cv2.imwrite(image_filename, cv_image)
                print(f'Saved image: {image_filename}')
            except Exception as e:
                print(f'Error processing image message: {str(e)}')
