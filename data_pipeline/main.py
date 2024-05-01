# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.

# import pandas as pd
# def print_hi(name):
#     # Use a breakpoint in the code line below to debug your script.
#     print(f'Hi, {name}')  # Press Ctrl+F8 to toggle the breakpoint.
#
#
# # Press the green button in the gutter to run the script.
# if __name__ == '__main__':
#     image_data = pd.DataFrame({'image_name': [1,2,3], 'coordinates': [(1,2), (3,4),(5,6)]})
#     image_data.to_csv('test.txt', index=False, header=None, sep=" ")
#     print_hi('PyCharm')
#
# # See PyCharm help at https://www.jetbrains.com/help/pycharm/

import rosbag
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def save_images_from_rosbag(rosbag_file, image_topic, output_directory):
    bridge = CvBridge()
    bag = rosbag.Bag(rosbag_file, 'r')

    for topic, msg, t in bag.read_messages(topics=[image_topic]):
        try:
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            # You can process the image here if needed
            # For example, you can resize it or apply other OpenCV operations
            # cv_image = cv2.resize(cv_image, (new_width, new_height))

            # Construct the output file name based on the timestamp
            file_name = f"{t.to_nsec()}.png"
            output_path = f"{output_directory}/{file_name}"

            # Save the image to the output directory
            cv2.imwrite(output_path, cv_image)

            print(f"Saved image: {file_name}")
        except Exception as e:
            print(f"Error processing image: {e}")

    bag.close()


if __name__ == "__main__":
    rosbag_file = "/home/ur10/mnt_pt/test_folder/route3/forward/_2023-09-11-14-44-28_0.bag"  # Replace with your ROS bag file path
    image_topic = "/camera/cam0/image_raw"  # Replace with your image topic
    output_directory = "test_image"  # Replace with your desired output directory

    save_images_from_rosbag(rosbag_file, image_topic, output_directory)
