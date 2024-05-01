# from gmplot import gmplot

# # Create a gmplot object
# gmap = gmplot.GoogleMapPlotter(37.428, -122.145, 16)

# # List of coordinates (latitude and longitude)
# coordinates = [(37.4219999, -122.0840575), (37.4229998, -122.0840574), (37.4239997, -122.0840573)]

# # Create a list of marker colors
# colors = ['red', 'green', 'blue']

# # Loop through the coordinates and add markers with different colors
# for i, (lat, lon) in enumerate(coordinates):
#     gmap.marker(lat, lon, color=colors[i])

# # Draw lines between all the coordinates
# for i in range(len(coordinates)):
#     lat1, lon1 = coordinates[i]
#     for j in range(i+1, len(coordinates)):
#         lat2, lon2 = coordinates[j]
#         gmap.plot([lat1, lat2], [lon1, lon2], color=colors[i], edge_width=2)

# # Draw the map to an HTML file
# gmap.draw("mymap.html")

# import folium
# from IPython.display import display

# # Create a basic map
# m = folium.Map(location=[51.505, -0.09], zoom_start=13)

# # Display the map
# display(m)

# # Define a function to add markers dynamically
# def add_marker(map, lat, lon, tooltip, popup):
#     folium.Marker([lat, lon], tooltip=tooltip, popup=popup).add_to(map)

# # Add markers dynamically
# add_marker(m, 51.5, -0.09, "Marker 1", "Popup for Marker 1")
# add_marker(m, 51.51, -0.1, "Marker 2", "Popup for Marker 2")


import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image, ImageDraw
import rosbag
from sensor_msgs.msg import Image
#from cv_bridge import CvBridge
import cv2
import os

route2 = [(17.4586, 78.3657), (17.3832, 78.4728)]


class GPSVis(object):
    """
        Class for GPS data visualization using pre-downloaded OSM map in image format.
    """
    def __init__(self,bag_path, data_path, map_path, points):
        """
        :param data_path: Path to file containing GPS records.
        :param map_path: Path to pre-downloaded OSM map in image format.
        :param points: Upper-left, and lower-right GPS points of the map (lat1, lon1, lat2, lon2).
        """
        self.data_path = data_path
        self.points = points
        self.map_path = map_path

        self.result_image = Image
        self.x_ticks = []
        self.y_ticks = []
        self.gps_topic = ""
        self.bag_directory = bag_path


    def read_bag(self):
        bag_files = sorted(os.listdir(self.bag_directory))
        gps_data = []
        
        for file in bag_files:
            print(f"Reading the bag {file}")
            bag = rosbag.Bag(f"{self.bag_directory}/{file}", 'r')

            for topic, msg, t in bag.read_messages(topics=self.gps_topic):
                    try:
                        data = msg.data
                        if data[0] != 0.0:
                            gps_data.append(data)
                    except Exception as e:
                        print(f"Error processing image: {e}")

        gps_array = np.array(gps_data)
        gps_dataFrame = pd.DataFrame({'Latitude':gps_array[:,0],'Longitude':gps_array[:,1]})
        file_save_path = "gps_data.csv" # This will change according to reference query data.
        image_data.to_csv(path_or_buf= file_save_path, index=False, header=None, sep=" ")
        return gps_data




    def plot_map(self, output='save', save_as='resultMap.png'):
        """
        Method for plotting the map. You can choose to save it in file or to plot it.
        :param output: Type 'plot' to show the map or 'save' to save it.
        :param save_as: Name and type of the resulting image.
        :return:
        """
        self.get_ticks()
        fig, axis1 = plt.subplots(figsize=(10, 10))
        axis1.imshow(self.result_image)
        axis1.set_xlabel('Longitude')
        axis1.set_ylabel('Latitude')
        axis1.set_xticklabels(self.x_ticks)
        axis1.set_yticklabels(self.y_ticks)
        axis1.grid()
        if output == 'save':
            plt.savefig(save_as)
        else:
            plt.show()

    def create_image(self, color, width=2):
        """
        Create the image that contains the original map and the GPS records.
        :param color: Color of the GPS records.
        :param width: Width of the drawn GPS records.
        :return:
        """
        data = pd.read_csv(self.data_path, names=['LATITUDE', 'LONGITUDE'], sep=',')

        self.result_image = Image.open(self.map_path, 'r')
        img_points = []
        gps_data = tuple(zip(data['LATITUDE'].values, data['LONGITUDE'].values))
        for d in gps_data:
            x1, y1 = self.scale_to_img(d, (self.result_image.size[0], self.result_image.size[1]))
            img_points.append((x1, y1))
        draw = ImageDraw.Draw(self.result_image)
        draw.line(img_points, fill=color, width=width)

    def scale_to_img(self, lat_lon, h_w):
        """
        Conversion from latitude and longitude to the image pixels.
        It is used for drawing the GPS records on the map image.
        :param lat_lon: GPS record to draw (lat1, lon1).
        :param h_w: Size of the map image (w, h).
        :return: Tuple containing x and y coordinates to draw on map image.
        """
        # https://gamedev.stackexchange.com/questions/33441/how-to-convert-a-number-from-one-min-max-set-to-another-min-max-set/33445
        old = (self.points[2], self.points[0])
        new = (0, h_w[1])
        y = ((lat_lon[0] - old[0]) * (new[1] - new[0]) / (old[1] - old[0])) + new[0]
        old = (self.points[1], self.points[3])
        new = (0, h_w[0])
        x = ((lat_lon[1] - old[0]) * (new[1] - new[0]) / (old[1] - old[0])) + new[0]
        # y must be reversed because the orientation of the image in the matplotlib.
        # image - (0, 0) in upper left corner; coordinate system - (0, 0) in lower left corner
        return int(x), h_w[1] - int(y)

    def get_ticks(self):
        """
        Generates custom ticks based on the GPS coordinates of the map for the matplotlib output.
        :return:
        """
        self.x_ticks = map(
            lambda x: round(x, 4),
            np.linspace(self.points[1], self.points[3], num=7))
        y_ticks = map(
            lambda x: round(x, 4),
            np.linspace(self.points[2], self.points[0], num=8))
        # Ticks must be reversed because the orientation of the image in the matplotlib.
        # image - (0, 0) in upper left corner; coordinate system - (0, 0) in lower left corner
        self.y_ticks = sorted(y_ticks, reverse=True)

if __name__ == '__main__':
    vis = GPSVis(bag_path ="/media/ur10/IDD-3/2023-10-16/route2/forward",
             data_path='data.csv',
             map_path='map.png',  # Path to map downloaded from the OSM.
             points=(45.8357, 15.9645, 45.6806, 16.1557)) # Two coordinates of the map (upper left, lower right)
    vis.read_bag()