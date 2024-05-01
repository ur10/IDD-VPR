import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QLabel, QVBoxLayout, QGridLayout, QSplitter, QFrame, QRadioButton, QPushButton, QMessageBox, QHBoxLayout, QButtonGroup, QSizePolicy, QSpacerItem, QFormLayout,QDialog,QGroupBox,QDesktopWidget,QGraphicsPixmapItem,QGraphicsView,QGraphicsScene,QLineEdit
from PyQt5.QtGui import QPixmap,  QFont, QImage
from PyQt5.QtCore import Qt, QThread, pyqtSignal,pyqtSlot, pyqtSignal
import os
import json
import pandas as pd
import math
import cv2
import numpy as np
import threading
import concurrent.futures
import yaml
import argparse
import subprocess
from pathlib import Path

    # TODO - Create a dictionary which helps in tagging the selected images.This is related to the json file creation. - PARTIAL COMPLETE
    # TODO - Check for distance between the tagged images. - PARTIAL COMPLETE CHECK NEEDS TO BE DONE
    # TODO - Create buttons for route1, route2, route3 and forward, back - PARTIAL COMPLETE
    # TODO - Check that the same image is not being tagged twice. - COMPLETED
    # TODO - Beautify - This include changing the color of the buttons, possibly alignment, also when you click on the query image it should enlarge.
    # TODO - Take bounds on the extent to which query images can be moved w.r.t. the ref image selected.
    # TODO - Highlight the clicked button as well as the image. - COMPLETED
    # TODO - Hide the image ranking, but display the image that in ranked manner.
    # TODO - Rearrange the select button. 
    # TODO - Add the labels for direction and the routes.
    # TODO - Remove the pop up on select button.
    # TODO - Prev limit route is still buggy.
    # TODO - Route and direction should be in different groups with the instruction given on top of the label. THe message should occur after both of them are selected.
    # GBXdMlJ&EfbKAYBZ

class LoadImageThread(QThread):
    loaded = pyqtSignal(list)  # Signal to return the loaded images

    def __init__(self, path, image_list, start_index, end_index):
        super().__init__()
        self.image_list = image_list
        self.path = path
        self.start_index = start_index
        self.end_index = end_index

    def run(self):
        loaded_images = []
        for i in range(self.start_index, self.end_index):
            image_path = f"{self.path}/{self.image_list[i]}" #f"{self.image_dir}/{i}.jpg"  # Replace with your image file naming logic
            try:
                print(f"Loading image{image_path}")
                image = cv2.imread(image_path)

                loaded_images.append(image)
            except Exception as e:
                print(f"Failed to load image at index {i}")
                break

        self.loaded.emit(loaded_images)

class HeavyComputationThread(QThread):
    """
    Thread performing the computation
    """

    # setup a signal, which takes a single object as parameter
    resultAvailable = pyqtSignal(object)

    def __init__(self, x, y):
        QThread.__init__(self)
        self.x = x
        self.y = y

        # mark the thread is alive
        self.alive = True

    # called when the thread starts running
    def run(self):
        # compute something
        sum = self.x + self.y

        # simulated delay
        for i in range(1, 6):
            print(f"Thread :: sleeping {i}")
            time.sleep(1)

            # make the thread interruptible - check the alive flag regularly if possible
            if not self.alive:
                break

        if not self.alive:
            print("Thread :: Computation cancelled")
            return

        print(f"Thread :: computation finished, sum is {sum}")

        # result can be complex, as an example I am passing a dictionary
        result = {
            'x': self.x,
            'y': self.y,
            'sum': sum
        }

        # emit result to the AppWidget
        self.resultAvailable.emit(result)

    def stop(self):
        # mark this thread as not alive
        self.alive = False
        # wait for it to really finish
        self.wait()


class AppWidget(QWidget):
    """
    Main (and only ;) ) UI element
    """

    def __init__(self):
        QWidget.__init__(self)
        self.resize(225, 100)
        self.setWindowTitle('Start async')

        # slot for the running thread, empty now
        self.running_thread = None

        self.startButton = QPushButton('Start computation', self)
        self.startButton.move(10, 50)
        self.startButton.clicked.connect(self.start)  # register onClick callback

        self.cancelButton = QPushButton('Cancel', self)
        self.cancelButton.move(130, 50)
        self.cancelButton.clicked.connect(self.cancel)  # register onClick callback
        self.cancelButton.setEnabled(False)

    @pyqtSlot()
    def start(self):
        """
        onclick callback of the startButton
        """
        print('UI :: Starting the computation')
        self.startButton.setEnabled(False)
        self.cancelButton.setEnabled(True)

        # create a background thread, execute something on it
        self.running_thread = HeavyComputationThread(31, 11)
        self.running_thread.resultAvailable.connect(self.result_ready)  # register a callback method
        self.running_thread.start()

    @pyqtSlot(object)
    def result_ready(self, result):
        """
        called when the computation is finished
        :param result: result of the computation
        """
        print(f"UI :: Received result {result}")
        self.startButton.setEnabled(True)
        self.cancelButton.setEnabled(False)
        self.running_thread = None

    @pyqtSlot()
    def cancel(self):
        print('UI :: Cancelling the thread')
        """
        onclick callback of the cancelButton
        """
        self.running_thread.stop()
        print('UI :: Thread cancelled')
        self.startButton.setEnabled(True)
        self.cancelButton.setEnabled(False)



class InitialGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        self.setWindowTitle("Initial GUI")
        self.setGeometry(100, 100, 300, 100)

        layout = QVBoxLayout()

        # Create the first group of radio buttons
        group1 = QGroupBox("Routes")
        group1_layout = QVBoxLayout()
        self.route1_button = QRadioButton("Route 1")
        self.route1_button.clicked.connect(lambda state, name="route1": self.set_route(name))
        self.route2_button = QRadioButton("Route 2")
        self.route2_button.clicked.connect(lambda state, name="route2": self.set_route(name))
        self.route3_button = QRadioButton("Route 3")
        self.route3_button.clicked.connect(lambda state, name="route3": self.set_route(name))
        group1_layout.addWidget(self.route1_button)
        group1_layout.addWidget(self.route2_button)
        group1_layout.addWidget(self.route3_button)
        group1.setLayout(group1_layout)
        layout.addWidget(group1)

        # Create the second group of buttons
        group2 = QGroupBox("Navigation")
        group2_layout = QVBoxLayout()
        self.forward_button = QRadioButton("Forward")
        self.forward_button.clicked.connect(lambda state, name="forward": self.set_direction(name))

        self.backward_button = QRadioButton("Backward")
        self.backward_button.clicked.connect(lambda state, name="back": self.set_direction(name))

        group2_layout.addWidget(self.forward_button)
        group2_layout.addWidget(self.backward_button)
        group2.setLayout(group2_layout)
        layout.addWidget(group2)

        self.text_input = QLineEdit()
        layout.addWidget(self.text_input)
        
        # Add a QLabel for the indicator message
        self.indicator_label = QLabel("annotation date")
        layout.addWidget(self.indicator_label)

        self.setLayout(layout)
        self.show_page_button = QPushButton("Show Page")
        self.show_page_button.clicked.connect(self.showPage)
        layout.addWidget(self.show_page_button)


        self.setLayout(layout)
        
        

    def showPage(self):
        # Check if the page is already displayed
        entered_date = self.text_input.text()
        print('Entered Date:', entered_date)
        if not hasattr(self, 'page') or not self.page:
            self.page = ImageGridApp(self.route, self.direction, entered_date)
        # self.page.reset_parameters()
        self.page.show()
    def set_route(self, route_name):
        self.route = route_name
        print(route_name)
    def set_direction(self, direction_name):
        self.direction = direction_name
        print(direction_name)

class EnlargedImageWindow(QDialog):
    def __init__(self, images_set1, images_set2, query_image_path, ref_image_path):
        super().__init__()

        self.setWindowTitle("Tagged Images")
        self.setMinimumWidth(1200)  # Set the minimum width of the dialog

        self.query_image_path = query_image_path
        self.ref_image_path = ref_image_path

        self.query_imgs = sorted(os.listdir(query_image_path))
        self.ref_imgs = sorted  (os.listdir(ref_image_path))

        # for i in range(len(query_imgs)):
        #     if images_set1[i] in query_imgs:
        #         self.images_set1.append(f"{query_image_path}/{images_set1[i]}") 

        #     if images_set2[i] in ref_imgs:
        #         self.images_set2.append(f"{ref_image_path}/{images_set2[i]}")

        self.images_set1 = images_set1
        self.images_set2 = images_set2
        self.current_index = 0  # Initialize the current index to 0

        self.set_number_label = QLabel(f"Set Number: {1}")

        # Create a layout for the label
        label_layout = QVBoxLayout()
        label_layout.addWidget(self.set_number_label)

        # Add the label layout to the main layout of the dialog
        layout = QVBoxLayout()
        # Add other widgets to the layout as needed

        # Add the label layout to the main layout
        layout.addLayout(label_layout)

        # self.setLayout(layout)

        # Create layouts for left and right image sets
        left_layout = QHBoxLayout()
        right_layout = QHBoxLayout()

        # Create labels for the current pair of images
        self.left_label = QLabel()
        self.right_label = QLabel()
        self.show_images()  # Display the initial pair of images

        # Create buttons to navigate to the next pair of images
        next_button = QPushButton("Next Pair")
        prev_button = QPushButton("Previous Pair")

        next_button.clicked.connect(self.next_pair)
        prev_button.clicked.connect(self.previous_pair)

        # Add labels and buttons to the layouts
        left_layout.addWidget(self.left_label)
        right_layout.addWidget(self.right_label)

        image_layout = QHBoxLayout()
        # image_layout.addLayout(layout)
        image_layout.addLayout(left_layout)
        image_layout.addLayout(right_layout)

        # Create a horizontal layout for navigation buttons
        nav_layout = QHBoxLayout()
        nav_layout.addWidget(prev_button)
        nav_layout.addWidget(next_button)

        # Set up the right layout
        # right_layout.addLayout(nav_layout)

        # Add layouts to the main layout
        main_layout = QVBoxLayout()
        main_layout.addLayout(layout)
        main_layout.addLayout(image_layout)
        main_layout.addLayout(nav_layout)

        # Set the main layout for the dialog
        self.setLayout(main_layout)

    def show_images(self):
        # Display the current pair of images
        right_image_path = f"{self.query_image_path}/{self.query_imgs[self.images_set2[self.current_index]]}"
        left_image_path = f"{self.ref_image_path}/{self.ref_imgs[self.images_set1[self.current_index]]}"
        print(left_image_path)
        left_pixmap = QPixmap(left_image_path)
        left_pixmap = left_pixmap.scaledToWidth(700)  # Adjust the width as needed
        self.left_label.setPixmap(left_pixmap)
        print(right_image_path)
        right_pixmap = QPixmap(right_image_path)
        right_pixmap = right_pixmap.scaledToWidth(700)  # Adjust the width as needed
        self.right_label.setPixmap(right_pixmap)

    def next_pair(self):
        # Move to the next pair of images
        self.current_index = (self.current_index + 1) % len(self.images_set1)
        self.show_images()
        self.set_number_label.setText(f"Set Number: {self.current_index}")

    def previous_pair(self):
        # Move to the previous pair of images
        self.current_index = (self.current_index - 1) % len(self.images_set1)
        self.show_images()
        self.set_number_label.setText(f"Set Number: {self.current_index}")

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        # Create a central widget and set it as the main window's central widget
        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)

        # Create a layout for the central widget
        layout = QVBoxLayout()

        # Create a button to open the enlarged image window
        open_button = QPushButton("Open Enlarged Image")
        open_button.clicked.connect(self.open_enlarged_image)

        # Add the button to the layout
        layout.addWidget(open_button)

        # Set the layout for the central widget
        central_widget.setLayout(layout)

    def open_enlarged_image(self):
        # Sample image paths for the two sets (replace with your image paths)
        images_set1 = ["image1.jpg", "image2.jpg", "image3.jpg"]
        images_set2 = ["image4.jpg", "image5.jpg", "image6.jpg"]

        # Create and show the enlarged image window
        enlarged_window = EnlargedImageWindow(images_set1, images_set2)
        enlarged_window.exec_()

class ImageGridApp(QMainWindow, QDialog):
    def __init__(self, route, direction, date):
        super().__init__()

        # Set up the main window
        self.setWindowTitle('VPR ANNOTATION TOOL')
        screen = QDesktopWidget().screenGeometry()
        
        ssh_path = "utkarsh_rai@10.4.16.30:/mnt/base/idd_comprehensive"
        
        
        # Calculate the window size as a percentage of the screen size
        window_width = screen.width() * 0.5  # Adjust the width as needed
        window_height = screen.height() * 0.5  # Adjust the height as needed

        # Calculate the x and y position to center the window
        x = (screen.width() - window_width) / 2
        y = (screen.height() - window_height) / 2
        if route != "route3":
            ref_dir_name = "/home/cvit/idd_comp/mnt_pt/2023-27-09_modified"
            # ref_dir_name = "/home/ur10/mnt_pt/2023-27-09_modified"
        else:
            ref_dir_name = "/home/cvit/idd_comp/mnt_pt/2023-10-03_modified"
            # ref_dir_name = "/home/ur10/mnt_pt/2023-10-03_modified"
            


        query_dir_name = "/home/cvit/idd_comp"
        # query_dir_name = "/home/ur10/annotation_pipeline"
        subprocess.run(["mkdir", "-p", f"{query_dir_name}/{date}_modified/annotation_data"])

        subprocess.run(["mkdir", "-p", f"{query_dir_name}/{date}_modified/{route}/images/{direction}/cam3"])
        subprocess.run(["mkdir", "-p", f"{query_dir_name}/{date}_modified/{route}/images_path/{direction}"])
        # subprocess.run(["sshpass", "-p", "GBXdMlJ&EfbKAYBZ", "rsync", "-r", "--info=progress2", f"{ssh_path}/{date}_modified/{route}/images_path/{direction}/queryImagesPath.txt", f"{query_dir_name}/{date}_modified/{route}/images_path/{direction}"])
        # subprocess.run(["sshpass", "-p", "GBXdMlJ&EfbKAYBZ", "rsync", "-r", "--info=progress2", f"{ssh_path}/{date}_modified/{route}/images/{direction}/cam3/", f"{query_dir_name}/{date}_modified/{route}/images/{direction}/cam3"])

        # Set the window size and position
        # self.setGeometry(x, window_width,y, window_height)
        self.json_data_list = []
        self.select_button = []
        self.mont_dic = {'01':'Jan', '02':'Feb','03':'Mar','04':'Apr','05':'May','06':'Jun','07':'Jul','08':'Aug','09':'Sep','10':'Oct','11':'Nov','12':'Dec'}
        self.tem_dict = {"route":0, "direction":"forward", "ref_images":{"cam0":"", "cam1":"", "cam2":"", "cam3":"", "cam4":"", "cam5":""}, "query_images":{"cam0":"", "cam1":"", "cam2":"", "cam3":"", "cam4":"", "cam5":""},"coordinate":[]}
        self.ref_image_dir = f"{ref_dir_name}"
        self.query_image_dir = f"{query_dir_name}/{date}_modified"
        self.current_route = route
        self.current_direction = direction
        self.ref_image_path = f"{self.ref_image_dir}/{self.current_route}/images/{self.current_direction}/cam3"
        self.query_image_path = f"{self.query_image_dir}/{self.current_route}/images/{self.current_direction}/cam3"
        self.selected = False
        self.selected_image_label = QLabel('<html><b><font color="blue">LAST SELECTED IMAGE</font></b></html>')
        self.image_paths = sorted(os.listdir(self.query_image_path))
        self.buffer_query_image_list = []
        self.buffer_ref_image_list = []
        self.query_clicked_list = []
        self.ref_heading = QLabel('<html><b><font size = 7 color="green">REFERENCE IMAGE</font></b></html>')
        self.query_heading = QLabel('<html><b><font size = 7 color="green">QUERY IMAGE</font></b></html')
        self.instruction_label = QLabel('<html><b><font size = 2 color="blue"> Instruction to select image-<br> 1.Click on the [SELECT] button to select an image <br> 2. Select the [CONFIRM] button\n if match is found <br>3. Select the [NEXT SET] button in the right column for next QUERY IMAGE\n<br>4. Select the [NEXT] button from the left column for next REF IMAGE</font></b></html>')


        self.annotation_path = f"{self.query_image_dir}/annotation_data/annotation.json"
        self.saved_annotations_file = f"{self.query_image_dir}/saved_annotation.json"


        self.single_image_paths = os.listdir(self.ref_image_path)
        self.single_image_paths = sorted(self.single_image_paths)
        self.current_single_image_index = -1
        self.tagged_ref_image = []
        self.tagged_query_image = []

        self.query_image_info = f"{self.query_image_dir}/{self.current_route}/images_path/{self.current_direction}/queryImagesPath.txt"
        self.query_route_data = pd.read_csv(self.query_image_info, sep = " ", names = ['cam0','cam1', 'cam2', 'cam3', 'cam4', 'cam5', 'coords'])
        self.query_selection_idx = []

        self.ref_image_info = f"{self.ref_image_dir}/{self.current_route}/images_path/{self.current_direction}/referenceImagesPath.txt"
        self.ref_route_data = pd.read_csv(self.ref_image_info, sep = " ", names = ['cam0','cam1', 'cam2', 'cam3', 'cam4', 'cam5', 'coords'])
        self.current_query_clicked_idx = 0
        self.clicked_image_index = 0
        self.query_loading = False
        self.ref_loading = False
        print(self.ref_route_data.iloc[0]["coords"])

        self.last_selected_ref_idx = -2
        self.last_query_selected_idx = -2
        self.match_dict = dict()
        temp_data = {}

        annotations_file = Path(self.annotation_path)

        if annotations_file.is_file():
            with open(self.annotation_path, 'r') as fin:
                temp_data = json.load(fin)

            self.json_data_list.append(temp_data)
        else:
            subprocess.run(["touch", f"{query_dir_name}/{date}_modified/annotation_data/annotation.json"])


        # self.enlarged_window = EnlargedImageWindow()
        
        self.query_images_np_list = []
        self.ref_images_np_list = []



        # Create a central widget to hold the layout
        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)

        # Create a splitter to divide the central widget into two columns
        splitter = QSplitter()

        # Create the left column for the single imag        subprocess.run(["mkdir", "-p", f"{query_dir_name}/{route}/images/{direction}/cam3"])
        # subprocess.run(["mkdir", "-p", f"{query_dir_name}/{route}/images_path/{direction}"])
        # subprocess.run(["sshpass", "-p", "GBXdMlJ&EfbKAYBZ", "rsync", "-r", "--info=progress2", f"{ssh_path}/{date}/{route}", f"{ref_dir_name}"])
        left_column = QWidget()
        left_layout = QVBoxLayout(left_column)
        print(f"Screen width is {self.frameGeometry().width()}")
        # Create radio buttons for route selection
        
        left_layout.addWidget(self.ref_heading)
        left_layout.addWidget(self.selected_image_label)
        self.selected_image_label.hide()


        # Add direction radio buttons to the left layout
        # left_layout.addWidget(forward_button)
        # left_layout.addWidget(back_button)

        # Create prev/next buttons for the single image
        self.prev_button_single = QPushButton("Previous")
        self.prev_button_single.setFixedWidth(80)
        self.prev_button_single.clicked.connect(self.show_previous_single_image)
        self.next_button_single = QPushButton("Next")
        self.next_button_single.setFixedWidth(80)
        self.next_button_single.clicked.connect(self.show_next_single_image)
        
        prev_next_button_style = "background-color: #3498db; color: white; font-weight: bold;"
        self.prev_button_single.setStyleSheet(prev_next_button_style)
        self.next_button_single.setStyleSheet(prev_next_button_style)
        
        button_layout = QHBoxLayout()
        button_layout.addWidget(self.prev_button_single)
        button_layout.addWidget(self.next_button_single)

        # Load and display a single image in the left column
        self.single_image_label = QLabel()
          # Track the current single image index

        left_layout.addWidget(self.single_image_label)
        left_layout.addSpacing(20)

        left_layout.addLayout(button_layout)

        # left_layout.addWidget(self.prev_button_single)
        # left_layout.addWidget(self.next_button_single)
        # left_layout.addWidget(self.confirm_button)

        # Inside the __init__ method of ImageGridApp class

        # Create labels for displaying current route and direction
        self.current_route_label = QLabel("Current Route: route1")
        self.current_direction_label = QLabel("Current Direction: forward")
        
        if date != "":
            date_list = date.split('-')    
            month = self.mont_dic[date_list[1]]
            date_list[1] =month
            self.current_date_label = QLabel(f"Current Date: {date_list[2]}-{date_list[1]}-{date_list[0]}")
        else:
            self.current_date_label = QLabel("ENTER A VALID DATE")
        # Set styles for the labels
        label_style = "font-size: 16px; font-weight: bold; color: #333333;"

        self.current_route_label.setStyleSheet(label_style)
        self.current_direction_label.setStyleSheet(label_style)
        self.current_date_label.setStyleSheet(label_style)

        # Create a layout for the labels
        bottom_left_layout = QVBoxLayout()

        # Add space above the labels
        bottom_left_layout.addSpacing(10)

        # Add the labels to the layout
        bottom_left_layout.addWidget(self.current_route_label)
        bottom_left_layout.addSpacing(5)  # Add space between the labels
        bottom_left_layout.addWidget(self.current_direction_label)
        bottom_left_layout.addStretch()  # Add stretch to push labels to the bottom
        bottom_left_layout.addWidget(self.current_date_label)
        bottom_left_layout.addStretch()  # Add stretch to push labels to the bottom

        # Add the bottom left layout to the left column layout
        left_layout.addLayout(bottom_left_layout)

        # Set initial text for labels
        self.current_route_label.setText(f"Current Route: {self.current_route}")
        self.current_direction_label.setText(f"Current Direction: {self.current_direction}")

        left_layout.addWidget(self.instruction_label)

        # Add a spacer to align the Refresh button to the right
        spacer = QWidget()
        spacer.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        left_layout.addWidget(spacer)

        # Create a separator between the columns
        separator = QFrame()
        separator.setFrameShape(QFrame.VLine)
        separator.setFrameShadow(QFrame.Sunken)

        # Create the right column for a grid of 10 images
        right_column = QWidget()
        self.right_layout = QGridLayout(right_column)

        self.image_labels = []
        # self.current_set_start_index = 0

        if os.path.isfile(self.saved_annotations_file):
            print("Opening from the last place.....")
            with open(self.saved_annotations_file,'r') as file:
                saving_dict = json.load(file)
                self.match_dict = saving_dict['match_dict']
                self.current_single_image_index = saving_dict['last_ref_image']
                self.current_set_start_index = saving_dict['last_query_image'] 
                self.last_selected_ref_idx =  saving_dict['check_last_ref_image']
                self.last_query_selected_idx = saving_dict['check_last_query_image']

            for key,value in self.match_dict.items():
                self.tagged_ref_image.append(int(key))
                self.tagged_query_image.append(int(value))
        else:
            self.current_set_start_index = 0
            self.current_single_image_index = 0

        for i in range(0):
            print('here')
            ref_img = cv2.imread(f"{self.ref_image_path}/{self.single_image_paths[self.current_single_image_index + i]}")
            self.ref_images_np_list.append(ref_img)

            query_img = cv2.imread(f"{self.query_image_path}/{self.image_paths[self.current_set_start_index + i]}")
            # print(self.image_paths[i])
            self.query_images_np_list.append(query_img)

        for i in range(0):
            print('hrer')
            ref_img_buf = cv2.imread(f"{self.ref_image_path}/{self.single_image_paths[self.current_single_image_index+ 100+i]}")
            self.buffer_ref_image_list.append(ref_img_buf)
            print(self.single_image_paths[self.current_single_image_index+ 100+i])

            query_img_buf = cv2.imread(f"{self.query_image_path}/{self.image_paths[self.current_set_start_index + i+100]}")
            self.buffer_query_image_list.append(query_img_buf)

        

        # Load and display a set of 10 images in the right column
        for i in range(10):
            image_layout = QVBoxLayout()  # Create a vertical layout for each image item

            image_label = QLabel()
            image_path = f"{self.query_image_path}/{self.image_paths[self.current_set_start_index + i]}"
            print(f"Opening image  {image_path} of index : {self.current_set_start_index+i}")
            pixmap = QPixmap(image_path)  # Replace with your image loading logic
            pixmap = pixmap.scaled(250, 300)
            image_label.setPixmap(pixmap)

            self.right_layout.addWidget(image_label, i // 5, i % 5)
            self.image_labels.append(image_label)
            self.select_button_style = "background-color: #f39c12; color: white; font-weight: bold;"

            self.select_button.append (QPushButton("Select"))
            self.select_button[i].clicked.connect(lambda state, index=i: self.select_image(index))
            self.select_button[i].setCheckable(True)
            self.select_button[i].setStyleSheet(self.select_button_style)
            self.select_button[i].setFixedWidth(200)



            # Add select button to the vertical layout
            # image_layout.addWidget(self.select_button[i])
            w = self.select_button[i].pos().x()
            y = self.select_button[i].pos().y()
            print(f"The button heigth and width are {y} {w}")
            spacer = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)


            # Add the vertical layout to the grid
            image_layout = QFormLayout()
            image_layout.addRow(image_label)
            image_layout.addRow(self.select_button[i])
            # image_layout.addRow(label_index)

            self.right_layout.addLayout(image_layout, i // 5, i % 5)
            # right_layout.addLayout(image_layout, i // 5, i % 5)
        

        self.load_single_image()
        

        self.prev_button_set = QPushButton("Previous Set")
        self.prev_button_set.setFixedWidth(150*2)
        self.prev_button_set.setStyleSheet("background-color: #3498db; color: white; font-weight: bold;")
        self.prev_button_set.clicked.connect(self.show_previous_image_set)
        self.next_button_set = QPushButton("Next Set")
        self.next_button_set.setFixedWidth(150*2)
        self.next_button_set.setStyleSheet("background-color: #3498db; color: white; font-weight: bold;")
        self.next_button_set.clicked.connect(self.show_next_image_set)
        
        image_label.mousePressEvent = lambda e, index=i: self.enlarge_image(index)

        spacer_item = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)
        self.right_layout.addItem(spacer_item)  # Add spacer to the last row

        # self.right_layout.addWidget(self.prev_button_set, len(self.image_paths) // 5 + 1, 0, 1, 1)
        # self.right_layout.addWidget(self.next_button_set, len(self.image_paths) // 5 + 1, 4, 1, 1)


        button_layout = QHBoxLayout()
        button_layout.addWidget(self.prev_button_set)
        button_layout.addWidget(self.next_button_set)

        # Create a layout for the new button and display label
        self.confirm_button = QPushButton("Confirm")
        self.confirm_button.setCheckable(True)

        self.confirm_button.clicked.connect(self.confirm_click)
        confirm_button_style = "background-color: #27ae60; color: white; font-weight: bold;"
        self.confirm_button.setStyleSheet(confirm_button_style)

        new_button_layout = QVBoxLayout()
        # new_button_layout.addWidget(self.prev_button_set)
        # new_button_layout.addWidget(self.next_button_set)
        new_button_layout.addWidget(self.confirm_button )

        # Create a separator line
        separator_line = QFrame()
        separator_line.setFrameShape(QFrame.HLine)
        separator_line.setFrameShadow(QFrame.Sunken)

        # Create a layout for the display label
        display_label_layout = QHBoxLayout()
        display_label_layout.addWidget(QLabel("New Images: "))
        display_label = QLabel('<span style="font-weight: bold; color: blue;">FORWARD</span>')
        display_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        display_label_layout.addWidget(display_label)

        # Add stretchable spacers to reduce space between buttons and images
        spacer_top = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)
        spacer_bottom = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)
        # self.right_layout.addItem(spacer_top, 5, 0, 1, 5)  # Add spacer above the buttons

        spacer_item = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)
        self.right_layout.addItem(spacer_item, len(self.image_paths) // 5, 0, 1, 5)  # Add spacer to the last row

        self.right_layout.addLayout(button_layout, 3, 0, 1, 5)  # Add button_layout to row 4
        self.right_layout.addWidget(separator_line, 4, 0, 1, 5)  # Add separator_line to row 5
        self.right_layout.addItem(spacer_top, 5, 0, 1, 5)  # Add spacer above the buttons
        self.right_layout.addLayout(new_button_layout, 6, 0, 1, 5)  # Add new_button_layout to row 6
        self.right_layout.addLayout(display_label_layout, 7, 0, 1, 5)  # Add display_label_layout to row 7
        # self.right_layout.addItem(spacer_bottom, 8, 0, 1, 5)  # Add spacer below the buttons




        # self.right_layout.addWidget(self.prev_button_set, 2, 0, 1, 5)
        # self.right_layout.addWidget(self.next_button_set, 3, 0, 1, 5)

        splitter.addWidget(left_column)
        splitter.addWidget(separator)
        splitter.addWidget(right_column)

        # Set the handle width for the separator
        splitter.setHandleWidth(1)

        # Set the splitter as the central widget's layout
        central_layout = QVBoxLayout(central_widget)
        central_layout.addWidget(splitter)
        self.fit_to_screen()
        # Inside the initUI method of InitialGUI class
        self.check_matches_button = QPushButton("Check Matches")
        self.check_matches_button.clicked.connect(self.checkMatches)
        left_layout.addWidget(self.check_matches_button)

        # Create a new method to open a window with the current image

    def load_images(self, image_path, image_list, start_index, end_index):
        self.load_thread = LoadImageThread(image_path, image_list, start_index, end_index)
        self.load_thread.loaded.connect(self.images_loaded)
        self.load_thread.start()

    def images_loaded(self, loaded_images):
        if loaded_images:
            if self.current_load == "ref":
                print("Loaded ref images")
                self.buffer_ref_image_list = loaded_images
                self.ref_loading = False
            if self.current_load == "query":
                self.buffer_query_image_list = loaded_images
                self.query_loading = False
            print(f"Loaded {len(loaded_images)} images and saved.")
        else:
            print("Failed to load images.")

    def load_numpy_array(self, load_path):
        self.load_thread = LoadNumpyArrayThread(load_path)
        self.load_thread.loaded.connect(self.array_loaded)
        self.load_thread.start()

    def array_loaded(self, loaded_array):
        if loaded_array is not None:
            self.loaded_arrays.append(loaded_array)
            print("Array loaded and saved.")
        else:
            print("Failed to load array.")

    def checkMatches(self,index):
        if hasattr(self, 'current_image_window') and self.current_image_window is not None:
            self.current_image_window.close()
        image_path = f"{self.ref_image_path}/{self.single_image_paths[self.current_single_image_index]}"
        enlarged_window = EnlargedImageWindow(self.tagged_ref_image,self.tagged_query_image, self.query_image_path, self.ref_image_path)
        # enlarged_window.show()
        enlarged_window.exec_()


    def fit_to_screen(self):
        desktop = QDesktopWidget()
        screen_rect = desktop.availableGeometry(desktop.primaryScreen())
        self.setGeometry(screen_rect)
        self.showMaximized()

    def display_image(self, image_path):
        a =1 
        # self.enlarged_window.show_image(image_path)

    def load_new_ref_image(self,index):
        for i in range(100):
            ref_img = cv2.imread(f"{self.ref_image_path}/{self.single_image_paths[index+i]}")
            print(f"{self.ref_image_path}/{self.single_image_paths[index+i]}")
            self.buffer_ref_image_list.append(ref_img)

    def load_new_query_image(self,index):
        print("Replenishing")
        for i in range(100):
            query_img = cv2.imread(f"{self.query_image_path}/{self.image_paths[index+i]}")
            print(f"{self.query_image_path}/{self.image_paths[index+i]}")
            self.buffer_query_image_list.append(query_img)

    def show_image(self, image_path):
        # Load the image and create a pixmap
        pixmap = QPixmap(image_path)

        # Set the pixmap to the label
        self.image_label.setPixmap(pixmap)

        # Set the dialog size based on the image size
        self.setWindowTitle("Enlarged Image")
        self.resize(500, 500)
        self.exec_()


    def confirm_click(self):
        check = False
        saving_dict  =dict()

        index = self.clicked_image_index

        for i in range(len(self.select_button)):
            if self.select_button[i].isChecked():
                check = True
                break


        if check:
            self.match_dict[self.current_single_image_index] = self.current_set_start_index + index
            self.single_image_label.setStyleSheet("border: 5px solid black; padding: 10px;")
            self.image_labels[self.current_query_clicked_idx].setStyleSheet("border: 5px solid red; padding: 2px;")
            self.query_selection_idx = self.current_set_start_index + self.clicked_image_index
            self.last_selected_ref_idx =  self.current_single_image_index

            self.tagged_ref_image.append(self.current_single_image_index)
            self.tagged_query_image.append(self.current_set_start_index + index)
          
            # self.match_dict[self.current_single_image_index] = self.current_set_start_index
            self.query_clicked_list.append(self.current_set_start_index)
        else:
            self.confirm_button.setChecked(False)
            QMessageBox.critical(self, "Error", "First choose the image on the right and then click on confirm")
        # valid_check = self.toggle_box(index)


        # if valid_check:
        #     self.last_selected_ref_idx = self.current_single_image_index
        #     self.last_query_selected_idx = self.current_set_start_index

        # ref_coord = self.ref_route_data.iloc[self.current_single_image_index]['coords']
        # query_coord = self.query_route_data.iloc[self.current_set_start_index+index]['coords']
        # dist_check = self.check_coord_dist(ref_coord, query_coord)
        dist_check = True # Replace it with actual distance check
        
        if dist_check and check:
            selected_image = self.image_paths[self.clicked_image_index]
            self.tem_dict["route"] = self.current_route
            self.tem_dict["direction"] = self.current_direction
            self.tem_dict["ref_images"]["cam0"] = self.ref_route_data.iloc[self.current_single_image_index]["cam0"]
            self.tem_dict["ref_images"]["cam1"] = self.ref_route_data.iloc[self.current_single_image_index]["cam1"]
            self.tem_dict["ref_images"]["cam2"] = self.ref_route_data.iloc[self.current_single_image_index]["cam2"]
            self.tem_dict["ref_images"]["cam3"] = self.ref_route_data.iloc[self.current_single_image_index]["cam3"]
            self.tem_dict["ref_images"]["cam4"] = self.ref_route_data.iloc[self.current_single_image_index]["cam4"]
            self.tem_dict["ref_images"]["cam5"] = self.ref_route_data.iloc[self.current_single_image_index]["cam5"]
            self.tem_dict["query_images"]["cam0"] = self.query_route_data.iloc[self.current_set_start_index+index]["cam0"]
            self.tem_dict["query_images"]["cam1"] = self.query_route_data.iloc[self.current_set_start_index+index]["cam1"]
            self.tem_dict["query_images"]["cam2"] = self.query_route_data.iloc[self.current_set_start_index+index]["cam2"]
            self.tem_dict["query_images"]["cam3"] = self.query_route_data.iloc[self.current_set_start_index+index]["cam3"]
            self.tem_dict["query_images"]["cam4"] = self.query_route_data.iloc[self.current_set_start_index+index]["cam4"]
            self.tem_dict["query_images"]["cam5"] = self.query_route_data.iloc[self.current_set_start_index+index]["cam5"]
            self.tem_dict["coordinate"] = self.ref_route_data.iloc[self.current_single_image_index]['coords']

            self.json_data_list.append(self.tem_dict)
            print(f"Selected dict: {self.tem_dict}")
            with open(self.annotation_path, 'w+') as fout:
                json.dump(self.json_data_list, fout)

            saving_dict['match_dict'] = self.match_dict
            saving_dict['last_ref_image'] = self.current_single_image_index
            saving_dict['last_query_image'] = self.current_set_start_index
            saving_dict['check_last_ref_image'] = self.last_selected_ref_idx
            saving_dict['check_last_query_image'] = self.last_query_selected_idx

            with open(self.saved_annotations_file,'w+') as file:
                json.dump(saving_dict, file)

        else:
            QMessageBox.critical(self, "Error", "Invalid image! The distance is too much try selecting a different image")

    def numpy_array_to_qpixmap(self, image):
        if image is not None:
            height, width, channel = image.shape
            bytes_per_line = 3 * width
            return QPixmap(QImage(image.data, width, height, bytes_per_line, QImage.Format_RGB888))
        return QPixmap()

    def load_single_image(self):
        # Load and display the current single image
        if 0 <= self.current_single_image_index < len(self.single_image_paths):
            single_image_path = f"{self.ref_image_path}/{self.single_image_paths[self.current_single_image_index]}"

            # Load the image into a numpy array
            image_array = cv2.imread(single_image_path)

            # Display the image in the GUI
            pixmap = self.numpy_array_to_qpixmap(image_array)
            pixmap = pixmap.scaled(500, 500)

            self.single_image_label.setPixmap(pixmap)

            print(f"The current single image index is {self.current_single_image_index}")
            if self.current_single_image_index == self.last_selected_ref_idx:
                self.single_image_label.setStyleSheet("border: 5px solid black; padding: 2px;")
                self.selected_image_label.show()
            else:
                self.single_image_label.setStyleSheet("")
                self.selected_image_label.hide()

    def show_previous_single_image(self):
        # Show the previous single image
        if self.current_single_image_index <= self.last_selected_ref_idx:
            QMessageBox.critical(self, "Error", "Cannot go behind the last selected REF IMAGE choose from Next images")
        else:
            self.current_single_image_index -= 10
            if self.current_single_image_index < 0:
                print("Setting the single index to 0")
                self.current_single_image_index = 0
            self.load_single_image()

    def show_next_single_image(self):
        # Show the next single image
        self.confirm_button.setChecked(False)

        # if (self.current_single_image_index+10)%100 == 0:
        #     if len(self.buffer_ref_image_list) != 0 and  not self.ref_loading:
        #         self.ref_images_np_list = self.buffer_ref_image_list[:]
        #         self.buffer_ref_image_list = []
        #         self.current_load = "ref"
        #         self.ref_loading = True
        #         self.load_images(self.ref_image_path, self.single_image_paths, self.current_single_image_index+100, self.current_single_image_index+200)
        #         # ref_thread = threading.Thread(target=self.load_new_ref_image, args=(self.current_single_image_index,))
        #         # ref_thread.start()
        #         # ref_thread.join()
        #         self.current_set_start_index += 10
        #         self.load_single_image()


        #     else:
        #         QMessageBox.critical(self, "Wait", "Loading the reference image")

        # else:
        #     if self.current_single_image_index in self.match_dict or True:
        #         self.current_single_image_index += 10

                # Check if the index is within bounds
        if self.current_single_image_index >= len(self.single_image_paths):
            self.current_single_image_index = len(self.single_image_paths) - 10
        self.current_single_image_index += 10
        self.load_single_image()    
        # else:
        #     QMessageBox.critical(self, "Error", "First choose the current match and then click on confirm")


    def load_image_set(self):
        for i in range(10):
            image = self.load_image(i)
            pixmap = self.numpy_array_to_qpixmap(image)
            pixmap = pixmap.scaled(250, 300)
            self.image_labels[i].setPixmap(pixmap)
            self.select_button[i].setChecked(False)
            self.select_button[i].setStyleSheet(self.select_button_style)


    def load_image(self, index):
        query_list = self.query_images_np_list[:]
        print(f"The set start index is {self.current_set_start_index}")
        if 0 <= self.current_set_start_index + index < len(self.image_paths):
            image_path = f"{self.query_image_path}/{self.image_paths[self.current_set_start_index + index]}"
            print(f"The loaded index is {(self.current_set_start_index+index)}")
            image = cv2.imread(image_path)
            if image is not None:
                return image
        return np.zeros((500, 500, 3), dtype=np.uint8)  # A black image if loading fails

    def show_previous_image_set(self):
        # Show the previous set of 10 images
        if self.current_set_start_index < self.last_query_selected_idx:
            QMessageBox.critical(self, "Error", "Cannot go behind the last selected QUERY IMAGE choose from Next images")
        else:
            print("Setting the set index to 0")
            self.current_set_start_index -= 10
            if self.current_set_start_index < 0:
                print("Setting the set index to 0")
                self.current_set_start_index = 0
    
            # if self.query_clicked_list and self.current_set_start_index > self.query_clicked_list[-1] - 30:
            #     QMessageBox.critical(self, "Error", "Gone too far BEHIND cannot select images from here")
            self.load_image_set()

    def show_next_image_set(self):
        # Show the next set of 10 images



        self.current_set_start_index += 10
        if self.current_set_start_index > len(self.image_paths):
            print("Setting the set index to 0")
            self.current_set_start_index -= 10

        self.load_image_set()

        # query_thread = threading.Thread(target=self.load_new_query_image, args=(self.current_set_start_index,))
        # query_thread.start()
        # query_thread.join()
        
        self.image_labels[self.current_query_clicked_idx].setStyleSheet("")

        # if self.query_clicked_list and self.current_set_start_index > self.query_clicked_list[-1] + 30:
        #     QMessageBox.critical(self, "Error", "Gone too far AHEAD cannot select images from here")
        # for i in range(len(self.query_clicked_list), 0):
        #     if self.current_set_start_index <= self.query_clicked_list[i]:
        #     query_set = i
        #     break
        # query_label = QLabel(f"Selected query set is {self.query_clicked_list[0]}")
        # self.right_layout.addWidget(query_label)
        



    def toggle_selection(self):
        # Toggle the selection state and update the border
        self.selected = not self.selected
        self.update_border()

    def update_border(self):
        # Add a blue border if selected, remove it otherwise
        border_style = "2px solid blue" if self.selected else ""
        self.setStyleSheet(f"border: {border_style};")

        # Emit a signal to notify the selection status change
        self.selection_changed.emit(self.index, self.selected)

    def toggle_selection(self):
        # Toggle the selection state and update the border
        self.selected = not self.selected
        self.update_border()

    def update_border(self):
        # Add a blue border if selected, remove it otherwise
        border_style = "2px solid blue" if self.selected else ""
        self.setStyleSheet(f"border: {border_style};")

        # Emit a signal to notify the selection status change
        self.selection_changed.emit(self.index, self.selected)

    def select_image(self, index):
        valid_check = self.toggle_box(index)
        self.clicked_image_index = index
        self.display_image(self.image_paths[index])

    def set_route(self, route):
        self.current_route = route
        print(f"Selected route: {route}")
        instruction_box = QMessageBox()
        instruction_box.setIcon(QMessageBox.Information)
        instruction_box.setText(f" !!!!!!!  {route} SELECTED !!!  ")
        # instruction_box.setInformativeText(" If you are sure\n 1. Click the CONFIRM button in the left.\n2. Then click on NEXT SET in this left column.\n3. Click on NEXT in the left column")
        instruction_box.setStandardButtons(QMessageBox.Ok)
        instruction_box.setDefaultButton(QMessageBox.Ok)

        # Show the instruction message box
        instruction_box.exec_()

    def set_direction(self, direction):
        self.current_direction = direction
        print(f"Selected direction: {direction}")
        dir_box = QMessageBox()
        dir_box.setIcon(QMessageBox.Information)
        dir_box.setText(f" !!!! {direction} SELECTED: !!!!  ")
        # instruction_box.setInformativeText(" If you are sure\n 1. Click the CONFIRM button in the left.\n2. Then click on NEXT SET in this left column.\n3. Click on NEXT in the left column")
        dir_box.setStandardButtons(QMessageBox.Ok)
        dir_box.setDefaultButton(QMessageBox.Ok)

        # Show the instruction message box
        dir_box.exec_()

    def check_coord_dist(self, query_coord, ref_coord):

        distance = 0.0

        R = 6371000  # meters
        q_coord = []
        r_coord = []

        q_coord.append(float(query_coord[1:-2].split(',')[0]))
        q_coord.append(float(query_coord[1:-2].split(',')[0][1:]))
        
        r_coord.append(float(ref_coord[1:-2].split(',')[0]))
        r_coord.append(float(ref_coord[1:-2].split(',')[0][1:]))        
        
        print(q_coord)
        # Convert latitude and longitude from degrees to radians
        lat1 = math.radians(float(q_coord[0]))
        lon1 = math.radians(float(q_coord[1]))
        lat2 = math.radians(float(r_coord[0]))
        lon2 = math.radians(float(r_coord[1]))

        # Haversine formula
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        # Calculate the distance
        distance = R * c


        if distance > 40:
            return False
        
        return True



    def toggle_box(self,index):

        # if self.last_selected_ref_idx == self.current_single_image_index and self.current_set_start_index > self.last_query_selected_idx:
        #     QMessageBox.critical(self, "Error", "Cannot select the last the last selected IMAGE choose from Next images")
        self.current_query_clicked_idx = index
        if (self.last_selected_ref_idx in self.match_dict and self.match_dict[self.last_selected_ref_idx] >= self.current_set_start_index):
            QMessageBox.critical(self, "Error", "Invalid select! Cannot select from or before the last selected set")
            return False
        if  (self.current_single_image_index in self.match_dict):
            QMessageBox.critical(self, "Error", "Invalid select! Cannot select the same set twice! Select next ref image")
            return False

        else:
            print(f"The selected image is of index - {index}")
            for i in range(len(self.select_button)):
                if i != index and self.select_button[i].isChecked():
                    self.select_button[i].setStyleSheet(self.select_button_style)
                    self.select_button[i].setChecked(False)
                    print(f'found a checked one at index {i}')



            if self.select_button[index].isChecked():
                # Apply a different style sheet when the button is checked (toggled)
                self.select_button[index].setChecked(True)
                self.select_button[index].setStyleSheet("border: 5px solid red; padding: 2px")
            else:
                # Apply the original style sheet when the button is unchecked
                self.select_button[index].setStyleSheet(self.select_button_style)
                self.select_button[index].setChecked(False)
            return True

    def reset_parameters(self):
        # Reset parameters to their initial values
        # self.current_route = "route1"
        # self.current_direction = "forward"
        self.current_single_image_index = 0
        self.current_set_start_index = 0
        
        self.ref_image_path = f"{self.ref_image_dir}/{self.current_route}/images/{self.current_direction}/cam3"
        self.query_image_path = f"{self.query_image_dir}/{self.current_route}/images/{self.current_direction}/cam3"
        self.image_paths = sorted(os.listdir(self.query_image_path))

        self.load_single_image()
        self.load_image_set()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    initial_gui = InitialGUI()
    initial_gui.show()
    sys.exit(app.exec_())
