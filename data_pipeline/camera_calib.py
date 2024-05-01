# import numpy as np
# import cv2 as cv
# import glob
# # termination criteria
# criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
# objp = np.zeros((6*7,3), np.float32)
# objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
# # Arrays to store object points and image points from all the images.
# objpoints = [] # 3d point in real world space
# imgpoints = [] # 2d points in image plane.

# images = glob.glob('/home/ur10/annotation_pipeline/saved_images/*.png')

# for fname in images:
#     print(fname)
#     img = cv.imread(fname)
#     gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
#     # Find the chess board corners
#     ret, corners = cv.findChessboardCorners(gray, (7,6), None)
#     # If found, add object points, image points (after refining them)
#     if ret == True:
#         objpoints.append(objp)
#         corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
#         imgpoints.append(corners2)
#         # Draw and display the corners
#         cv.drawChessboardCorners(img, (7,6), corners2, ret)
#         cv.imshow('img', img)
#         cv.waitKey(500)
#     else:
#         print(ret)
# cv.destroyAllWindows()

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Generate some sample LiDAR points (replace this with your actual data)
lidar_points = np.random.rand(1000, 3) * 100  # 1000 points in a 100x100x100 box

# Read the image
image = plt.imread('/home/ur10/poorquery.jpg')  # Replace 'your_image.jpg' with the path to your image

# Define the transformation matrix (replace this with your actual transformation)
# This is a placeholder transformation, you need to replace it with the actual transformation matrix
transformation_matrix = np.eye(3)  

# Transform LiDAR points to the image coordinate system
lidar_points_transformed = np.dot(lidar_points, transformation_matrix.T)

# Project LiDAR points onto the image (assuming the image is in pixel coordinates)
lidar_points_pixels = lidar_points_transformed[:, :2].astype(int)

# Create a figure and plot the image
plt.figure()
plt.imshow(image)

# Plot LiDAR points on top of the image
plt.scatter(lidar_points_pixels[:, 0], lidar_points_pixels[:, 1], c='r', s=1)

plt.show()
