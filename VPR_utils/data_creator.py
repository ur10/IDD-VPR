import pandas as pd
import numpy as np
import utm

def create_ground_truth(database_file, query_file, ground_truth_path):

	coordinates_q = []
	coordinates_db = []

	query_data = pd.read_csv(query_file, sep = " ", names = ['cam0','cam1', 'cam2', 'cam3', 'cam4', 'cam5', 'coords'])
	database_data = pd.read_csv(database_file, sep = " ", names = ['cam0','cam1', 'cam2', 'cam3', 'cam4', 'cam5', 'coords'])

	coordinates_q = query_data['coords']
	coordinates_db = database_data['coords']

	for i in range(len(coordinates_q)):
		coordinates_q[i] = utm.from_latlon(coordinates_q[i][0], coordinates_q[i][1])

	for i in range(len(coordinates_db)):
		coordinates_db[i] = utm.from_latlon(coordinates_db[i][0], coordinates_db[i][1])

	distThr = 25
	np.savez(ground_truth_path, utmQ=coordinates_q, utmDb=coordinates_db, posDistThr=distThr)


def create_image_name(query_file, target_path = "", file_save_path=""):
	image_names = []

	query_data = pd.read_csv(query_file, sep = " ", names = ['cam0','cam1', 'cam2', 'cam3', 'cam4', 'cam5', 'coords'])
	image_names = query_data['cam3']
	print(image_names)

	image_names = [target_path+image_name for image_name in image_names]
	image_name_data = pd.DataFrame({'image_names':image_names})
	image_name_data.to_csv(path_or_buf= file_save_path, index=False, header=None, sep=" ")


create_image_name('queryImagesPath.txt', '/home2/ur10/', '/home/ur10/VPR_utils/image_name.csv')