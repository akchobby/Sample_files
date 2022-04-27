import rosbag
import rospy
import numpy as np
import cv2
import sensor_msgs.point_cloud2 as pc2
import subprocess
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from matplotlib.widgets import Slider, Button
from cv_bridge import CvBridge
import random
import math


# a class to calibrate lidar and camera for sensor fusion
# it takes the view of a pattern seen by both sensors and calculates the rotation and transformation
# the camera instrinsics need to be calculated seperately
# You need to select the pattern in the lidar view manually


class Lidar_camera_calibrator:
	

	#initalize list of arrays for the 3d points used in the final calibration
	pattern_points_lidar = []
	pattern_points_camera = []

	#initalize bridge to convert ros image messages
	bridge = CvBridge()
	
	#the bag for calibration
	#it should contain the same number of lidar messages as camera messages (2 separate topics)
	calibration_bag = rosbag.Bag("output.bag")
	
	#initialize list of lidar and image messages
	lidar_msgs = []
	img_msgs = []

	#starting values for filtering out the lidar data
	LIDAR_X_MAX = 6.0
	LIDAR_X_MIN = 2.0
	LIDAR_Y_MAX = 2.0
	LIDAR_Y_MIN = -2.0
	LIDAR_Z_MAX = 2.0
	LIDAR_Z_MIN = -0.5
	
	#save the defined shape of the checkerboard pattern to be used by openCV
	CHECKERBOARD = (8,11)
	criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
	pattern = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
	pattern[:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
	pattern = pattern * 0.06
	
	
	#camera intrinsics
	camera_matrix = np.array([2339.351070968967, 0.0, 1240.6027516400636, 0.0, 2341.6423659053853, 1014.1489624283051, 0.0, 0.0, 1.0]).reshape(3,3)
                			
	dist_coeffs = np.array([-0.0993994320099732, 0.11792419440007315, -0.00022882594401172615, 0.000506925765611351, 0.0])


	#initialization of value to remember on which view we are on (which lidar message)
	lidar_index = 0
	
	#initalize lidar points to filter
	lidar_points = None
	

	def create_plot(self):
		#create plot for showing 3d points
		#start by giving the dimensions of the plot
		X_MAX = 8
		Y_MAX = 2
		Z_MAX = 2
		X_MIN = 2
		Y_MIN = -2
		Z_MIN = -1
		
		#make plot 3d
		self.fig = plt.figure(1)
		self.ax = self.fig.add_subplot(projection='3d')
		self.ax.set_title("Lidar Overview")
		
		#sex axis limits
		self.ax.set_xlim([X_MIN, X_MAX])
		self.ax.set_ylim([Y_MIN, Y_MAX])
		self.ax.set_zlim([Z_MIN, Z_MAX])
		
		#random settings
		self.ax.set_autoscale_on(False)
		self.sct = None
		
		#make room for sliders
		plt.subplots_adjust(left=0.5, bottom=0)
		
	
	#filter lidar points based on given limits in sliders
	
	def filter_lidar_points(self):
		lidar_points_filtered = np.zeros((0,3), dtype = np.float)
		for lpoint in self.lidar_points:
			if lpoint[0] < self.LIDAR_X_MAX and lpoint[0] > self.LIDAR_X_MIN and lpoint[1] < self.LIDAR_Y_MAX and lpoint[1] > self.LIDAR_Y_MIN and lpoint[2] < self.LIDAR_Z_MAX and lpoint[2] > self.LIDAR_Z_MIN:
				lidar_points_filtered = np.concatenate((lidar_points_filtered, lpoint.reshape((1,3))), 0)
		return lidar_points_filtered	


	#calc scatter plot

	def scatter_points(self, points_to_scatter):
		if self.sct is not None:
			self.sct.remove()
		self.sct = self.ax.scatter(points_to_scatter[:,0], points_to_scatter[:,1], points_to_scatter[:,2], s=0.2, c="b")
		
		
	#redraw plot with new points
			
	def redraw_plt(self):
		lidar_points_filtered = self.filter_lidar_points()
		self.scatter_points(lidar_points_filtered)
		self.fig.canvas.draw_idle()
		
	
	#functions to define what happens when adjusting the sliders
	
	def on_change_x_max(self, value):
		self.LIDAR_X_MAX=value
		self.redraw_plt()
	
	def on_change_x_min(self, value):
		self.LIDAR_X_MIN=value
		self.redraw_plt()
	
	def on_change_y_max(self, value):
		self.LIDAR_Y_MAX=value
		self.redraw_plt()
	
	def on_change_y_min(self, value):
		self.LIDAR_Y_MIN=value
		self.redraw_plt()
	
	def on_change_z_max(self, value):
		self.LIDAR_Z_MAX=value
		self.redraw_plt()
	
	def on_change_z_min(self, value):
		self.LIDAR_Z_MIN=value
		self.redraw_plt()
	
		
	#initialize the matplotlib sliders
		
	def init_sliders(self):

		#define slider positions
		axmaxx = plt.axes([0.05, 0.1, 0.02, 0.8])
		axminx = plt.axes([0.1, 0.1, 0.02, 0.8])
		axmaxy = plt.axes([0.15, 0.1, 0.02, 0.8])
		axminy = plt.axes([0.20, 0.1, 0.02, 0.8])
		axmaxz = plt.axes([0.25, 0.1, 0.02, 0.8])
		axminz = plt.axes([0.30, 0.1, 0.02, 0.8])

		#define sliders
		self.xmax_slider = Slider(ax=axmaxx, label="max x", valmin=-10, valmax=10, valinit=self.LIDAR_X_MAX, orientation="vertical")
		self.xmin_slider = Slider(ax=axminx, label="min x", valmin=-10, valmax=10, valinit=self.LIDAR_X_MIN, orientation="vertical")
		self.ymax_slider = Slider(ax=axmaxy, label="max y", valmin=-10, valmax=10, valinit=self.LIDAR_Y_MAX, orientation="vertical")
		self.ymin_slider = Slider(ax=axminy, label="min y", valmin=-10, valmax=10, valinit=self.LIDAR_Y_MIN, orientation="vertical")
		self.zmax_slider = Slider(ax=axmaxz, label="max z", valmin=-10, valmax=10, valinit=self.LIDAR_Z_MAX, orientation="vertical")
		self.zmin_slider = Slider(ax=axminz, label="min z", valmin=-10, valmax=10, valinit=self.LIDAR_Z_MIN, orientation="vertical")
		
		#link slider functions
		self.xmax_slider.on_changed(self.on_change_x_max)
		self.xmin_slider.on_changed(self.on_change_x_min)
		self.ymax_slider.on_changed(self.on_change_y_max)
		self.ymin_slider.on_changed(self.on_change_y_min)
		self.zmax_slider.on_changed(self.on_change_z_max)
		self.zmin_slider.on_changed(self.on_change_z_min)

	
	#read in the next set of lidar points (new pattern view)
	def next_lidar_points(self, event):
		self.pattern_points_lidar.append(self.filter_lidar_points())
		self.lidar_index += 1
		if self.lidar_index >= len(self.lidar_msgs):
			plt.close()
			return
		self.lidar_points = np.array([[p[0],p[1],p[2]] for p in pc2.read_points(self.lidar_msgs[self.lidar_index], field_names=("x", "y", "z"), skip_nans=True)], dtype=np.float64)
		self.redraw_plt()
	
		
	#initialize next lidar points button
	def init_buttons(self):
		axnext = plt.axes([0.35, 0.1, 0.1, 0.07])
		self.bnext = Button(axnext, 'next')
		self.bnext.on_clicked(self.next_lidar_points)
		
	
	#draw on an image the projected 3D local coordinate system of the pattern based on its rotation/transformation in the camera coordinate system
	def draw_board_axis(self, board_rot, board_trans, img):
		point_3d_o = np.array([0,0,0])
		point_3d_x = np.array([1,0,0])
		point_3d_y = np.array([0,1,0])
		point_3d_z = np.array([0,0,-1])
	
		axis_points = np.array(point_3d_o[:3]).reshape((1,3)).astype(np.float)
		axis_points = np.concatenate((axis_points, point_3d_x[:3].reshape((1,3))), 0)
		axis_points = np.concatenate((axis_points, point_3d_y[:3].reshape((1,3))), 0)
		axis_points = np.concatenate((axis_points, point_3d_z[:3].reshape((1,3))), 0)
		axis_points_p, jacobian = cv2.projectPoints(axis_points, board_rot, board_trans, self.camera_matrix, self.dist_coeffs)
		img = cv2.line(img, (int(axis_points_p[0][0][0]), int(axis_points_p[0][0][1])), (int(axis_points_p[1][0][0]), int(axis_points_p[1][0][1])), (0,0,255), 2)
		img = cv2.line(img, (int(axis_points_p[0][0][0]), int(axis_points_p[0][0][1])), (int(axis_points_p[2][0][0]), int(axis_points_p[2][0][1])), (0,255,0), 2)
		img = cv2.line(img, (int(axis_points_p[0][0][0]), int(axis_points_p[0][0][1])), (int(axis_points_p[3][0][0]), int(axis_points_p[3][0][1])), (255,0,0), 2)
		cv2.imshow("pattern", img)
		cv2.waitKey(0)
	


	#initialization of calibration class
	def __init__(self):
		
		#read in rosbag messages to fill lidar and camera data
		for topic, msg, t in self.calibration_bag.read_messages(topics=["/os_cloud_node/points"]):
			self.lidar_msgs.append(msg)
		for topic, msg, t in self.calibration_bag.read_messages(topics=["/images"]):
			self.img_msgs.append(msg)
		
		assert len(self.lidar_msgs) == len(self.img_msgs), "bag file should contain corresponding lidar pointclouds and images"
	
		self.create_plot()
		self.init_sliders()
		self.init_buttons()
		
		
	#get the points of the pattern in the 3D coordinate system of the camera
		
	def get_camera_calibration_points(self):
		for x in range(len(self.img_msgs)):
			img = self.bridge.imgmsg_to_cv2(self.img_msgs[x])
			gray = img#cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
	
			# Find the chess board corners
			# If desired number of corners are found in the image then ret = true
			ret, corners = cv2.findChessboardCorners(gray, self.CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
			assert ret, "no chessboard found on image %d" % (x)
			
				
			
			
			
			# refining pixel coordinates for given 2d points.
			imgp = cv2.cornerSubPix(gray, corners, (3,3),(-1,-1), self.criteria)
			
			imgcopy = img.copy()
			#print(imgp.shape)
			for cpoint in imgp:
				#print(cpoint.shape)
				pt = cpoint.astype(np.int)[0]
				imgcopy = cv2.circle(imgcopy,tuple(pt), 1, (255,255,0), 2)
			
			cv2.imshow('corners', imgcopy)
			
			
			
			#find the rotation and translation of the board pattern
			retval, board_rot, board_trans = cv2.solvePnP(self.pattern, imgp, self.camera_matrix, self.dist_coeffs)
			board_rot_mx, jac = cv2.Rodrigues(board_rot)
			board_tf_matrix = np.concatenate((board_rot_mx, board_trans), 1)
			board_tf_matrix = np.concatenate((board_tf_matrix, np.array([[0,0,0,1]])), 0)
			self.draw_board_axis(board_rot, board_trans, img)
			
			#calculate the 3d location of each specific chessboard pattern point in the camera coordinate system
			ones = np.ones((self.CHECKERBOARD[0] * self.CHECKERBOARD[1],1), dtype = np.float)
			pattern_to_multiply = np.concatenate((self.pattern, ones), 1)
			projected_pattern = np.matmul(board_tf_matrix, pattern_to_multiply.T).T
			
			
			#add to the list of 3D pattern points
			self.pattern_points_camera.append(projected_pattern[:,:3])
	
	#read in the lidar 3d points
	def get_lidar_calibration_points(self):
		self.lidar_points = np.array([[p[0],p[1],p[2]] for p in pc2.read_points(self.lidar_msgs[self.lidar_index], field_names=("x", "y", "z"), skip_nans=True)], dtype=np.float64)
		#print(self.lidar_points.shape)
		self.redraw_plt()
		plt.show()
		
		
	#do ransac calculation to get the optimal normal vector of the pattern plane
	def ransac(self, pointcloud, iterations):
		best_error = 100000000
		bestnormal = np.array([0,0,0])
		for x in range (iterations):
			randompoint_1 = pointcloud[random.randint(0,pointcloud.shape[0]-1)]
			randompoint_2 = pointcloud[random.randint(0,pointcloud.shape[0]-1)]
			randompoint_3 = pointcloud[random.randint(0,pointcloud.shape[0]-1)]
			if np.array_equal(randompoint_1, randompoint_2) or np.array_equal(randompoint_2, randompoint_3) or np.array_equal(randompoint_1, randompoint_3):
				#print("oopsie")
				continue
			vector1 = randompoint_1 - randompoint_2
			vector2 = randompoint_3 - randompoint_2
			normal = np.cross(vector1, vector2)
			if normal[0] > 0:
				normal = -normal
			normal_unit = normal / np.linalg.norm(normal)
			a = normal_unit[0]
			b = normal_unit[1]
			c = normal_unit[2]
			d = -(randompoint_1[0] * a + randompoint_1[1] * b + randompoint_1[2] * c)
			total_error = 0
			for x in range (pointcloud.shape[0]):
				pt = pointcloud[x]
				top = abs(pt[0]*a + pt[1]*b + pt[2]*c + d)
				bottom = math.sqrt(a*a + b*b + c*c)
				total_error += top / bottom
			if total_error < best_error:
				best_error = total_error
				bestnormal = normal_unit
		
		return bestnormal
		
	#get the centroid of a pointcloud	
	def find_centroid(self, pointcloud):
		totalpoint = np.array([0,0,0], dtype = np.float)
		for point_3d in pointcloud:
			totalpoint += point_3d
		return totalpoint / pointcloud.shape[0]
		
	#do the calibration	
	def calibrate(self):
		self.get_camera_calibration_points()
		self.get_lidar_calibration_points()
		
		self.fig2 = plt.figure(2)
		self.ax2 = self.fig2.add_subplot(projection='3d')
		self.ax2.set_title("Lidar Overview")
		self.ax2.set_xlim([-4, 4])
		self.ax2.set_ylim([-4, 4])
		self.ax2.set_zlim([-4, 4])
		self.ax2.set_autoscale_on(False)
		self.sct2 = None
		
		
		
		camera_calibration_points = np.zeros((0,3), dtype = np.float)
		lidar_calibration_points = np.zeros((0,3), dtype = np.float)
		
		
		
		
		for x in range (len(self.img_msgs)):
			
			#determine 2 vectors on the image checkerboard plane to find its normal vector
			img_vector_1 = self.pattern_points_camera[x][87] - self.pattern_points_camera[x][82]
			img_vector_2 = self.pattern_points_camera[x][0] - self.pattern_points_camera[x][82]
			img_normal = np.cross(img_vector_1, img_vector_2)
			image_normal = img_normal / np.linalg.norm(img_normal)
			lidar_normal = self.ransac(self.pattern_points_lidar[x], 1000)
			#print(self.pattern)
			
			camera_plane_unit_vector_x = self.pattern_points_camera[x][8] - self.pattern_points_camera[x][0]
			camera_plane_unit_vector_x = camera_plane_unit_vector_x / np.linalg.norm(camera_plane_unit_vector_x)
			
			camera_plane_unit_vector_y = self.pattern_points_camera[x][7] - self.pattern_points_camera[x][0]
			camera_plane_unit_vector_y = camera_plane_unit_vector_y / np.linalg.norm(camera_plane_unit_vector_y)
			
			
			
			lidar_centroid = self.find_centroid(self.pattern_points_lidar[x])
			
			camera_centroid = self.find_centroid(self.pattern_points_camera[x])
			
			camera_calibration_points = np.concatenate((camera_calibration_points, camera_centroid.reshape((1,3))))
			camera_calibration_points= np.concatenate((camera_calibration_points, camera_centroid.reshape((1,3)) + image_normal.reshape((1,3))))
			
			lidar_calibration_points = np.concatenate((lidar_calibration_points, lidar_centroid.reshape((1,3))))
			lidar_calibration_points = np.concatenate((lidar_calibration_points, lidar_centroid.reshape((1,3)) + lidar_normal.reshape((1,3))))
			
		#print(lidar_calibration_points)
		#print(camera_calibration_points)
		
		
		
		
		
		lidar_svd_centroid = self.find_centroid(lidar_calibration_points)
		camera_svd_centroid = self.find_centroid(camera_calibration_points)
		
		H = np.matmul((lidar_calibration_points - lidar_svd_centroid).T, camera_calibration_points - camera_svd_centroid)
		#print(H)
		U, S, V = np.linalg.svd(H)
		
		R = np.matmul(V.T, U.T)
		
		#print(R)
		
		
		rotated_lidar_centroid = np.matmul(R, lidar_svd_centroid.T).T#.reshape((self.pattern_points_lidar[0].shape[0], 3))
		
		T = camera_svd_centroid - rotated_lidar_centroid
		
		if self.sct is not None:
			self.sct.remove()
			
		pointnr = 0
		for lpoints in self.pattern_points_lidar:
			rotated_lpoints = np.matmul(R, lpoints.T).T
			transformed_lpoints = rotated_lpoints + T
		
			self.sct2 = self.ax2.scatter(self.pattern_points_camera[pointnr][:,0], self.pattern_points_camera[pointnr][:,1], self.pattern_points_camera[pointnr][:,2], s=2, c="b")
			self.sct2 = self.ax2.scatter(self.pattern_points_lidar[pointnr][:,0], self.pattern_points_lidar[pointnr][:,1], self.pattern_points_lidar[pointnr][:,1], s=2, c="r")
			self.sct2 = self.ax2.scatter(transformed_lpoints[:,0], transformed_lpoints[:,1], transformed_lpoints[:,2], s=0.2, c="g")
			pointnr += 1
			
		
		print(R, T)
		self.fig2.canvas.draw()
		plt.show()
			
		

def main():
	calibrator = Lidar_camera_calibrator()
	
	calibrator.calibrate()
	
	
main()

