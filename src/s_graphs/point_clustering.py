#!/usr/bin/python
# SPDX-License-Identifier: BSD-2-Clause
from matplotlib.pyplot import axis
from sklearn import cluster
import tf
import rospy
from geometry_msgs.msg import *
from sensor_msgs.msg import *
import numpy as np
from sklearn.cluster import AgglomerativeClustering, KMeans, DBSCAN, Birch, SpectralClustering
from sklearn.mixture import GaussianMixture
import scipy.cluster.hierarchy as sch
import ros_numpy
import sensor_msgs.point_cloud2
import matplotlib.pyplot as plt
from numpy import unique
from sklearn.neighbors import NearestCentroid


class HierarchicalClustering:
	def __init__(self):
		self.subscriber = rospy.Subscriber('/room_segmentation/cluster_cloud', PointCloud2, self.callback)
		self.cloud_array = []
		self.cluster_labels = []

	def callback(self, cloud_msg):	
		self.cloud_array = []
		for point in sensor_msgs.point_cloud2.read_points(cloud_msg, skip_nans=False):	
			pt_x = point[0]
			pt_y = point[1]
			self.cloud_array.append([pt_x, pt_y])
		#self.perform_k_means_clustering(np.array(self.cloud_array))
		self.perform_hierarchical_clustering(np.array(self.cloud_array))
		#self.perform_dbscan_clustering(np.array(self.cloud_array))		
		#self.perform_birch_clustering(self.cloud_array)
		#self.perform_spectral_clustering(self.cloud_array)
		#self.perform_gaussian_clustering(self.cloud_array)

	def get_cloud_array(self):
		return self.cloud_array

	def perform_k_means_clustering(self, cloud_array):
		model = KMeans(n_clusters=4, init='k-means++', max_iter=300, n_init=10, random_state=0)
		model.fit_predict(cloud_array)
		self.cluster_labels = model.labels_

	def perform_hierarchical_clustering(self, cloud_array):
		model = AgglomerativeClustering(n_clusters=None, affinity='euclidean', linkage='average', compute_full_tree = True, distance_threshold=2.5)
		model.fit_predict(cloud_array)
		self.cluster_labels = model.labels_

	def perform_dbscan_clustering(self, cloud_array):
		model = DBSCAN(eps=3, min_samples=2)
		model.fit_predict(cloud_array)
		self.cluster_labels = model.labels_

	def perform_birch_clustering(self, cloud_array):
		model = Birch(n_clusters=None)
		model.fit_predict(cloud_array)	
		self.cluster_labels = model.labels_	

	def perform_spectral_clustering(self, cloud_array):
		model = SpectralClustering(n_clusters=2)
		model.fit_predict(cloud_array)
		self.cluster_labels =  model.labels_

	def perform_gaussian_clustering(self, cloud_array):
		model = GaussianMixture()
		yhat  = model.fit_predict(cloud_array)
		self.cluster_labels = unique(yhat)

	def get_cluster_centroids(self, cloud_array, cloud_labels):
		clf = NearestCentroid()
		clf.fit(cloud_array, cloud_labels)
		print(clf.centroids_)
		return clf.centroids_

	def get_cluster_labels(self):
		return self.cluster_labels

if __name__ == '__main__':
	rospy.init_node('hierarchical_clustering')
	node = HierarchicalClustering()
	
	plt.title("points")
	plt.xlabel("x axis caption") 
	plt.ylabel("y axis caption") 
	plt.xlim(-10, 10)
	plt.ylim(-10, 10)
	plt.axis('equal')
	#plt.figure(figsize=(10, 7))
	rate = rospy.Rate(1.0)
	while not rospy.is_shutdown():
		cloud_array = np.array(node.get_cloud_array())
		cloud_labels = np.array(node.get_cluster_labels())
		#print('cloud_array', cloud_array.shape)
		#print('cloud_labels', cloud_labels.shape)
		plt.scatter(cloud_array[:,0],cloud_array[:,1], c=cloud_labels)
		#cloud_centroids = node.get_cluster_centroids(cloud_array, cloud_labels)
		cloud_stacked = np.hstack((cloud_array,cloud_labels[:,None]))
		cloud_stacked = cloud_stacked[cloud_stacked[:, -1].argsort()]
		cloud_stacked = np.split(cloud_stacked[:,:-1], np.unique(cloud_stacked[:, -1], return_index=True)[1][1:])
		centroids = []
		for x in cloud_stacked:
			#print("cloud stacked", np.array(x))
			centroid = np.mean(x, axis=0)
			print("centroid", centroid)
			centroids.append([centroid])
		centroids = np.array(centroids)
		print("centroids:", centroids[:,:,0])
		
		diff = np.diff(centroids[:,:,0])
		print("diff", diff)			
		plt.show()
		rate.sleep()
