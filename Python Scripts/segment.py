import numpy as np
import open3d
from sklearn.cluster import KMeans
from UliEngineering.Math.Coordinates import BoundingBox
import cv2

num_clusters = 112
pc = open3d.io.read_point_cloud('/home/user01/Data/PointCloud/6-4-2019/recentered.pcd')
cloud = np.asarray(pc.points)

filtered_cloud = cloud.transpose()
filtered_cloud = filtered_cloud[:, filtered_cloud[2] > 0.15]
filtered_cloud = filtered_cloud.transpose()

overall_filename = "/home/user01/Data/PointCloud/complete.pcd"
pcd_total = open3d.geometry.PointCloud()
pcd_total.points = open3d.utility.Vector3dVector(filtered_cloud)
open3d.io.write_point_cloud(overall_filename, pcd_total)

y = np.full(filtered_cloud.shape[0], 255)
color = np.zeros(filtered_cloud.shape)
color[:, 2] = y

kmeans = KMeans(n_clusters=num_clusters, random_state=0, n_jobs=-1).fit(filtered_cloud)

rectangle_list = []
for i in range(0, num_clusters):
    cluster_cloud = filtered_cloud.transpose()
    cluster_cloud = cluster_cloud[:, kmeans.labels_ == i]
    cluster_cloud = cluster_cloud.transpose()
    filename = "/home/user01/Data/PointCloud/Clusters/cluster_" + str(i) + ".pcd"
    pcd_cluster = open3d.geometry.PointCloud()
    pcd_cluster.points = open3d.utility.Vector3dVector(cluster_cloud)
    open3d.io.write_point_cloud(filename, pcd_cluster)
    #average = np.mean(cluster_cloud, axis=0)
    #box_points = np.delete(cluster_cloud, 2, 1)
    #box_points = np.float32(box_points)
    #box = BoundingBox(box_points)
    #rect = cv2.minAreaRect(box_points)
    #points = [[box.center[0] - box.width/2, box.center[1] - box.height/2, average[2]], [box.center[0] - box.width/2, box.center[1] + box.height/2, average[2]],
    #          [box.center[0] + box.width/2, box.center[1] - box.height/2, average[2]], [box.center[0] + box.width/2, box.center[1] + box.height/2, average[2]]]
    #lines = [[0, 1], [0, 2], [1, 3], [2, 3]]
    #colors = [[1, 0, 0] for i in range(len(lines))]
    #line_set = open3d.geometry.LineSet()
    #line_set.points = open3d.utility.Vector3dVector(points)
    #line_set.lines = open3d.utility.Vector2iVector(lines)
    #line_set.colors = open3d.utility.Vector3dVector(colors)
    #rectangle_list.append(line_set)

# test_cloud = filtered_cloud[kmeans.labels_ == 100]

centers = np.asarray(kmeans.cluster_centers_)

cluster_colors = np.zeros(centers.shape)
x = np.full(num_clusters, 255);
cluster_colors[:, 0] = x

filtered_cloud = np.concatenate((filtered_cloud, centers))
color = np.concatenate((color, cluster_colors))

pcd_filtered = open3d.geometry.PointCloud()
pcd_filtered.points = open3d.utility.Vector3dVector(filtered_cloud)
pcd_filtered.colors = open3d.utility.Vector3dVector(color)



vis = open3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(pcd_filtered)
#for i in range(0, num_clusters):
#    vis.add_geometry(rectangle_list[i])
vis.run()
vis.destroy_window()
