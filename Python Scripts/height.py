import numpy as np
import open3d
import os
import math

folder_path = "/home/user01/Data/CorrectCluster/"

pc_gp = open3d.io.read_point_cloud('/home/user01/Data/PointCloud/ground_plane.pcd')
ground_plane = np.asarray(pc_gp.points)

centroid = np.mean(ground_plane, axis=0)

r = ground_plane - centroid

xx_a = np.multiply(r[:, 0], r[:, 0])
xx = np.sum(xx_a)
xy_a = np.multiply(r[:, 0], r[:, 1])
xy = np.sum(xy_a)
xz_a = np.multiply(r[:, 0], r[:, 2])
xz = np.sum(xz_a)
yy_a = np.multiply(r[:, 1], r[:, 1])
yy = np.sum(yy_a)
yz_a = np.multiply(r[:, 1], r[:, 2])
yz = np.sum(yz_a)
zz_a = np.multiply(r[:, 2], r[:, 2])
zz = np.sum(zz_a)

det_x = yy*zz - yz*yz
det_y = xx*zz - xz*xz
det_z = xx*yy - xy*xy

max = det_x
max_letter = 1

if det_y > max:
    max = det_y
    max_letter = 2
if det_z > max:
    max = det_z
    max_letter = 3

dir = np.zeros(3)
if max_letter == 1:
    dir[0] = det_x
    dir[1] = xz * yz - xy * zz
    dir[2] = xy * yz - xz * yy
elif max_letter == 2:
    dir[0] = xz*yz - xy*zz
    dir[1] = det_y
    dir[2] = xy*xz - yz*xx
elif max_letter == 3:
    dir[0] = xy * yz - xz * yy
    dir[1] = xy * xz - yz * xx
    dir[2] = det_z

length = (dir[0] * dir[0]) + (dir[1] * dir[1]) + (dir[2] * dir[2])
length = math.sqrt(length)

dir = np.divide(dir, length)

for filename in os.listdir(folder_path):
    file = folder_path + filename
    pc = open3d.io.read_point_cloud(file)
    cloud = np.asarray(pc.points)
    subtract_x = cloud[:, 0] - centroid[0]
    subtract_y = cloud[:, 1] - centroid[1]
    subtract_z = cloud[:, 2] - centroid[2]
    subtract_x = subtract_x * dir[0]
    subtract_y = subtract_y * dir[1]
    subtract_z = subtract_z * dir[2]

    temp_distance = np.add(subtract_x, subtract_y)
    temp_distance = np.add(temp_distance, subtract_z)
    temp_distance = np.absolute(temp_distance)
    max_height = np.amax(temp_distance)
    print("Max height: " + str(max_height) + " in meters.")

