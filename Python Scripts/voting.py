import csv
import numpy as np
import open3d

pc = open3d.io.read_point_cloud('/home/user01/Data/PointCloud/6-4-2019/recentered.pcd')

with open("/home/user01/Data/clusters_data.csv", 'r') as dest_f:
    data_iter = csv.reader(dest_f,
                           delimiter = ',',
                           quotechar = '"')
    data = [data for data in data_iter]

orientation_range = 0.3
width_range = 0.3
height_range = 0.5

voting_scheme_orientation = dict()
voting_scheme_width = dict()
voting_scheme_height = dict()

for line in data:
    orientation = float(line[1])
    count = 0
    while orientation > 0:
        orientation -= orientation_range
        count = count + 1
    key = count * orientation_range
    if key in voting_scheme_orientation:
        voting_scheme_orientation[key].append(float(line[1]))
    else:
        voting_scheme_orientation[key] = [float(line[1])]

    width = float(line[2])
    count = 0
    while width > 0:
        width -= width_range
        count = count + 1
    key = count * width_range
    if key in voting_scheme_width:
        voting_scheme_width[key].append(float(line[2]))
    else:
        voting_scheme_width[key] = [float(line[2])]

    height = float(line[3])
    count = 0
    while height > 0:
        height -= height_range
        count = count + 1
    key = count * height_range
    if key in voting_scheme_height:
        voting_scheme_height[key].append(float(line[3]))
    else:
        voting_scheme_height[key] = [float(line[3])]

max_orientation_key = -100.0
print("ORIENTATION")
for key, value in voting_scheme_orientation.items():
    if max_orientation_key == -100.0:
        max_orientation_key = key
    else:
        if len(voting_scheme_orientation[key]) > len(voting_scheme_orientation[max_orientation_key]):
            max_orientation_key = key
    print(key, len(value))

max_width_key = -100.0
print("\nWIDTH")
for key, value in voting_scheme_width.items():
    if max_width_key == -100.0:
        max_width_key = key
    else:
        if len(voting_scheme_width[key]) > len(voting_scheme_width[max_width_key]):
            max_width_key = key
    print(key, len(value))

max_height_key = -100.0
print("\nHEIGHT")
for key, value in voting_scheme_height.items():
    if max_height_key == -100.0:
        max_height_key = key
    else:
        if len(voting_scheme_height[key]) > len(voting_scheme_height[max_height_key]):
            max_height_key = key
    print(key, len(value))

print("\n")
print(len(voting_scheme_orientation[max_orientation_key]))
print("\n")
print(len(voting_scheme_width[max_width_key]))
print("\n")
print(len(voting_scheme_height[max_height_key]))

orientation_values = np.asarray(voting_scheme_orientation[max_orientation_key], dtype=np.float32)
width_values = np.asarray(voting_scheme_width[max_width_key], dtype=np.float32)
height_values = np.asarray(voting_scheme_height[max_height_key], dtype=np.float32)

orientation_average = np.average(orientation_values)
width_average = np.average(width_values)
height_average = np.average(height_values)

with open('/home/user01/Data/averages.csv', mode='w') as average_file:
    average_writer = csv.writer(average_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    average_writer.writerow([orientation_average, width_average, height_average])

# all_data = np.asarray(data)
#
# cloud = np.asarray(pc.points)
#
# filtered_cloud = cloud.transpose()
# filtered_cloud = filtered_cloud[:, filtered_cloud[2] > 0.15]
# filtered_cloud = filtered_cloud.transpose()
#
# pcd_filtered = open3d.geometry.PointCloud()
# pcd_filtered.points = open3d.utility.Vector3dVector(filtered_cloud)
# #pcd_filtered.colors = open3d.utility.Vector3dVector(color)
#
# vis = open3d.visualization.Visualizer()
# vis.create_window()
# vis.add_geometry(pcd_filtered)
# #for i in range(0, num_clusters):
# #    vis.add_geometry(rectangle_list[i])
# vis.run()
# vis.destroy_window()

print("\n")
