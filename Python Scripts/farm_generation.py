from scipy.spatial import Delaunay
import numpy as np
import matplotlib.pyplot as plt
from sympy import Plane, Point3D
import math
from collada import *
from random import seed
from random import random
from random import randint
from math import pi
import xml.etree.ElementTree as ET

seed(1)


def sign(x1, y1, x2, y2, x3, y3):
    return (x1 - x3) * (y2 - y3) - (x2 - x3) * (y1 - y3)


def isInside(x1, y1, x2, y2, x3, y3, x, y):

    d1 = sign(x, y, x1, y1, x2, y2)
    d2 = sign(x, y, x2, y2, x3, y3)
    d3 = sign(x, y, x3, y3, x1, y1)

    has_neg = (d1 < 0) or (d2 < 0) or (d3 < 0)
    has_pos = (d1 > 0) or (d2 > 0) or (d3 > 0)

    return not(has_neg and has_pos)


height_variation = 0
grid_size = 0
num_of_vertices = 0

ground_param_file = open('ground_params.txt', 'r')
lines = ground_param_file.readlines()

count = 0
for line in lines:
    if count == 0:
        grid_size = int(line)
    elif count == 1:
        num_of_vertices = int(line)
    elif count == 2:
        height_variation = float(line)
    count = count + 1

print('Generating 2D Delaunay triangulation')
points = np.random.random([num_of_vertices, 2])
end_vertices = np.array([[-0.5 * grid_size, -0.5 * grid_size], [-0.5 * grid_size, 0.5 * grid_size], [0.5 * grid_size, -0.5 * grid_size], [0.5 * grid_size, 0.5 * grid_size]])
points = points * grid_size
points = points - (grid_size / 2)
points = np.append(points, end_vertices, axis=0)
tri = Delaunay(points)

indices = np.asarray(tri.simplices)

#plt.triplot(points[:,0], points[:,1], tri.simplices)
#plt.plot(points[:,0], points[:,1], 'o')
#plt.show()

print('Randomizing Z values')
points_3d = np.random.random((points.shape[0], 3))
points_3d = (-1 * height_variation / 2) + (points_3d * height_variation)

points_3d[:,0] = points[:,0]
points_3d[:,1] = points[:,1]

all_normals = np.zeros((1,3))

count = 0

print('Computing Normal vectors')
for item in tri.simplices:
    point1 = points_3d[item[0]]
    point2 = points_3d[item[1]]
    point3 = points_3d[item[2]]
    triangle = Plane(Point3D(point1[0], point1[1], point1[2]), Point3D(point2[0], point2[1], point2[2]), Point3D(point3[0], point3[1], point3[2]))
    normal = np.zeros((1,3))
    normal[0][0] = float(triangle.normal_vector[0])
    normal[0][1] = float(triangle.normal_vector[1])
    normal[0][2] = float(triangle.normal_vector[2])

    mag = (normal[0][0] * normal[0][0]) + (normal[0][1] * normal[0][1]) + (normal[0][2] * normal[0][2])
    mag = math.sqrt(mag)
    normal[0][0] = normal[0][0] / mag
    normal[0][1] = normal[0][1] / mag
    normal[0][2] = normal[0][2] / mag

    if count == 0:
        count = count + 1
        all_normals[0,:] = normal
    else:
        all_normals = np.append(all_normals, normal, axis=0)

reformed_ind = np.zeros((all_normals.shape[0], 3, 2))

for i in range(0, all_normals.shape[0]):
    reformed_ind[i][0][0] = indices[i][0]
    reformed_ind[i][1][0] = indices[i][1]
    reformed_ind[i][2][0] = indices[i][2]
    reformed_ind[i][0][1] = 4 * i
    reformed_ind[i][1][1] = (4 * i) + 1
    reformed_ind[i][2][1] = (4 * i) + 2

normals_4 = np.zeros((all_normals.shape[0] * 4, all_normals.shape[1]))
for i in range(0, all_normals.shape[0]):
    normals_4[4 * i] = all_normals[i]
    normals_4[(4*i) + 1] = all_normals[i]
    normals_4[(4*i) + 2] = all_normals[i]
    normals_4[(4*i) + 3] = all_normals[i]

points_3d_ravel = np.ravel(points_3d, order='C')
normals_4_ravel = np.ravel(normals_4, order='C')
reformed_ind_ravel = np.ravel(reformed_ind, order='C')
reformed_ind_ravel = reformed_ind_ravel.astype(int)

print('Creating ground Collada/.dae model')
mesh = Collada()
effect = material.Effect("effect0", [], "phong", diffuse=(1,1,1), specular=(0,1,0))
mat = material.Material("material0", "mymaterial", effect)
mesh.effects.append(effect)
mesh.materials.append(mat)

vert_src = source.FloatSource("verts-array", np.array(points_3d_ravel), ('X', 'Y', 'Z'))
normal_src = source.FloatSource("normals-array", np.array(normals_4_ravel), ('X', 'Y', 'Z'))

geom = geometry.Geometry(mesh, "geometry0", "myground", [vert_src, normal_src])

input_list = source.InputList()
input_list.addInput(0, 'VERTEX', "#verts-array")
input_list.addInput(1, 'NORMAL', "#normals-array")

triset = geom.createTriangleSet(reformed_ind_ravel, input_list, "materialref")

geom.primitives.append(triset)
mesh.geometries.append(geom)

matnode = scene.MaterialNode("materialref", mat, inputs=[])
geomnode = scene.GeometryNode(geom, [matnode])
node = scene.Node("node0", children=[geomnode])

myscene = scene.Scene("myscene", [node])
mesh.scenes.append(myscene)
mesh.scene = myscene

mesh.write('generated_ground.dae')

print(mesh)

indices = indices.astype(int)
plot_width = 0
plot_length = 0
row_width = 0
row_length = 0
plants_per_plot = 0
low_scale = 0
high_scale = 0
row_params = []
model_locations = []
model_heights = []

param_file = open('params.txt', 'r')
lines = param_file.readlines()

count = 0
for line in lines:
    if count == 0:
        items = line.split()
        count2 = 0
        for item in items:
            if count2 == 0:
                plot_width = float(item)
            elif count2 == 1:
                plot_length = float(item)
            count2 = count2 + 1
    elif count == 1:
        items = line.split()
        count2 = 0
        for item in items:
            if count2 == 0:
                row_width = float(item)
            elif count2 == 1:
                row_length = float(item)
            count2 = count2 + 1
    elif count == 2:
        plants_per_plot = int(line)
    elif count == 3:
        items = line.split()
        count2 = 0
        for item in items:
            if count2 == 0:
                low_scale = float(item)
            elif count2 == 1:
                high_scale = float(item)
            count2 = count2 + 1
    else:
        row_params.append(line)
    count = count + 1

model_location_file = open('models.txt', 'r')
lines = model_location_file.readlines()
for line in lines:
    model_locations.append(line.rstrip('\n'))

model_heights_file = open("3D_Models/height.txt", "r")
lines = model_heights_file.readlines()
for line in lines:
    model_heights.append(float(line))

tree = ET.parse('empty.world')
root = tree.getroot()
model = ET.parse('soy.model')
model_root = model.getroot()

x_origin = 0
y_origin = 0

print('Generating soy models')
sim_heights_file = open("simulated_heights.csv", "w")
count = 1
y_count = 1
for item in row_params:
    params = item.split()

    num_of_plots = int(params[0])
    row_offset = float(params[1])
    x_origin = row_offset
    for i in range(0, num_of_plots):
        heights = ""
        for j in range(0, plants_per_plot):
            x_val = (random() * plot_width) + x_origin
            y_val = (random() * plot_length) + y_origin
            z_val = 0.0
            for k in range(0, indices.shape[0]):
                x1 = points_3d[indices[k][0]][0]
                y1 = points_3d[indices[k][0]][1]
                x2 = points_3d[indices[k][1]][0]
                y2 = points_3d[indices[k][1]][1]
                x3 = points_3d[indices[k][2]][0]
                y3 = points_3d[indices[k][2]][1]
                z1 = points_3d[indices[k][0]][2]
                z2 = points_3d[indices[k][1]][2]
                z3 = points_3d[indices[k][2]][2]

                if isInside(x1, y1, x2, y2, x3, y3, x_val, y_val):
                    normal = all_normals[k]
                    d = (normal[0] * x1) + (normal[1] * y1) + (normal[2] * z1)
                    z_val = (d - (normal[0] * x_val) - (normal[1] * y_val)) / normal[2]
                    #print(z_val > 0.0)
                    #print(str(z1) + " " + str(z2) + " " + str(z3) + " " + str(z_val))
                    break

            x_scale = low_scale + (random() * (high_scale - low_scale))
            y_scale = low_scale + (random() * (high_scale - low_scale))
            z_scale = low_scale + (random() * (high_scale - low_scale))
            yaw = random() * (2 * pi)
            model_num = randint(0, 4)
            model_file = model_locations[model_num]
            pose_val = str(x_val) + " " + str(y_val) + " " + str(z_val) + " 0 0 " + str(yaw)
            scale_val = str(x_scale) + " " + str(y_scale) + " " + str(z_scale)
            soy_name = "soybean_" + str(count)
            link_name = "soybean_" + str(count) + "_link"
            col_name = "soybean_" + str(count) + "_collision"
            heights = heights + str(model_heights[model_num] * z_scale) + ","
            for model_name in model_root.iter('model'):
                model_name.set('name', soy_name)
            for model_name in model_root.iter('link'):
                model_name.set('name', link_name)
            for pose in model_root.iter('pose'):
                pose.text = pose_val
            for model_name in model_root.iter('collision'):
                model_name.set('name', col_name)
            for model_name in model_root.iter('visual'):
                model_name.set('name', soy_name)
            for scale_name in model_root.iter('scale'):
                scale_name.text = scale_val
            for model_location in model_root.iter('uri'):
                model_location.text = model_file
            for world in root.findall('world'):
                world.append(model_root)
            #print(str(x_val) + " " + str(y_val))
            count = count + 1
            tree.write('soy_row.world')
            tree = ET.parse('soy_row.world')
            root = tree.getroot()
        x_origin = x_origin + plot_width + row_width
        heights = heights[:-1] + "\n"
        sim_heights_file.write(heights)
    x_origin = 0
    y_origin = y_origin - plot_length - row_length
    y_count = y_count + 1

ground_param_file.close()
param_file.close()
model_location_file.close()
