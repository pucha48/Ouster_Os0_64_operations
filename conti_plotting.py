# from os1 import OS1
# from os1.utils import xyz_points
import mayavi.mlab as mlab
import numpy as np
import pcl
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
# import open3d as o3d


# import os
# from functools import partial
import time
axes_str = ['X', 'Y', 'Z']

# import open3d
import time
# import pcl
init_sample_data = np.random.randn(65536 ,3)   # Random data

# vis = o3d.visualization.Visualizer()
# vis.create_window()
# # vis.get_render_option().load_from_json("renderoption.json")
#
# pcd = o3d.geometry.PointCloud()
# pcd.points = o3d.utility.Vector3dVector(init_sample_data)
#
# vis.add_geometry(pcd)


# line_x = []
# line_y = []
# line_z = []
# counter = 0


def roi_filter(cloud, x_roi, y_roi, z_roi):
    """
    Input Parameters:
        cloud: input point cloud
        x_roi: ROI range in X
        y_roi: ROI range in Y
        z_roi: ROI range in Z

    Output:
        ROI region filtered point cloud
    """
    clipper = cloud.make_cropbox()
    cloud_roi_filtered = pcl.PointCloud()
    xc_min, xc_max = x_roi
    yc_min, yc_max = y_roi
    zc_min, zc_max = z_roi
    clipper.set_MinMax(xc_min, yc_min, zc_min, 0, xc_max, yc_max, zc_max, 0)
    cloud_roi_filtered = clipper.filter()
    return cloud_roi_filtered


# def draw_lidar(
#     pc,
#     color=None,
#     fig=None,
#     bgcolor=(0, 0, 0),
#     pts_scale=0.3,
#     pts_mode="sphere",
#     pts_color=None,
#     color_by_intensity=False,
#     pc_label=False,
# ):
#     """ Draw lidar points
#     Args:
#         pc: numpy array (n,3) of XYZ
#         color: numpy array (n) of intensity or whatever
#         fig: mayavi figure handler, if None create new one otherwise will use it
#     Returns:
#         fig: created or used fig
#     """
#     # ind = (pc[:,2]< -1.65)
#     # pc = pc[ind]
#     pts_mode = "point"
#     print("====================", pc.shape)
#     if fig is None:
#         fig = mlab.figure(
#             figure=None, bgcolor=bgcolor, fgcolor=None, engine=None, size=(1600, 1000)
#         )
#     if color is None:
#         color = pc[:, 2]
#     if pc_label:
#         color = pc[:, 4]
#     if color_by_intensity:
#         color = pc[:, 2]
#
#     mlab.points3d(
#         pc[:, 0],
#         pc[:, 1],
#         pc[:, 2],
#         color,
#         color=pts_color,
#         mode=pts_mode,
#         colormap="gnuplot",
#         scale_factor=pts_scale,
#         figure=fig,
#     )
#
#     # draw origin
#     mlab.points3d(0, 0, 0, color=(1, 1, 1), mode="sphere", scale_factor=0.2)
#
#     # draw axis
#     axes = np.array(
#         [[2.0, 0.0, 0.0, 0.0], [0.0, 2.0, 0.0, 0.0], [0.0, 0.0, 2.0, 0.0]],
#         dtype=np.float64,
#     )
#     mlab.plot3d(
#         [0, axes[0, 0]],
#         [0, axes[0, 1]],
#         [0, axes[0, 2]],
#         color=(1, 0, 0),
#         tube_radius=None,
#         figure=fig,
#     )
#     mlab.plot3d(
#         [0, axes[1, 0]],
#         [0, axes[1, 1]],
#         [0, axes[1, 2]],
#         color=(0, 1, 0),
#         tube_radius=None,
#         figure=fig,
#     )
#     mlab.plot3d(
#         [0, axes[2, 0]],
#         [0, axes[2, 1]],
#         [0, axes[2, 2]],
#         color=(0, 0, 1),
#         tube_radius=None,
#         figure=fig,
#     )
#
#     # draw fov (todo: update to real sensor spec.)
#     fov = np.array(
#         [[20.0, 20.0, 0.0, 0.0], [20.0, -20.0, 0.0, 0.0]], dtype=np.float64  # 45 degree
#     )
#
#     mlab.plot3d(
#         [0, fov[0, 0]],
#         [0, fov[0, 1]],
#         [0, fov[0, 2]],
#         color=(1, 1, 1),
#         tube_radius=None,
#         line_width=1,
#         figure=fig,
#     )
#     mlab.plot3d(
#         [0, fov[1, 0]],
#         [0, fov[1, 1]],
#         [0, fov[1, 2]],
#         color=(1, 1, 1),
#         tube_radius=None,
#         line_width=1,
#         figure=fig,
#     )
#
#     # draw square region
#     TOP_Y_MIN = -20
#     TOP_Y_MAX = 20
#     TOP_X_MIN = 0
#     TOP_X_MAX = 40
#     #TOP_Z_MIN = -2.0
#     #TOP_Z_MAX = 0.4
#
#     x1 = TOP_X_MIN
#     x2 = TOP_X_MAX
#     y1 = TOP_Y_MIN
#     y2 = TOP_Y_MAX
#     mlab.plot3d(
#         [x1, x1],
#         [y1, y2],
#         [0, 0],
#         color=(0.5, 0.5, 0.5),
#         tube_radius=0.1,
#         line_width=1,
#         figure=fig,
#     )
#     mlab.plot3d(
#         [x2, x2],
#         [y1, y2],
#         [0, 0],
#         color=(0.5, 0.5, 0.5),
#         tube_radius=0.1,
#         line_width=1,
#         figure=fig,
#     )
#     mlab.plot3d(
#         [x1, x2],
#         [y1, y1],
#         [0, 0],
#         color=(0.5, 0.5, 0.5),
#         tube_radius=0.1,
#         line_width=1,
#         figure=fig,
#     )
#     mlab.plot3d(
#         [x1, x2],
#         [y2, y2],
#         [0, 0],
#         color=(0.5, 0.5, 0.5),
#         tube_radius=0.1,
#         line_width=1,
#         figure=fig,
#     )
#
#     # mlab.orientation_axes()
#     mlab.view(
#         azimuth=180,
#         elevation=70,
#         focalpoint=[12.0909996, -1.04700089, -2.03249991],
#         distance=100,
#         figure=fig,
#     )
#     return fig
#

def plane_segmentation(cloud, dist_thold, max_iter):
    """
    Input parameters:
        cloud: Input cloud
        dist_thold: distance threshold
        max_iter: maximal number of iteration
    Output:
        indices: list of indices of the PCL points that belongs to the plane
        coefficient: the coefficients of the plane-fitting (e.g., [a, b, c, d] for ax + by +cz + d =0)
    """
    seg = cloud.make_segmenter_normals(ksearch=50)  # For simplicity,hard coded
    seg.set_optimize_coefficients(True)
    seg.set_model_type(pcl.SACMODEL_NORMAL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_distance_threshold(dist_thold)
    seg.set_max_iterations(max_iter)
    indices, coefficients = seg.segment()
    return indices, coefficients

def clustering(cloud, tol, min_size, max_size):
    """
    Input parameters:
        cloud: Input cloud
        tol: tolerance
        min_size: minimal number of points to form a cluster
        max_size: maximal number of points that a cluster allows
    Output:
        cluster_indices: a list of list. Each element list contains the indices of the points that belongs to
                         the same cluster
    """
    tree = cloud.make_kdtree()
    ec = cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(tol)
    ec.set_MinClusterSize(min_size)
    ec.set_MaxClusterSize(max_size)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()
    return cluster_indices

def draw_point_cloud(cloud, ax, title,axes_limit, axes=[0, 1, 2],xlim3d=None, ylim3d=None, zlim3d=None):

    cloud = np.array(cloud)  # Covert point cloud to numpy array
    no_points = np.shape(cloud)[0]
    point_size = 10 ** (3 - int(np.log10(no_points)))  # Adjust the point size based on the point cloud size
    if np.shape(cloud)[1] == 4:  # If point cloud is XYZI format (e.g., I stands for intensity)
        ax.scatter(*np.transpose(cloud[:, axes]), s=point_size, c=cloud[:, 3], cmap='gray')
    elif np.shape(cloud)[1] == 3:  # If point cloud is XYZ format
        ax.scatter(*np.transpose(cloud[:, axes]), s=point_size, c='b', alpha=0.7)
    ax.set_xlabel('{} axis'.format(axes_str[axes[0]]))
    ax.set_ylabel('{} axis'.format(axes_str[axes[1]]))
    if len(axes) > 2:  # 3-D plot
        ax.set_xlim3d(axes_limit[axes[0]])
        ax.set_ylim3d(axes_limit[axes[1]])
        ax.set_zlim3d(axes_limit[axes[2]])
        ax.set_zlabel('{} axis'.format(axes_str[axes[2]]))
    else:  # 2-D plot
        ax.set_xlim(*axes_limit[axes[0]])
        ax.set_ylim(*axes_limit[axes[1]])
    # User specified limits
    if xlim3d != None:
        ax.set_xlim3d(xlim3d)
    if ylim3d != None:
        ax.set_ylim3d(ylim3d)
    if zlim3d != None:
        ax.set_zlim3d(zlim3d)
        ax.set_title(title)

def get_cluster_box_list(cluster_indices, cloud_obsts):
    """
    Input parameters:
        cluster_indices: a list of list. Each element list contains the indices of the points that belongs to
                         the same cluster
        colud_obsts: PCL for the obstacles
    Output:
        cloud_cluster_list: a list for the PCL clusters: each element is a point cloud of a cluster
        box_coord_list: a list of corrdinates for bounding boxes
    """
    cloud_cluster_list = []
    box_coord_list = []

    for j, indices in enumerate(cluster_indices):
        points = np.zeros((len(indices), 3), dtype=np.float32)
        for i, indice in enumerate(indices):
            points[i][0] = cloud_obsts[indice][0]
            points[i][1] = cloud_obsts[indice][1]
            points[i][2] = cloud_obsts[indice][2]
        cloud_cluster = pcl.PointCloud()
        cloud_cluster.from_array(points)
        cloud_cluster_list.append(cloud_cluster)
        x_max, x_min = np.max(points[:, 0]), np.min(points[:, 0])
        y_max, y_min = np.max(points[:, 1]), np.min(points[:, 1])
        z_max, z_min = np.max(points[:, 2]), np.min(points[:, 2])
        box = np.zeros([8, 3])
        box[0, :] = [x_min, y_min, z_min]
        box[1, :] = [x_max, y_min, z_min]
        box[2, :] = [x_max, y_max, z_min]
        box[3, :] = [x_min, y_max, z_min]
        box[4, :] = [x_min, y_min, z_max]
        box[5, :] = [x_max, y_min, z_max]
        box[6, :] = [x_max, y_max, z_max]
        box[7, :] = [x_min, y_max, z_max]
        box = np.transpose(box)
        box_coord_list.append(box)
    return cloud_cluster_list, box_coord_list


def draw_box(pyplot_axis, vertices, axes=[0, 1, 2], color='red'):
    """
    Draws a bounding 3D box in a pyplot axis.

    Parameters
    ----------
    pyplot_axis : Pyplot axis to draw in.
    vertices    : Array 8 box vertices containing x, y, z coordinates.
    axes        : Axes to use. Defaults to `[0, 1, 2]`, e.g. x, y and z axes.
    color       : Drawing color. Defaults to `black`.
    """
    vertices = vertices[axes, :]
    connections = [
        [0, 1], [1, 2], [2, 3], [3, 0],  # Lower plane parallel to Z=0 plane
        [4, 5], [5, 6], [6, 7], [7, 4],  # Upper plane parallel to Z=0 plane
        [0, 4], [1, 5], [2, 6], [3, 7]  # Connections between upper and lower planes
    ]
    for connection in connections:
        pyplot_axis.plot(*vertices[:, connection], c=color, lw=0.5)



def handler(raw_packet, ):
    """Takes each packet and log it to a file as xyz points"""
    global line_x
    global line_y
    global line_z
    global counter
    global file_cnt
    file_cnt =0
    # fig = plt.figure()
    # global ax
    # xyz = np.zeros(65536, 3)
    with open('poin.csv', 'a') as f:
        xyz = np.zeros(shape=(65536, 3))
        x, y, z = xyz_points(raw_packet)
        x = list(np.round(x,3))
        y = list(np.round(y,3))
        z = list(np.round(z,3))
        line_x.append(x)
        line_y.append(y)
        line_z.append(z)
        # print("one packet", completeounter)
        counter += 1
        if counter % 64 == 0:
            file_cnt += 1
            xyz[:, 0] = np.reshape(np.asarray(line_x), -1)
            xyz[:, 1] = np.reshape(np.asarray(line_y), -1)
            xyz[:, 2] = np.reshape(np.asarray(line_z), -1)
            print('xyz')
            print(xyz)
            # for row in xyz:
            #     print(row)
            #     np.savetxt(a_file, row)
            # a_file.close()
            # # array32 = partial(np.array, dtype=np.float32)
            # # line_x_arr = np.ndarray.flatten(np.asarray(line_x)).reshape(1,65536) #type= ndarray , size =(1,65536)
            # # line_y_arr = np.ndarray.flatten(np.asarray(line_y)).reshape(1,65536)
            # # line_z_arr = np.ndarray.flatten(np.asarray(line_z)).reshape(1,65536)
            #
            # # tp = np.flatten(line_x)
            # point_cloud_np = np.concatenate((np.transpose(line_x_arr),np.transpose(line_y_arr),np.transpose(line_z_arr)),axis=1)
            # point_cloud_np.astype(float32 , casting='unsafe')
            point_cloud_np_32 = np.float32(xyz)
            df = pd.DataFrame(point_cloud_np_32)
            filename = str(file_cnt)+"_data.csv"
            df.to_csv(filename)
            # # print("cloud data", point_cloud_np.shape)
            x_max, x_min = np.max(xyz[:, 0]), np.min(xyz[:, 0])
            y_max, y_min = np.max(xyz[:, 1]), np.min(xyz[:, 1])
            z_max, z_min = np.max(xyz[:, 2]), np.min(xyz[:, 2])
            axes_limits = [
                [int(x_min * 1.2), int(x_max * 1.2)],  # X axis range
                [int(y_min * 1.2), int(y_max * 1.2)],  # Y axis range
                [-5, 5]  # Z axis range
            ]
            # # print('x_max: ', x_max, ', x_min: ', x_min)
            # # print('y_max: ', y_max, ', y_min: ', y_min)
            # # print('z_max: ', z_max, ', z_min: ', z_min)
            # print('Number of points: ', xyz.size)
            cloud_XYZ = pcl.PointCloud()
            cloud_XYZ.from_array(point_cloud_np_32)
            cloud_roi_filtered = roi_filter(cloud_XYZ,[-2, 50], [-5, 20], [-2, 2])
            print('Input cloud size: ', cloud_XYZ.size)
            print('Size after ROI filter: ', cloud_roi_filtered.size)
            cloud_roi_filtered_np = np.array(cloud_roi_filtered)
            # print(cloud_roi_filtered_np.shape)
            indices, coefficients = plane_segmentation(cloud_roi_filtered, 0.2, 1000)
            if len(indices) == 0:
                print('Could not estimate a planar model for the given dataset.')
            # print('Model coefficients: ' + str(coefficients[0]) + ', ' + str(
            #     coefficients[1]) + ', ' + str(coefficients[2]) + ', ' + str(coefficients[3]))
            cloud_plane = cloud_roi_filtered.extract(indices, negative=False)
            # cloud_obsts = cloud_roi_filtered.extract(indices, negative=True)
            cloud_plane_np =np.array(cloud_plane)

            pcd.points = o3d.utility.Vector3dVector(cloud_plane_np)
            vis.update_geometry()
            vis.poll_events()
            vis.update_renderer()
            # # cluster_indices = clustering(cloud_obsts, 0.7, 30, 100)
            # # cloud_cluster_list, box_coord_list = get_cluster_box_list(cluster_indices, cloud_obsts)
            # # f = plt.figure(figsize=(15, 8))
            # # ax = f.add_subplot(111, projection='3d')
            # # draw_point_cloud(cloud_obsts, ax, 'Point Cloud',axes_limits, xlim3d=(-30, 40))
            # # for box in box_coord_list:
            # #     draw_box(ax, box, axes=[0, 1, 2], color='m')
            # # plt.show()
            #
            # fig = draw_lidar(cloud_plane_np)
            #
            # mlab.savefig("pc_view.jpg", figure=fig)
            # # ax.scatter3D(line_x, line_y, line_z,s=0.05 ,c=line_z, cmap='Greens')
            # # plt.plot(line_z)

            counter = 0
            line_x = []
            line_y = []
            line_z = []
            np.delete(xyz)
            # time.sleep(1)
            # plt.clf()
            # plt.cla()
            # plt.close()

#
#
# os1 = OS1('169.254.61.31', '169.254.61.32', mode='1024x10')  # OS1 sensor IP, destination IP, and resolution
# # Inform the sensor of the destination host and reintialize it
# os1.start()
# # Start the loop which will handle and dispatch each packet to the handler
# # function for processing
# os1.run_forever(handler)


