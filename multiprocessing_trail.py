from os1 import OS1
from os1.utils import xyz_points , build_trig_table
import numpy as np
import open3d as o3d
import pcl
import time
import json
from multiprocessing import Process, Queue

from conti_plotting import plane_segmentation,roi_filter , clustering, get_cluster_box_list
lines = [
        [0, 1],
        [0, 2],
        [1, 3],
        [2, 3],
        [4, 5],
        [4, 6],
        [5, 7],
        [6, 7],
        [0, 4],
        [1, 5],
        [2, 6],
        [3, 7],
]

#  System IP and Sensor IP
system_ip = '169.254.61.32'
sensor_ip = '169.254.61.31'
fetch_mode = '1024x10'
lidar_channels = 64
resolution = 1024
fnt =0
unprocessed_packets = Queue()

# init_sample_data = np.random.randn(16384 ,3)   # Random data
init_sample_data = np.random.randn(65536 ,3)   # Random data


vis = o3d.visualization.Visualizer()
vis.create_window()
vis.get_render_option().load_from_json("renderoption.json")

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(init_sample_data)

vis.add_geometry(pcd)


line_x = []
line_y = []
line_z = []
counter = 0


def handler(packet):
    unprocessed_packets.put(packet)


def spawn_workers(n, worker, *args, **kwargs):
    processes = []
    for i in range(n):
        process = Process(
            target=worker,
            args=args,
            kwargs=kwargs
        )
        process.start()
        processes.append(process)
    return processes


def worker(queue, beam_altitude_angles, beam_azimuth_angles):
    global line_x
    global line_y
    global line_z
    global counter
    global lines
    global fnt
    build_trig_table(beam_altitude_angles, beam_azimuth_angles)

    while True:
        packet = queue.get()
        x, y, z = xyz_points(packet)
        x = list(np.round(x,3))
        y = list(np.round(y,3))
        z = list(np.round(z,3))
        line_x.append(x)
        line_y.append(y)
        line_z.append(z)
        counter += 1
        if counter % 64 == 0:
            fnt +=1
            line_x_arr = np.asarray(line_x).reshape(1,lidar_channels*resolution)
            line_y_arr = np.asarray(line_y).reshape(1,lidar_channels*resolution)
            line_z_arr = np.asarray(line_z).reshape(1,lidar_channels*resolution)
            # line_y_arr = line_y_arr[:,0::4]
            # line_x_arr = line_x_arr[:,0::4]
            # line_z_arr = line_z_arr[:,0::4]
            # tp = np.flatten(line_x)
            point_cloud_np = np.concatenate((np.transpose(line_x_arr),np.transpose(line_y_arr),np.transpose(line_z_arr)),axis=1)

            ############################ PCL codeing ###################################################
            xyz = np.float32(point_cloud_np)

            # cloud_XYZ = pcl.PointCloud()
            # cloud_XYZ.from_array(xyz)
            # cloud_roi_filtered = roi_filter(cloud_XYZ, [-1, 50], [-2,2], [-2, 2])
            #
            # indices, coefficients = plane_segmentation(cloud_XYZ, 0.1, 1000)
            #
            # cloud_plane = cloud_XYZ.extract(indices, negative=True)

            # cluster_indices = clustering(cloud_plane, 0.3, 30, 100)
            #
            # cloud_cluster_list, box_coord_list = get_cluster_box_list(cluster_indices, cloud_plane)

            # colors = [[1, 0, 0] for i in range(len(lines))]
            # line_set = []
            # for box in box_coord_list:
            #     # o3d.geometry.OrientedBoundingBox()
            #     try:
            #         o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(box.transpose()),
            #                                     lines=o3d.utility.Vector2iVector(lines))
            #     except :
            #         pass
            # line_set.colors = o3d.utility.Vector3dVector(colors)
            # o3d.visualization.draw_geometries([line_set], zoom=0.8)
            # if box_coord_list != []:
            # point_cloud_np = np.array(line_set)
            # point_cloud_np = np.array(point_cloud_np)
            # point_cloud_np = np.array(cloud_plane)
                ################################################################
            print("cloud data", point_cloud_np.shape)
            pcd.points = o3d.utility.Vector3dVector(point_cloud_np)
            # file_name= "3d_point_"+str(fnt)+".png"
            # vis.capture_screen_image(file_name)
            vis.update_geometry()
            vis.poll_events()
            vis.update_renderer()
            counter = 0
            line_x = []
            line_y = []
            line_z = []


os1 = OS1(sensor_ip, system_ip)
beam_intrinsics = json.loads(os1.get_beam_intrinsics())
beam_alt_angles = beam_intrinsics['beam_altitude_angles']
beam_az_angles = beam_intrinsics['beam_azimuth_angles']
workers = spawn_workers(4, worker, unprocessed_packets, beam_alt_angles, beam_az_angles)
os1.start()
try:
    os1.run_forever(handler)
except KeyboardInterrupt:
    for w in workers:
        w.terminate()


