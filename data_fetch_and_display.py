from os1 import OS1
from os1.utils import xyz_points
import mayavi.mlab as mlab
import numpy as np
import open3d as o3d


#  System IP and Sensor IP
system_ip = '169.254.61.32'
sensor_ip = '169.254.61.31'
fetch_mode = '1024x10'
lidar_channels = 64
resolution = 1024

init_sample_data = [[0.2 , 0.2 , 0.2]]   # Random data

vis = o3d.visualization.Visualizer()
vis.create_window()
vis.get_render_option().load_from_json("renderoption.json")

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(init_sample_data)

vis.add_geometry(pcd)

vis.get_render_option()

line_x = []
line_y = []
line_z = []
counter = 0

def handler(raw_packet):
    global line_x
    global line_y
    global line_z
    global counter
    global vis
    with open('poin.csv', 'a') as f:
        x, y, z = xyz_points(raw_packet)
        x = list(np.round(x,3))
        y = list(np.round(y,3))
        z = list(np.round(z,3))
        line_x.append(x)
        line_y.append(y)
        line_z.append(z)
        counter += 1
        if counter % 64 == 0:
            line_x_arr = np.asarray(line_x).reshape(1,lidar_channels*resolution)
            line_y_arr = np.asarray(line_y).reshape(1,lidar_channels*resolution)
            line_z_arr = np.asarray(line_z).reshape(1,lidar_channels*resolution)

            # tp = np.flatten(line_x)
            point_cloud_np =np.concatenate((np.transpose(line_x_arr),np.transpose(line_y_arr),np.transpose(line_z_arr)),axis=1)
            print("cloud data", point_cloud_np.shape)
            pcd.points = o3d.utility.Vector3dVector(point_cloud_np)
            vis.update_geometry()
            vis.poll_events()
            vis.update_renderer()
            counter = 0
            line_x = []
            line_y = []
            line_z = []

os1 = OS1(sensor_ip, system_ip, mode=fetch_mode)  # OS1 sensor IP, destination IP, and resolution
# Inform the sensor of the destination host and reintialize it
os1.start()
os1.run_forever(handler)


