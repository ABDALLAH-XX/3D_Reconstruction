import numpy as np
import matplotlib.pyplot as plt
import plyfile as ply
import pykitti

# --- 1. CONFIGURATION ---
# Change this to the directory where you store KITTI data
basedir = 'KITTI_SAMPLE/RAW'

# Specify the dataset to load
date = '2011_09_26'
drive = '0009'

# Load the data. Optionally, specify the frame range to load.
dataset = pykitti.raw(basedir, date, drive,
                      frames=range(0, 447, 1))


# dataset.calib:         Calibration data are accessible as a named tuple
# dataset.timestamps:    Timestamps are parsed into a list of datetime objects
# dataset.oxts:          List of OXTS packets and 6-dof poses as named tuples
# dataset.camN:          Returns a generator that loads individual images from camera N
# dataset.get_camN(idx): Returns the image from camera N at idx
# dataset.gray:          Returns a generator that loads monochrome stereo pairs (cam0, cam1)
# dataset.get_gray(idx): Returns the monochrome stereo pair at idx
# dataset.rgb:           Returns a generator that loads RGB stereo pairs (cam2, cam3)
# dataset.get_rgb(idx):  Returns the RGB stereo pair at idx
# dataset.velo:          Returns a generator that loads velodyne scans as [x,y,z,reflectance]
# dataset.get_velo(idx): Returns the velodyne scan at idx

# --- 2. PARAMETERS INITIALIZATION (System Invariants) ---
# Calibration Matrix
T_velo_to_cam2 = dataset.calib.T_cam2_velo
K_cam2 = dataset.calib.K_cam2

# 3D -> 2D Projection Matrix
P_matrix = np.eye(4)
P_matrix[:3, :3] = K_cam2
T_world_projection = P_matrix @ T_velo_to_cam2

# IMU to Lidar transformation
T_velo_to_imu = np.linalg.inv(dataset.calib.T_velo_imu)

# Containers for 3D points and their corresponding colors
all_world_points = []
all_vertex_colors = []

# --- 3. PROCESSING LOOP ---
for i in range(50):
    # Load the RGB image and the corresponding LiDAR point cloud
    rgb_img = np.array(dataset.get_cam2(i))
    velo_scan = dataset.get_velo(i) # [x, y, z, reflectance]
    velo_scan = velo_scan[velo_scan[:, 0] > 5] # Filter out points that are too close to the sensor
    velo_scan[:,3] = 1 # Add homogeneous coordinate

    # Project LiDAR points to the camera frame
    T_imu_to_world = dataset.oxts[i].T_w_imu
    world_points = T_imu_to_world @ T_velo_to_imu @ velo_scan.T
    

    # 3D to 2D projection for color extraction
    homogeneous_2d_points = T_world_projection @ velo_scan.T
    points_2d = np.array([homogeneous_2d_points[0] / homogeneous_2d_points[2],
                          homogeneous_2d_points[1] / homogeneous_2d_points[2]])
    
    # Filter points that are outside the image boundaries
    mask = (points_2d[0] >= 0) & (points_2d[0] < rgb_img.shape[1]) & \
           (points_2d[1] >= 0) & (points_2d[1] < rgb_img.shape[0])
    
    # Color extraction for valid points
    valid_points_2d = points_2d[:, mask].astype(int)
    colors_for_valid_points = rgb_img[valid_points_2d[1], valid_points_2d[0]]

    # Append valid 3D points and their corresponding colors
    all_world_points.append(world_points[0:3, mask].T)
    all_vertex_colors.append(colors_for_valid_points)

# --- Save the reconstructed point cloud as a PLY file ---
all_points = np.concatenate(all_world_points, axis=0)
all_colors = np.concatenate(all_vertex_colors, axis=0)

# Create a structured array for PLY format
vertex_data = [
    (all_points[i, 0], all_points[i, 1], all_points[i, 2],
     all_colors[i, 0], all_colors[i, 1], all_colors[i, 2])
    for i in range(all_points.shape[0])
]

# Define the PLY data type
vertex = np.array(vertex_data, dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4'),
                                      ('red', 'u1'), ('green', 'u1'), ('blue', 'u1')])

# Create a PLY element
ply_element = ply.PlyElement.describe(vertex, 'vertex')

# Write the PLY file
print(f"Total points in reconstructed point cloud: {len(vertex)}")
print(f"Saving reconstructed point cloud to 'reconstructed_point_cloud.ply'...")
ply.PlyData([ply_element], text=True).write('/data/reconstructed_point_cloud.ply')
print("Reconstruction complete.")




