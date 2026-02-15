import numpy as np
import cv2
import pykitti
import open3d as o3d

# --- 1. CONFIGURATION ---
# Change this to the directory where you store KITTI data
basedir = 'KITTI_SAMPLE/RAW'
date = '2011_09_26'
drive = '0009'
nb_frames = 10  # Limited because stereo matching generates millions of points

# Load the raw KITTI dataset
dataset = pykitti.raw(basedir, date, drive, frames=range(0, nb_frames))

# --- 2. PARAMETERS INITIALIZATION (System Invariants) ---
# Intrinsic parameters for Camera 2 (Left RGB)
K2 = dataset.calib.K_cam2
f = K2[0, 0]   # Focal length
cx = K2[0, 2]  # Principal point x
cy = K2[1, 2]  # Principal point y

# Baseline (distance between cam2 and cam3 center)
# In KITTI, the horizontal baseline is approximately 0.54m
baseline = 0.54

# Transformation Matrix: Camera 2 -> IMU
T_cam2_from_imu = dataset.calib.T_cam2_imu
T_imu_from_cam2 = np.linalg.inv(T_cam2_from_imu)

# Semi-Global Block Matching (SGBM) Configuration
# These settings balance depth precision and computation time
window_size = 3
stereo_matcher = cv2.StereoSGBM_create(
    minDisparity=0,
    numDisparities=64,  # Must be a multiple of 16
    blockSize=window_size,
    P1=8 * 3 * window_size**2,
    P2=32 * 3 * window_size**2,
    disp12MaxDiff=1,
    uniquenessRatio=15,
    speckleWindowSize=100,
    speckleRange=32,
    mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
)

all_points_world = []
all_colors = []

print("Starting Stereo Reconstruction Pipeline...")

# --- 3. PROCESSING LOOP ---
for i in range(nb_frames):
    # Load Left (cam2) and Right (cam3) RGB images
    img_l = np.array(dataset.get_cam2(i))
    img_r = np.array(dataset.get_cam3(i))

    # Convert to grayscale for disparity computation
    gray_l = cv2.cvtColor(img_l, cv2.COLOR_RGB2GRAY)
    gray_r = cv2.cvtColor(img_r, cv2.COLOR_RGB2GRAY)

    # A. Disparity Map Computation
    # OpenCV returns disparity scaled by 16, we normalize it to float
    disparity = stereo_matcher.compute(gray_l, gray_r).astype(np.float32) / 16.0

    # B. Depth Masking
    # Depth Z = (f * baseline) / disparity
    # We ignore pixels with invalid or near-zero disparity to avoid division by zero
    valid_depth_mask = (disparity > 0.1)

    # C. 3D Triangulation (Camera Coordinate Frame)
    h, w = disparity.shape
    u, v = np.meshgrid(np.arange(w), np.arange(h))

    # Calculate depth and project to 3D space
    Z = (f * baseline) / disparity[valid_depth_mask]
    X = (u[valid_depth_mask] - cx) * Z / f
    Y = (v[valid_depth_mask] - cy) * Z / f

    # Filtering: Limit range to 40m to reduce background noise/artifacts
    distance_mask = Z < 40
    X, Y, Z = X[distance_mask], Y[distance_mask], Z[distance_mask]

    # D. Coordinate Transformation to WORLD Frame
    # 1. Prepare homogeneous coordinates [X, Y, Z, 1]
    pts_cam = np.vstack((X, Y, Z, np.ones_like(X)))

    # 2. Chain Transformation: Camera -> IMU -> World
    T_imu_to_world = dataset.oxts[i].T_w_imu
    pts_world = (T_imu_to_world @ T_imu_from_cam2 @ pts_cam).T

    # E. Color Extraction (Normalize RGB to [0, 1] for Open3D)
    colors = img_l[valid_depth_mask][distance_mask] / 255.0

    all_points_world.append(pts_world[:, :3])
    all_colors.append(colors)
    print(f"Frame {i} processed: {len(X)} points generated.")

# --- 4. EXPORT & POST-PROCESSING ---
combined_points = np.concatenate(all_points_world)
combined_colors = np.concatenate(all_colors)

# Create Open3D PointCloud object
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(combined_points)
pcd.colors = o3d.utility.Vector3dVector(combined_colors)

# Statistical Outlier Removal (Cleans up stereo "ghost" points)
pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

# Save result
o3d.io.write_point_cloud("stereo_reconstruction.pcd", pcd)
print("File stereo_reconstruction.pcd successfully saved.")