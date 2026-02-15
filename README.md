# 🏎️ KITTI 3D Perception & Reconstruction Suite
This repository features two advanced approaches for 3D environment sensing in autonomous driving, using the **KITTI Raw Dataset**. This project covers the entire perception pipeline: from coordinate frame transformations to sensor fusion and point cloud post-processing.

## 🚀 Performance & Technical Benchmarks
After optimizing the projection matrices and stereo matching parameters, the system achieves the following results:

| Metric | LiDAR-Camera Fusion | Stereo Vision (SGBM) |
| :--- | :--- | :--- |
| **Precision** | **Metric accuracy** (error < 5cm) | Estimation-based (Z-dependent) |
| **Density** | Sparse (64 vertical scan lines) | **Highly Dense** (per-pixel depth) |
| **Field of View**| Road-level focused | **Full scene** (trees, poles, buildings) |
| **Reliability** | 100% in all lighting conditions | Struggles with low-texture areas |

---

## 🛠️ Technical Implementation

### 1. LiDAR-Camera Global Mapper
This pipeline fuses the precision of a Velodyne HDL-64 LiDAR with RGB imagery to create a colorized global map.
* **Coordinate Transformations:** Handles the full chain $T_{Velo \to IMU}$ and $T_{IMU \to World}$ using OXTS/GPS data to align multiple frames into a single world-referenced map.
* **Perspective Projection:** Projects 3D points onto the 2D image plane using intrinsic ($K$) and extrinsic ($T$) matrices.
* **Color Extraction:** Maps RGB values to LiDAR points by normalizing homogeneous coordinates ($u = X/Z, v = Y/Z$).



### 2. Stereo Vision Reconstruction (SGBM)
A purely vision-based approach using two synchronized cameras to estimate depth through triangulation.
* **SGBM Algorithm:** Implementation of *Semi-Global Block Matching* for disparity map computation, optimized with $P_1$ and $P_2$ penalty parameters for depth smoothness.
* **Triangulation:** Conversion of disparity to depth using the camera baseline ($B = 0.54m$) and focal length ($f$).
* **Post-Processing:** Statistical outlier removal via **Open3D** to eliminate "ghost" points and artifacts common in stereo matching.



---

## 📊 Comparative Analysis: Sparsity vs. Density

A fundamental trade-off was identified during this project:

1.  **LiDAR Fusion (Sparsity & Precision):** The model is mathematically exact but appears "striped" due to the 64-line vertical resolution of the sensor. It often misses high-altitude details (tree branches) that fall outside the LiDAR's vertical FoV.
2.  **Stereo Reconstruction (Density & Coverage):** The SGBM algorithm provides a much richer semantic understanding of the environment, capturing vertical structures that the LiDAR misses, although it introduces more depth noise in complex textures like foliage.

---

## 📁 Project Structure
- `kitti_global_mapper.py`: Core logic for LiDAR-Camera fusion and global mapping.
- `kitti_stereo_reconstruction.py`: Stereo matching pipeline and Open3D post-processing.
- `calibration/`: Utilities to parse KITTI extrinsic and intrinsic matrices.
- `output/`: Sample `.ply` and `.pcd` files for visualization in CloudCompare or MeshLab.

## 🛠️ Tech Stack
* **Language:** Python 3.x
* **Computer Vision:** OpenCV (SGBM), Open3D (Point Cloud processing).
* **Data Science:** NumPy (Vectorized matrix operations), PyKitti.
* **3D Formats:** Plyfile (Binary PLY export).
