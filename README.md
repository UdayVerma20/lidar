# LiDAR Processing Pipeline

This repository contains a LiDAR processing pipeline for object detection and reconstruction, implemented using ROS and C++.

## Features

- **Ground Removal**: Removes the ground plane to reduce computational load for subsequent steps.  
- **Clustering**: Identifies distinct objects in the point cloud, detects cones, and removes false positives.  
- **Reconstruction**: Adds points back to the identified objects after ground removal.  
- **Algorithms**:
  - Custom K-Means-inspired clustering algorithm.
  - Plane construction and propagation using a min-heap approach.

## Installation

1. **Clone the repository**
```
git clone https://github.com/UdayVerma20/lidar.git
```
2. **Build using Catkin**
```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```
