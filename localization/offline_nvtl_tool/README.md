# Offline NVTL Tool

## How to use

### 1 record rosbag

```bash
ros2 bag record --use-sim-time \
    /map/pointcloud_map \
    /localization/pose_estimator/pose_with_covariance \
    /localization/util/downsample/pointcloud \
    /tf \
    /tf_static \
    /perception/object_recognition/detection/centerpoint/objects \
    /perception/object_recognition/detection/objects
```

### 2 launch offline_nvtl_tool

```bash
ros2 launch offline_nvtl offline_nvtl.launch.xml \
  rosbag_path:=sample_rosbag
```

It creates `nvtl.csv` in the directory where launch.

### 3 launch plot script

```bash
./install/offline_nvtl_tool/share/offline_nvtl_tool/scripts/plot_nvtl.py
```
