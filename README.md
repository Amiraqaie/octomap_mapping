octomap_mapping ![CI](https://github.com/OctoMap/octomap_mapping/workflows/CI/badge.svg)
===============

ROS stack for mapping with OctoMap, contains the `octomap_server` package.

The main branch for ROS1 Kinetic, Melodic, and Noetic is `kinetic-devel`.

The main branch for ROS2 Foxy and newer is `ros2`.

### Changes in this fork

This fork changes how `octomap_server` publishes its outputs:

1. **The 3D outputs always contain all heights.** `octomap_point_cloud_centers` and
   `occupied_cells_vis_array` are no longer cut to `occupancy_min_z`..`occupancy_max_z`.
2. **The 2D `projected_map` uses only a height slice**, and the slice can follow the robot:
   `[odom_z + occupancy_z_offset_min, odom_z + occupancy_z_offset_max]`, where `odom_z` comes from
   a `nav_msgs/Odometry` topic (default `/Odometry`).
3. **Map changes are published incrementally** as three point clouds (added, removed, changed
   voxels), so an external node can keep its own copy of the map up to date without receiving the
   whole map every time.
4. **Height-cropped point clouds for Nav2**: up to two input clouds (for example the SLAM cloud and the
   sensor cloud) are cropped in height and republished, one for the global and one for the local costmap.

Everything new is opt-in. With `dynamic_z_range:=false` and `publish_delta:=false` the node behaves
as before, except for point 1.

### Usage

#### Run the server (example)

The example below uses FAST-LIO style topics: a registered point cloud in the `camera_init` frame
and `/Odometry` in the same frame. Adapt the topic names and `frame_id` to your setup.

```
ros2 run octomap_server octomap_server_node --ros-args \
  -r cloud_in:=/cloud_registered \
  -p frame_id:=camera_init \
  -p resolution:=0.1 \
  -p dynamic_z_range:=true \
  -p odom_topic:=/Odometry \
  -p occupancy_z_offset_min:=-0.3 \
  -p occupancy_z_offset_max:=0.3 \
  -p publish_delta:=true \
  -p delta_changed_min_logodds:=1.0 \
  -p nav2_global_cloud_in:=/cloud_registered \
  -p nav2_local_cloud_in:=/cloud_registered_body \
  -p nav2_z_offset_min:=-0.3 \
  -p nav2_z_offset_max:=0.5
```

`/cloud_registered` (frame `camera_init`) and `/cloud_registered_body` (body frame) are the FAST-LIO names;
check your own with `ros2 topic list`.

Check the executable name of your build with `ros2 pkg executables octomap_server`.

Notes:

* A node must be able to transform `cloud_in` into `frame_id` through tf. If the point cloud is
  already in `frame_id`, an identity transform is enough.
* If `/Odometry` is not in `frame_id`, the server looks up the latest transform from the odometry
  frame to `frame_id` through tf. If the lookup fails, the slice keeps its last value.
* Floating point parameters must be written as decimals on the command line
  (`-0.3`, not `-1`), otherwise ROS 2 rejects the type change.

Tune the slice while the node is running:

```
ros2 param set /octomap_server occupancy_z_offset_min -0.5
ros2 param set /octomap_server occupancy_z_offset_max 0.5
ros2 param set /octomap_server publish_delta false
```

Check that the delta stream is alive:

```
ros2 topic echo /octomap_delta_epoch --once
ros2 topic hz /octomap_delta_added
```

#### Save octomap

```
ros2 run octomap_server octomap_saver_node --ros-args -p octomap_path:=(path for saving octomap)
```
Note: The extension of octomap path should be `.bt` or `.ot`

### New parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `dynamic_z_range` | bool | `false` | Make the 2D projection slice follow the odometry height. Can be changed at runtime. |
| `odom_topic` | string | `/Odometry` | `nav_msgs/Odometry` topic that provides the height. Read at startup only. |
| `occupancy_z_offset_min` | double | `-0.3` | Lower edge of the slice, relative to the odometry height [m]. |
| `occupancy_z_offset_max` | double | `0.3` | Upper edge of the slice, relative to the odometry height [m]. |
| `publish_delta` | bool | `false` | Publish the added / removed / changed clouds. Can be changed at runtime. |
| `delta_changed_min_logodds` | double | `0.0` | A voxel that stays occupied is reported as changed only if its log-odds moved by more than this. |
| `nav2_global_cloud_in` | string | `""` | Input cloud for the global costmap output. Empty disables it. Read at startup only. |
| `nav2_local_cloud_in` | string | `""` | Input cloud for the local costmap output. Empty disables it. Read at startup only. |
| `nav2_crop_relative_to_odom` | bool | `true` | Crop relative to the odometry height (`true`) or with absolute world heights (`false`). |
| `nav2_z_offset_min` / `nav2_z_offset_max` | double | `-0.3` / `0.3` | Crop limits relative to the odometry height [m]. |
| `nav2_min_z` / `nav2_max_z` | double | `-100.0` / `100.0` | Absolute crop limits in `frame_id` [m], used when `nav2_crop_relative_to_odom` is `false`. |

Existing parameters keep their meaning. `occupancy_min_z` and `occupancy_max_z` are still used for the
2D projection when `dynamic_z_range` is `false`, and until the first odometry message has arrived.

### Topics

| Topic | Type | Description |
|---|---|---|
| `octomap_point_cloud_centers` | `sensor_msgs/PointCloud2` | Occupied voxels of the whole map, all heights. |
| `projected_map` | `nav_msgs/OccupancyGrid` | 2D projection of the height slice only. |
| `octomap_delta_added` | `sensor_msgs/PointCloud2` (XYZI) | Voxels that became occupied. |
| `octomap_delta_removed` | `sensor_msgs/PointCloud2` (XYZI) | Voxels that are no longer occupied. |
| `octomap_delta_changed` | `sensor_msgs/PointCloud2` (XYZI) | Voxels that stay occupied but whose log-odds changed by more than `delta_changed_min_logodds`. |
| `octomap_delta_epoch` | `std_msgs/UInt32` (latched) | Increments when the map was reset, partly cleared or loaded from a file. |
| `nav2_global_cloud` | `sensor_msgs/PointCloud2` | `nav2_global_cloud_in`, cropped in height. For the Nav2 global costmap. |
| `nav2_local_cloud` | `sensor_msgs/PointCloud2` | `nav2_local_cloud_in`, cropped in height. For the Nav2 local costmap. |

### Delta clouds

* Points are voxel centers at the finest resolution of the tree, independent of pruning and of `max_depth`.
* The `intensity` field is the occupancy probability of the voxel after the update.
* All three clouds carry the stamp of the input point cloud that caused the update.
* The clouds are published only when at least one of them is not empty, and only while `publish_delta`
  is `true` and somebody subscribes to one of them.
* Deltas cover the whole tree. The height slice and the speckle filter do not apply to them.
* The delta topics are reliable with a queue depth of 10 and are not latched. Subscribe with reliable
  QoS: a lost message leaves your copy of the map out of sync.

#### Rebuilding a map from the deltas

1. Start from an `octomap_binary` message (latched) with stamp `T0`.
2. For every delta with a stamp greater than `T0`: erase the `removed` voxels, insert the `added`
   voxels, update the `changed` voxels. Identify a voxel by `round(coordinate / resolution)`.
3. If `octomap_delta_epoch` changes, drop your map and start again from step 1 with the next
   `octomap_binary` message. This happens after `~/reset`, `~/clear_bbox` and loading a file, which
   cannot be expressed as deltas.

### Cropped clouds for Nav2

* The height test is done in `frame_id`, so a body-frame cloud is cropped by its real height and not by
  its tilted body z. This needs a tf transform from the cloud frame to `frame_id`. If the exact stamp is
  not available, the latest transform is used.
* The output keeps the **frame, stamp and all fields** of its input. Only points are removed. A body-frame
  input therefore stays a body-frame output, which is what the Nav2 obstacle layer needs for raytracing.
* In the default mode the limits are `odom_z + nav2_z_offset_min` .. `odom_z + nav2_z_offset_max`. Until the first
  odometry message arrives, nothing is published in this mode.
* The x, y, z fields must be float32. Nothing is computed while an output has no subscriber.
* An empty result is still published, because Nav2 uses empty clouds for clearing.
* The two inputs can be the same topic.
* Suggested use: the SLAM cloud for the global costmap (marking only), the sensor cloud for the local
  costmap (marking and clearing).

Nav2 configuration (obstacle layer):

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      obstacle_layer:
        observation_sources: pointcloud
        pointcloud:
          topic: /nav2_global_cloud
          data_type: PointCloud2
          marking: true
          clearing: false
          min_obstacle_height: -10.0
          max_obstacle_height: 10.0
local_costmap:
  local_costmap:
    ros__parameters:
      obstacle_layer:
        observation_sources: pointcloud
        pointcloud:
          topic: /nav2_local_cloud
          data_type: PointCloud2
          marking: true
          clearing: true
          min_obstacle_height: -10.0
          max_obstacle_height: 10.0
```

The height limits are wide on purpose: the crop is already done by the server, and Nav2 measures these
limits in its own global frame, whose z may differ from `frame_id`.

### Behavior notes

* With `dynamic_z_range:=true` the 2D map is rebuilt completely on every update, so
  `incremental_2D_projection` has no effect.
* The slice is evaluated when the map is published (once per inserted point cloud). If the robot changes
  height between two scans, the 2D map lags by up to one scan.
* If the odometry stops, the slice stays at its last height.
* Tracking changes costs one extra octree lookup per updated voxel, and only while the delta
  topics have a subscriber.
* The delta buffers are not protected by a lock, like the octree itself. This is fine with the default
  single-threaded executor.