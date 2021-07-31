# obstacle_layer_laser_undistortion
A modification of costmap_2d::ObstacleLayer witch undistorts laser data from single line lidar.
## Configuration
Setup **obstacle_layer_laser_undistortion/ObstacleLayerLaserUndistortion** instead of costmap_2d::ObstacleLayer for obstacle_layer in `local_costmap_params.yaml` and `global_costmap_params.yaml` and add **lidar_rotating** and **lidar_rotation_direction** param for each lidar sensor in `costmap_common_params.yaml`.  
1. lidar_rotating (bool, default: true)  
Whether the lidar is rotating. Mechanical lidars are rotating while solid lidars are not.  
2. lidar_rotation_direction (bool, default: true)  
True for anticlockwise, false for clockwise.