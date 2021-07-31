#pragma once

#include <costmap_2d/obstacle_layer.h>
#include <obstacle_layer_laser_undistortion/laser_geometry.h>

namespace obstacle_layer_laser_undistortion
{
class ObstacleLayerLaserUndistortion : public costmap_2d::ObstacleLayer
{
public:
    void onInitialize() override;
    void laserScanCallback(const sensor_msgs::LaserScanConstPtr& message, const boost::shared_ptr<costmap_2d::ObservationBuffer>& buffer);
    void laserScanValidInfCallback(const sensor_msgs::LaserScanConstPtr& message, const boost::shared_ptr<costmap_2d::ObservationBuffer>& buffer);
protected:
    obstacle_layer_laser_undistortion::LaserProjection projector_; ///< @brief Used to project laser scans into point clouds
    bool rotating_ { true }; //whether the lidar is rotating
};
}
