#include <obstacle_layer_laser_undistortion/obstacle_layer_laser_undistortion.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(obstacle_layer_laser_undistortion::ObstacleLayerLaserUndistortion, costmap_2d::Layer)

namespace obstacle_layer_laser_undistortion
{
void ObstacleLayerLaserUndistortion::onInitialize()
{
  ros::NodeHandle nh("~/" + name_), g_nh;
  rolling_window_ = layered_costmap_->isRolling();

  bool track_unknown_space;
  nh.param("track_unknown_space", track_unknown_space, layered_costmap_->isTrackingUnknown());
  if (track_unknown_space)
    default_value_ = costmap_2d::NO_INFORMATION;
  else
    default_value_ = costmap_2d::FREE_SPACE;

  matchSize();
  current_ = true;

  global_frame_ = layered_costmap_->getGlobalFrameID();
  double transform_tolerance;
  nh.param("transform_tolerance", transform_tolerance, 0.2);

  std::string topics_string;
  // get the topics that we'll subscribe to from the parameter server
  nh.param("observation_sources", topics_string, std::string(""));
  ROS_INFO("    Subscribed to Topics: %s", topics_string.c_str());

  // now we need to split the topics based on whitespace which we can use a stringstream for
  std::stringstream ss(topics_string);

  std::string source;
  while (ss >> source)
  {
    ros::NodeHandle source_node(nh, source);

    // get the parameters for the specific topic
    double observation_keep_time, expected_update_rate, min_obstacle_height, max_obstacle_height;
    std::string topic, sensor_frame, data_type;
    bool inf_is_valid, clearing, marking;

    source_node.param("topic", topic, source);
    source_node.param("sensor_frame", sensor_frame, std::string(""));
    source_node.param("observation_persistence", observation_keep_time, 0.0);
    source_node.param("expected_update_rate", expected_update_rate, 0.0);
    source_node.param("data_type", data_type, std::string("PointCloud"));
    source_node.param("min_obstacle_height", min_obstacle_height, 0.0);
    source_node.param("max_obstacle_height", max_obstacle_height, 2.0);
    source_node.param("inf_is_valid", inf_is_valid, false);
    source_node.param("clearing", clearing, false);
    source_node.param("marking", marking, true);

    if (!(data_type == "PointCloud2" || data_type == "PointCloud" || data_type == "LaserScan"))
    {
      ROS_FATAL("Only topics that use point clouds or laser scans are currently supported");
      throw std::runtime_error("Only topics that use point clouds or laser scans are currently supported");
    }

    std::string raytrace_range_param_name, obstacle_range_param_name;

    // get the obstacle range for the sensor
    double obstacle_range = 2.5;
    if (source_node.searchParam("obstacle_range", obstacle_range_param_name))
    {
      source_node.getParam(obstacle_range_param_name, obstacle_range);
    }

    // get the raytrace range for the sensor
    double raytrace_range = 3.0;
    if (source_node.searchParam("raytrace_range", raytrace_range_param_name))
    {
      source_node.getParam(raytrace_range_param_name, raytrace_range);
    }

    ROS_DEBUG("Creating an observation buffer for source %s, topic %s, frame %s", source.c_str(), topic.c_str(),
              sensor_frame.c_str());

    // create an observation buffer
    observation_buffers_.push_back(
        boost::shared_ptr < costmap_2d::ObservationBuffer
            > (new costmap_2d::ObservationBuffer(topic, observation_keep_time, expected_update_rate, min_obstacle_height,
                                     max_obstacle_height, obstacle_range, raytrace_range, *tf_, global_frame_,
                                     sensor_frame, transform_tolerance)));

    // check if we'll add this buffer to our marking observation buffers
    if (marking)
      marking_buffers_.push_back(observation_buffers_.back());

    // check if we'll also add this buffer to our clearing observation buffers
    if (clearing)
      clearing_buffers_.push_back(observation_buffers_.back());

    ROS_DEBUG(
        "Created an observation buffer for source %s, topic %s, global frame: %s, "
        "expected update rate: %.2f, observation persistence: %.2f",
        source.c_str(), topic.c_str(), global_frame_.c_str(), expected_update_rate, observation_keep_time);

    // create a callback for the topic
    if (data_type == "LaserScan")
    {
      boost::shared_ptr < message_filters::Subscriber<sensor_msgs::LaserScan>
          > sub(new message_filters::Subscriber<sensor_msgs::LaserScan>(g_nh, topic, 50));

      boost::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::LaserScan> > filter(
        new tf2_ros::MessageFilter<sensor_msgs::LaserScan>(*sub, *tf_, global_frame_, 50, g_nh));

      if (inf_is_valid)
      {
        filter->registerCallback(boost::bind(&ObstacleLayerLaserUndistortion::laserScanValidInfCallback, this, _1,
                                            observation_buffers_.back()));
      }
      else
      {
        filter->registerCallback(boost::bind(&ObstacleLayerLaserUndistortion::laserScanCallback, this, _1, observation_buffers_.back()));
      }

      observation_subscribers_.push_back(sub);
      observation_notifiers_.push_back(filter);

      observation_notifiers_.back()->setTolerance(ros::Duration(0.05));

      source_node.param("lidar_rotating", rotating_, true);
      if(rotating_)
      {
          bool lidar_rotation_direction(true);
          source_node.param("lidar_rotation_direction", lidar_rotation_direction, true);
          projector_.setRotationDirection(lidar_rotation_direction);
          std::cout<<"Lidar "<<source<<" rotates "<<(lidar_rotation_direction?"anticlockwise":"clockwise")<<"."<<std::endl;
      }
      else
          std::cout<<"Lidar "<<source<<" is solid."<<std::endl;
    }
    else if (data_type == "PointCloud")
    {
      boost::shared_ptr < message_filters::Subscriber<sensor_msgs::PointCloud>
          > sub(new message_filters::Subscriber<sensor_msgs::PointCloud>(g_nh, topic, 50));

      if (inf_is_valid)
      {
       ROS_WARN("obstacle_layer: inf_is_valid option is not applicable to PointCloud observations.");
      }

        boost::shared_ptr < tf2_ros::MessageFilter<sensor_msgs::PointCloud>
        > filter(new tf2_ros::MessageFilter<sensor_msgs::PointCloud>(*sub, *tf_, global_frame_, 50, g_nh));
        filter->registerCallback(
          boost::bind(&ObstacleLayer::pointCloudCallback, this, _1, observation_buffers_.back()));

      observation_subscribers_.push_back(sub);
      observation_notifiers_.push_back(filter);
    }
    else
    {
      boost::shared_ptr < message_filters::Subscriber<sensor_msgs::PointCloud2>
          > sub(new message_filters::Subscriber<sensor_msgs::PointCloud2>(g_nh, topic, 50));

      if (inf_is_valid)
      {
       ROS_WARN("obstacle_layer: inf_is_valid option is not applicable to PointCloud observations.");
      }

      boost::shared_ptr < tf2_ros::MessageFilter<sensor_msgs::PointCloud2>
      > filter(new tf2_ros::MessageFilter<sensor_msgs::PointCloud2>(*sub, *tf_, global_frame_, 50, g_nh));
      filter->registerCallback(
          boost::bind(&ObstacleLayer::pointCloud2Callback, this, _1, observation_buffers_.back()));

      observation_subscribers_.push_back(sub);
      observation_notifiers_.push_back(filter);
    }

    if (sensor_frame != "")
    {
      std::vector < std::string > target_frames;
      target_frames.push_back(global_frame_);
      target_frames.push_back(sensor_frame);
      observation_notifiers_.back()->setTargetFrames(target_frames);
    }
  }

  dsrv_ = nullptr;
  setupDynamicReconfigure(nh);
}

void ObstacleLayerLaserUndistortion::laserScanCallback(const sensor_msgs::LaserScanConstPtr& message,const boost::shared_ptr<costmap_2d::ObservationBuffer>& buffer)
{
  // project the laser into a point cloud
  sensor_msgs::PointCloud2 cloud;
  cloud.header = message->header;

  // project the scan into a point cloud
  if(rotating_)
  {
      try
      {
        projector_.transformLaserScanToPointCloud(message->header.frame_id, *message, cloud, global_frame_, *tf_);
      }
      catch (tf2::TransformException &ex)
      {
        ROS_WARN("High fidelity enabled, but TF returned a transform exception to frame %s: %s", global_frame_.c_str(),
                 ex.what());
        projector_.projectLaser(*message, cloud);
      }
  }
  else
      projector_.projectLaser(*message, cloud);

  // buffer the point cloud
  buffer->lock();
  buffer->bufferCloud(cloud);
  buffer->unlock();
}

void ObstacleLayerLaserUndistortion::laserScanValidInfCallback(const sensor_msgs::LaserScanConstPtr& raw_message,const boost::shared_ptr<costmap_2d::ObservationBuffer>& buffer)
{
  // Filter positive infinities ("Inf"s) to max_range.
  float epsilon = 0.0001;  // a tenth of a millimeter
  sensor_msgs::LaserScan message = *raw_message;
  for (size_t i = 0; i < message.ranges.size(); i++)
  {
    float range = message.ranges[ i ];
    if (!std::isfinite(range) && range > 0)
    {
      message.ranges[ i ] = message.range_max - epsilon;
    }
  }

  // project the laser into a point cloud
  sensor_msgs::PointCloud2 cloud;
  cloud.header = message.header;

  // project the scan into a point cloud
  if(rotating_)
  {
      try
      {
        projector_.transformLaserScanToPointCloud(message.header.frame_id, message, cloud, global_frame_, *tf_);
      }
      catch (tf2::TransformException &ex)
      {
        ROS_WARN("High fidelity enabled, but TF returned a transform exception to frame %s: %s",
                 global_frame_.c_str(), ex.what());
        projector_.projectLaser(message, cloud);
      }
  }
  else
      projector_.projectLaser(message, cloud);

  // buffer the point cloud
  buffer->lock();
  buffer->bufferCloud(cloud);
  buffer->unlock();
}
}
