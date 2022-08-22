#pragma once
#ifndef _MAPPER_HPP_
#define _MAPPER_HPP_

// #include <chrono>


#include "rclcpp/rclcpp.hpp"
#include <rmw/qos_profiles.h>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
// #include <geometry_msgs/msg/point.hpp>
// #include <geometry_msgs/msg/vector3.hpp>
// 
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

//#include <tf2/buffer_core.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>

#include <mapper/transforms.hpp>

using PCLPoint = pcl::PointXYZ;

using PCLPointCloud = pcl::PointCloud<PCLPoint>;

namespace ph = std::placeholders;

namespace mapper {

  class Mapper: public rclcpp::Node {
    
    public: 
      Mapper();


    protected:

      PCLPointCloud::Ptr ptr_full_pc;

      std::string m_worldFrameId;
      std::shared_ptr<tf2_ros::Buffer> buffer_;
  
      std::shared_ptr<tf2_ros::TransformListener> m_tfListener;
      rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pc_;
      
      void callback_pc(const sensor_msgs::msg::PointCloud2::SharedPtr) ;

      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_occupiedPCLPub;
  };


} // namespace mapper


#endif
