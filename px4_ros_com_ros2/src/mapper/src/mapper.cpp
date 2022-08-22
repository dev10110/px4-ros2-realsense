//#include <cstdio>


#include <mapper/mapper.hpp>


namespace mapper{

Mapper::Mapper():
	Node("mapper")
	, m_worldFrameId("world")
        , ptr_full_pc(new PCLPointCloud)
{
    RCLCPP_INFO(this->get_logger(), "mapper has begun!");

    sub_pc_ = this -> create_subscription<sensor_msgs::msg::PointCloud2>(
        "/camera/depth/color/points", 10, std::bind(&Mapper::callback_pc, this, ph::_1));


    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    this->buffer_ = std::make_shared<tf2_ros::Buffer>(clock);
    this->buffer_ ->setUsingDedicatedThread(true);
    this->m_tfListener = std::make_shared<tf2_ros::TransformListener>(*buffer_, this, false);

    m_worldFrameId = this->declare_parameter("world_frame_id", m_worldFrameId);

    rclcpp::QoS qos(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

    this->m_occupiedPCLPub = this->create_publisher<sensor_msgs::msg::PointCloud2>("occupied_pcl", qos);
}


void Mapper::callback_pc(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

  RCLCPP_INFO(this->get_logger(), "I received data");

  // convert ros msg into pointcloud
  PCLPointCloud pc;
  pcl::fromROSMsg(*msg, pc);

  RCLCPP_INFO(this->get_logger(), "My pc has %d points", pc.size());

  // get the transform from sensor frame to world frame
  Eigen::Matrix4f sensorToWorld;
  {
    geometry_msgs::msg::TransformStamped tf;
    if (!this->buffer_->canTransform ( m_worldFrameId, msg->header.frame_id,msg->header.stamp)){
      RCLCPP_WARN(this->get_logger(), "failed to do transform");
      return;
    }

    tf = this->buffer_->lookupTransform( m_worldFrameId, msg->header.frame_id, msg->header.stamp);

    sensorToWorld = pcl_ros::transformAsMatrix(tf);

    
  }

  RCLCPP_INFO(this->get_logger(), "successfully got transform");


  // transform the pointcloud to the right frame
  pcl::transformPointCloud(pc, pc, sensorToWorld);

  // run some filters
  pcl::PassThrough<PCLPoint> pass_z;
  pass_z.setFilterFieldName("z");
  pass_z.setFilterLimits(-0.1, 2.0);
  pass_z.setInputCloud(pc.makeShared());
  pass_z.filter(pc);

 // // merge with existing point cloud
 (*ptr_full_pc) += pc;


 // // apply filter to downsample entire map
 // pcl::VoxelGrid<PCLPointCloud> voxel_filt;
 // voxel_filt.setInputCloud (ptr_full_pc);
 // voxel_filt.setLeafSize (0.1f, 0.1f, 0.1f);
 // voxel_filt.filter (*ptr_full_pc);


  // try voxel grid filtering


  pcl::VoxelGrid<PCLPoint> voxel_filt;
  //voxel_filt.setInputCloud(pc.makeShared());
  voxel_filt.setInputCloud(ptr_full_pc);
  voxel_filt.setLeafSize(0.1f, 0.1f, 0.1f);
  voxel_filt.filter(*ptr_full_pc);

  RCLCPP_INFO(this->get_logger(), "new pc size: %d, full pc size: %d", pc.size(), ptr_full_pc->size());


  // publish as pcl to ros
  sensor_msgs::msg::PointCloud2 out_msg;
  pcl::toROSMsg<PCLPoint>(*ptr_full_pc, out_msg);

  out_msg.header.stamp = msg->header.stamp;
  out_msg.header.frame_id = m_worldFrameId;

  m_occupiedPCLPub -> publish(out_msg);

}


} // namespace mapper

int main(int argc, char * argv[]){

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mapper::Mapper>());
  rclcpp::shutdown();
  return 0;
}
