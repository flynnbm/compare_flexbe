#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include "compare_flexbe_utilities/srv/voxel_grid_filter.hpp"

class VoxelGridFilterNode : public rclcpp::Node
{
public:
  VoxelGridFilterNode() : Node("voxel_grid_filter_node")
  {
    service_ = this->create_service<compare_flexbe_utilities::srv::VoxelGridFilter>(
      "voxel_grid_filter",
      std::bind(&VoxelGridFilterNode::handle_request, this, std::placeholders::_1, std::placeholders::_2)
    );
    RCLCPP_INFO(this->get_logger(), "Voxel Grid Filter service ready.");
  }

private:
  rclcpp::Service<compare_flexbe_utilities::srv::VoxelGridFilter>::SharedPtr service_;

  void handle_request(
    const std::shared_ptr<compare_flexbe_utilities::srv::VoxelGridFilter::Request> request,
    std::shared_ptr<compare_flexbe_utilities::srv::VoxelGridFilter::Response> response)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(request->input, *cloud_in);

    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(cloud_in);
    voxel.setLeafSize(0.01f, 0.01f, 0.01f);  // TODO: Make tunable
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>());
    voxel.filter(*cloud_out);

    pcl::toROSMsg(*cloud_out, response->filtered);
    response->filtered.header = request->input.header;
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VoxelGridFilterNode>());
  rclcpp::shutdown();
  return 0;
}