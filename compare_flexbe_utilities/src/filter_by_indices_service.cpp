#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_msgs/msg/point_indices.hpp>

#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

#include "compare_flexbe_utilities/srv/filter_by_indices.hpp"

using FilterByIndices = compare_flexbe_utilities::srv::FilterByIndices;

class FilterByIndicesServer : public rclcpp::Node
{
public:
  FilterByIndicesServer()
  : Node("filter_by_indices_server")
  {
    srv_ = this->create_service<FilterByIndices>(
      "/filter_by_indices",
      std::bind(&FilterByIndicesServer::handle_request, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "FilterByIndices service ready on %s/filter",
                this->get_fully_qualified_name());
  }

private:
  void handle_request(
      const std::shared_ptr<FilterByIndices::Request> req,
      std::shared_ptr<FilterByIndices::Response> res)
  {
    const auto & cloud_msg = req->input_cloud;
    const auto & idx_msg   = req->target_indices;

    // Quick sanity checks
    if (cloud_msg.width == 0 || cloud_msg.height == 0 || cloud_msg.data.empty()) {
      RCLCPP_WARN(get_logger(), "Input cloud is empty; returning empty cloud.");
      res->filtered_cloud = sensor_msgs::msg::PointCloud2();
      res->filtered_cloud.header = cloud_msg.header;
      return;
    }
    if (idx_msg.indices.empty()) {
      RCLCPP_WARN(get_logger(), "Indices list is empty; returning empty cloud.");
      res->filtered_cloud = sensor_msgs::msg::PointCloud2();
      res->filtered_cloud.header = cloud_msg.header;
      return;
    }

    // Convert ROS2 -> PCL (use PCLPointCloud2 to preserve arbitrary fields)
    pcl::PCLPointCloud2::Ptr pcl_in(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr pcl_out(new pcl::PCLPointCloud2());
    pcl_conversions::toPCL(cloud_msg, *pcl_in);

    // Validate indices against cloud size to avoid exceptions
    const std::size_t n_points = static_cast<std::size_t>(cloud_msg.width) *
                                 static_cast<std::size_t>(cloud_msg.height);

    bool out_of_range = false;
    for (int v : idx_msg.indices) {
      if (v < 0 || static_cast<std::size_t>(v) >= n_points) {
        out_of_range = true;
        break;
      }
    }
    if (out_of_range) {
      RCLCPP_ERROR(get_logger(), "Indices contain out-of-range entries (0..%zu-1). Returning empty cloud.",
                   n_points);
      res->filtered_cloud = sensor_msgs::msg::PointCloud2();
      res->filtered_cloud.header = cloud_msg.header;
      return;
    }

    // Build PCL indices
    pcl::PointIndices::Ptr pcl_indices(new pcl::PointIndices());
    pcl_indices->indices = idx_msg.indices;

    // Extract ONLY the specified indices (setNegative(false))
    pcl::ExtractIndices<pcl::PCLPointCloud2> extractor;
    extractor.setInputCloud(pcl_in);
    extractor.setIndices(pcl_indices);
    extractor.setNegative(false);
    try {
      extractor.filter(*pcl_out);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "ExtractIndices threw: %s", e.what());
      res->filtered_cloud = sensor_msgs::msg::PointCloud2();
      res->filtered_cloud.header = cloud_msg.header;
      return;
    }

    // Convert back to ROS2 and preserve header
    pcl_conversions::fromPCL(*pcl_out, res->filtered_cloud);
    res->filtered_cloud.header = cloud_msg.header; // keep original frame and stamp
  }

  rclcpp::Service<FilterByIndices>::SharedPtr srv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FilterByIndicesServer>());
  rclcpp::shutdown();
  return 0;
}