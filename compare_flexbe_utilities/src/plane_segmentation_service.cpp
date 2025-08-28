#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/extract_indices.h>  // optional (not needed for final removal in original)
#include <pcl/common/common.h>

#include <pcl/search/kdtree.h>  // not strictly needed here
#include <Eigen/Dense>

#include "pcl_msgs/msg/point_indices.hpp"
#include "compare_flexbe_utilities/srv/plane_segmentation.hpp"  // adjust to your package

using compare_flexbe_utilities::srv::PlaneSegmentation;

class PlaneSegmentationServiceNode : public rclcpp::Node
{
public:
  PlaneSegmentationServiceNode() : Node("plane_segmentation_service_node")
  {
    service_ = this->create_service<PlaneSegmentation>(
      "plane_segmentation",
      std::bind(&PlaneSegmentationServiceNode::handle_request, this,
                std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(get_logger(), "PlaneSegmentation service ready.");
  }

private:
  rclcpp::Service<PlaneSegmentation>::SharedPtr service_;

  void handle_request(
    const std::shared_ptr<PlaneSegmentation::Request> req,
    std::shared_ptr<PlaneSegmentation::Response> res)
  {
    // Convert input to PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_orig(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(req->input, *cloud_orig);

    if (cloud_orig->empty()) {
      RCLCPP_WARN(get_logger(), "Input cloud is empty.");
      res->without_plane = req->input;
      res->plane_indices.indices.clear();
      res->plane_coefficients = {0.0f, 0.0f, 0.0f, 0.0f};
      res->inlier_count = 0;
      return;
    }

    // Optionally downsample for plane detection
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_for_ransac(new pcl::PointCloud<pcl::PointXYZ>());
    if (req->use_voxel) {
      pcl::VoxelGrid<pcl::PointXYZ> voxel;
      voxel.setInputCloud(cloud_orig);
      float leaf = (req->leaf_size > 0.0f) ? req->leaf_size : 0.01f;
      voxel.setLeafSize(leaf, leaf, leaf);
      voxel.filter(*cloud_for_ransac);
    } else {
      cloud_for_ransac = cloud_orig;
    }

    // Guard again
    if (cloud_for_ransac->empty()) {
      RCLCPP_WARN(get_logger(), "Cloud for RANSAC is empty after voxel filtering.");
      res->without_plane = req->input;
      res->plane_indices.indices.clear();
      res->plane_coefficients = {0.0f, 0.0f, 0.0f, 0.0f};
      res->inlier_count = 0;
      return;
    }

    // RANSAC plane segmentation on cloud_for_ransac
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(req->max_iterations > 0 ? req->max_iterations : 1000);
    float ransac_dist = (req->distance_threshold > 0.0f) ? req->distance_threshold : 0.01f;
    seg.setDistanceThreshold(ransac_dist);
    seg.setInputCloud(cloud_for_ransac);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers_voxel(new pcl::PointIndices());
    seg.segment(*inliers_voxel, *coefficients);

    if (coefficients->values.size() < 4 || inliers_voxel->indices.empty()) {
      RCLCPP_WARN(get_logger(), "No plane found.");
      // Return original cloud unchanged
      res->without_plane = req->input;
      res->plane_indices.indices.clear();
      res->plane_coefficients = {0.0f, 0.0f, 0.0f, 0.0f};
      res->inlier_count = 0;
      return;
    }

    // Plane coefficients [a b c d]
    const float a = coefficients->values[0];
    const float b = coefficients->values[1];
    const float c = coefficients->values[2];
    const float d = coefficients->values[3];

    // Re-compute inliers ON THE ORIGINAL CLOUD using the plane equation
    // Distance from point (x,y,z) to plane ax+by+cz+d=0 is |ax+by+cz+d| / sqrt(a^2+b^2+c^2)
    const double denom = std::sqrt(a*a + b*b + c*c);
    const double dist_thresh = static_cast<double>(ransac_dist); // reuse same threshold (tweak if desired)

    pcl_msgs::msg::PointIndices orig_inliers;
    orig_inliers.header = req->input.header;
    orig_inliers.indices.reserve(cloud_orig->size());

    for (std::size_t i = 0; i < cloud_orig->size(); ++i) {
      const auto& p = cloud_orig->points[i];
      const double dist = std::abs(a * p.x + b * p.y + c * p.z + d) / denom;
      if (dist <= dist_thresh) {
        orig_inliers.indices.push_back(static_cast<int>(i));
      }
    }

    // Build "without_plane" by copying all non-inliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_plane(new pcl::PointCloud<pcl::PointXYZ>());
    cloud_no_plane->reserve(cloud_orig->size() - orig_inliers.indices.size());

    // Create a fast boolean mask
    std::vector<uint8_t> is_inlier(cloud_orig->size(), 0);
    for (int idx : orig_inliers.indices) {
      if (idx >= 0 && static_cast<std::size_t>(idx) < is_inlier.size()) is_inlier[idx] = 1;
    }
    for (std::size_t i = 0; i < cloud_orig->size(); ++i) {
      if (!is_inlier[i]) cloud_no_plane->push_back(cloud_orig->points[i]);
    }

    // Fill response
    sensor_msgs::msg::PointCloud2 ros_no_plane;
    pcl::toROSMsg(*cloud_no_plane, ros_no_plane);
    ros_no_plane.header = req->input.header;

    res->without_plane = std::move(ros_no_plane);
    res->plane_indices = std::move(orig_inliers);
    res->plane_coefficients = {a, b, c, d};
    res->inlier_count = static_cast<int32_t>(res->plane_indices.indices.size());

    RCLCPP_INFO(get_logger(),
      "Plane found: coeffs [%.4f %.4f %.4f %.4f], inliers (orig) = %d, output size = %zu",
      a, b, c, d, res->inlier_count, static_cast<size_t>(cloud_no_plane->size()));
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlaneSegmentationServiceNode>());
  rclcpp::shutdown();
  return 0;
}