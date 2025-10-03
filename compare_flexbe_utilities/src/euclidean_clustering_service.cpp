#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>

#include <pcl_msgs/msg/point_indices.hpp>

#include "compare_flexbe_utilities/srv/euclidean_clustering.hpp"

using compare_flexbe_utilities::srv::EuclideanClustering;

class EuclideanClusteringServiceNode : public rclcpp::Node
{
public:
  EuclideanClusteringServiceNode() : Node("euclidean_clustering_service_node")
  {
    service_ = this->create_service<EuclideanClustering>(
      "euclidean_clustering",
      std::bind(&EuclideanClusteringServiceNode::handle_request, this,
                std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(get_logger(), "EuclideanClustering service ready.");
  }

private:
  rclcpp::Service<EuclideanClustering>::SharedPtr service_;

  static inline bool hasFiniteXYZ(const pcl::PointXYZ& p)
  {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
  }

  void handle_request(
    const std::shared_ptr<EuclideanClustering::Request> req,
    std::shared_ptr<EuclideanClustering::Response> res)
  {
    // Convert input to PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(req->input, *cloud);

    // Fast exit on empty input
    if (cloud->empty()) {
      RCLCPP_WARN(get_logger(), "Input cloud is empty.");
      res->target_cluster_indices = pcl_msgs::msg::PointIndices();
      res->obstacle_cluster_indices.clear();
      return;
    }

    // Build KD-Tree
    auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
    tree->setInputCloud(cloud);

    // Euclidean cluster extraction
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(req->cluster_tolerance > 0.f ? req->cluster_tolerance : 0.02f);
    ec.setMinClusterSize(req->min_cluster_size > 0 ? req->min_cluster_size : 100);
    ec.setMaxClusterSize(req->max_cluster_size > 0 ? req->max_cluster_size : 25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    if (cluster_indices.empty()) {
      RCLCPP_INFO(get_logger(), "No clusters found.");
      res->target_cluster_indices = pcl_msgs::msg::PointIndices();
      res->obstacle_cluster_indices.clear();
      return;
    }

    // Compute cluster centroids and sort clusters by squared distance to camera
    const auto& camera_pose = req->camera_pose.pose.position;

    // Create struct to hold cluster indices and distance from camera for sorting
    struct ClusterWithDistance
    {
      pcl::PointIndices indices;
      double dist2 = 0.0;
    };

    std::vector<ClusterWithDistance> clusters;
    clusters.reserve(cluster_indices.size());

    // Compute clusters and put into vector of ClusterWithDistance for sorting
    for (const auto& ci : cluster_indices) {
      // Compute centroid (skip NaNs)
      double cx = 0.0, cy = 0.0, cz = 0.0;
      size_t valid = 0;
      for (int idx : ci.indices) {
        const auto& p = (*cloud)[static_cast<size_t>(idx)];
        if (!hasFiniteXYZ(p)) continue;
        cx += static_cast<double>(p.x);
        cy += static_cast<double>(p.y);
        cz += static_cast<double>(p.z);
        ++valid;
      }
      if (valid == 0) continue;

      cx /= static_cast<double>(valid);
      cy /= static_cast<double>(valid);
      cz /= static_cast<double>(valid);

      const double dx = cx - static_cast<double>(camera_pose.x);
      const double dy = cy - static_cast<double>(camera_pose.y);
      const double dz = cz - static_cast<double>(camera_pose.z);

      // Avoid sqrt(d) because it is not necessary for this comparison
      // In order to save this value as a response, sqrt() first
      clusters.push_back(ClusterWithDistance{ci, dx*dx + dy*dy + dz*dz});

      // // Example for logging cluster distance
      // double dist2 = x*dx + dy*dy + dz*dz;
      // double dist_m = std::sqrt(dist2);
      // RCLCPP_INFO(get_logger(), "Cluster is %.3f meters from camera.", dist_m);
    }

    if (clusters.empty()) {
      RCLCPP_INFO(get_logger(), "All clusters invalid after centroid filtering.");
      res->target_cluster_indices = pcl_msgs::msg::PointIndices();
      res->obstacle_cluster_indices.clear();
      return;
    }

    std::sort(clusters.begin(), clusters.end(),
              [](const ClusterWithDistance& a, const ClusterWithDistance& b){
                return a.dist2 < b.dist2;  // nearest first
              });

    // Prepare output headers
    const auto& header = req->input.header;

    // Target = nearest cluster
    res->target_cluster_indices.header = header;
    res->target_cluster_indices.indices = clusters.front().indices.indices;

    // Obstacles = all remaining clusters, nearest->furthest order (excluding target)
    res->obstacle_cluster_indices.clear();
    res->obstacle_cluster_indices.reserve(
      clusters.size() > 0 ? (clusters.size() - 1) : 0);

    for (size_t i = 1; i < clusters.size(); ++i) {
      pcl_msgs::msg::PointIndices pi;
      pi.header = header;
      pi.indices = clusters[i].indices.indices;
      res->obstacle_cluster_indices.emplace_back(std::move(pi));
    }

    RCLCPP_INFO(get_logger(),
                "Clusters found: %zu (target + %zu obstacles).",
                clusters.size(),
                res->obstacle_cluster_indices.size());
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EuclideanClusteringServiceNode>());
  rclcpp::shutdown();
  return 0;
}