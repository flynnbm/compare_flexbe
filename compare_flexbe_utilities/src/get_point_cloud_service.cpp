#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/wait_for_message.hpp>

#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include "compare_flexbe_utilities/srv/get_point_cloud.hpp"

class GetPointCloudServiceNode : public rclcpp::Node {
public:
  GetPointCloudServiceNode()
  : rclcpp::Node("get_point_cloud_service"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // Create a service server on "/get_cloud".
    service_ = this->create_service<compare_flexbe_utilities::srv::GetPointCloud>(
      "/get_point_cloud",
      std::bind(&GetPointCloudServiceNode::handle_get_cloud, this,
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  }

private:
  rclcpp::Service<compare_flexbe_utilities::srv::GetPointCloud>::SharedPtr service_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Service callback signature: (request_header, request, response)
  void handle_get_cloud(const std::shared_ptr<rmw_request_id_t> /*request_header*/,
                        const std::shared_ptr<compare_flexbe_utilities::srv::GetPointCloud::Request> req,
                        std::shared_ptr<compare_flexbe_utilities::srv::GetPointCloud::Response> res)
  {
    RCLCPP_INFO(this->get_logger(), "Received /get_point_cloud service call");

    // Create a PointCloud2 object to hold the message we will get from the cloud topic
    sensor_msgs::msg::PointCloud2 cloud;
    const std::string topic = req->camera_topic;  // grt camera_topic from request

    // Build your QoS
    rclcpp::QoS qos(10);
    qos.reliability(rclcpp::ReliabilityPolicy::Reliable);

    // Make a temporary subscription; no executor spin, so the callback will not run.
    auto sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      topic, qos,
      [](sensor_msgs::msg::PointCloud2::SharedPtr) { /* no-op */ });

    // Convert float seconds → std::chrono duration
    // (wait_for_message accepts any std::chrono::duration)
    auto timeout = std::chrono::duration<float>(std::max(0.0f, req->timeout_sec));

    // Build context - create a handle to the ROS runtime the node is using so wait_for_message operates int he right place
    auto context = this->get_node_base_interface()->get_context();

    // wait_for_message on specified topic 
    const bool ok = rclcpp::wait_for_message<sensor_msgs::msg::PointCloud2>(cloud, sub, context, timeout);

    // Validate frame id
    if (cloud.header.frame_id.empty()) {
      res->success = false;
      res->message = "Incoming cloud has empty frame_id";
      RCLCPP_ERROR(this->get_logger(), "Cloud frame_id is empty");
      return;
    }

    // Store the cloud Header frame_id for response
    res->cloud_frame = cloud.header.frame_id;
    RCLCPP_INFO(this->get_logger(), "Got cloud in frame '%s', stamp %u.%u",
            cloud.header.frame_id.c_str(),
            cloud.header.stamp.sec, cloud.header.stamp.nanosec);

    // lookup Transform from camera_frame to target_frame
    geometry_msgs::msg::TransformStamped tf_cam_in_target;
    try {
      tf_cam_in_target = tf_buffer_.lookupTransform(
        req->target_frame,                  // target
        cloud.header.frame_id,              // source
        cloud.header.stamp,                 // time cloud arrives
        tf2::durationFromSec(req->timeout_sec));
    } catch (tf2::TransformException & ex) {
      res->success = false;
      res->message = std::string("Transform error: ") + ex.what();
      return;
    }

    // Transform the cloud
    sensor_msgs::msg::PointCloud2 cloud_out_msg;
    try {
      tf2::doTransform(cloud, cloud_out_msg, tf_cam_in_target);
    } catch (const std::exception &e) {
      res->success = false;
      res->message = std::string("Cloud transform failed: ") + e.what();
      return;
    }

    // Store transformed cloud as response
    res->cloud_out = cloud_out_msg;

    // Convert PointCloud2 to PCL PointCloud to transform and optionally save pcd file
    pcl::PointCloud<pcl::PointXYZRGB> pcl_in;
    pcl::fromROSMsg(cloud, pcl_in);

    // Save pointcloud to a pcd file
    std::string path = "/home/csrobot/Desktop/test_pcd.pcd";
    int ret = pcl::io::savePCDFileBinary(path, pcl_in);
    (void)ret;          // silence unused variable warning
    RCLCPP_INFO(get_logger(), "Saved cloud to test_cloud.pcd.");

    // --- Camera pose in target_frame (from the same transform) ---
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = cloud.header.stamp;
    pose.header.frame_id = req->target_frame;
    pose.pose.position.x = tf_cam_in_target.transform.translation.x;
    pose.pose.position.y = tf_cam_in_target.transform.translation.y;
    pose.pose.position.z = tf_cam_in_target.transform.translation.z;
    pose.pose.orientation = tf_cam_in_target.transform.rotation;
    res->camera_pose = pose;

    if (ok) {
      // Fill response fields before sending
      res->success = true;
      res->message = "Got point cloud: " +
                     std::to_string(cloud.width) + "x" +
                     std::to_string(cloud.height);
      RCLCPP_INFO(this->get_logger(), "Cloud received; responding.");
    } else {
      // Do something if it fails
      res->success = false;
      res->message = "Timed out waiting for " + topic;
      RCLCPP_WARN(this->get_logger(), "Timeout waiting for %s", topic.c_str());
    }
    // ------------------------------------------
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GetPointCloudServiceNode>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}