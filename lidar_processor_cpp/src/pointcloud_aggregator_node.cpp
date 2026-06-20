// Copyright (c) 2024, RoboVerse community
// SPDX-License-Identifier: BSD-3-Clause

#include "lidar_processor_cpp/pointcloud_aggregator_node.hpp"
#include <cmath>

namespace lidar_processor_cpp
{

StatisticalFilter::StatisticalFilter(int k_neighbors, double std_ratio)
  : k_neighbors_(k_neighbors), std_ratio_(std_ratio)
{
  sor_filter_.setMeanK(k_neighbors_);
  sor_filter_.setStddevMulThresh(std_ratio_);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr StatisticalFilter::filterPoints(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud)
{
  if (input_cloud->points.size() < static_cast<size_t>(k_neighbors_)) {
    return input_cloud;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>);
  sor_filter_.setInputCloud(input_cloud);
  sor_filter_.filter(*out);
  return out;
}

PointCloudAggregatorNode::PointCloudAggregatorNode()
  : Node("pointcloud_aggregator")
{
  declareParameters();
  config_ = loadConfiguration();
  setupSubscriptions();
  setupPublishers();
  RCLCPP_INFO(this->get_logger(), "PointCloud Aggregator Node ready");
  logConfiguration();
}

void PointCloudAggregatorNode::declareParameters()
{
  this->declare_parameter("max_range",         10.0);
  this->declare_parameter("min_range",          0.15);
  this->declare_parameter("height_filter_min",  0.05);
  this->declare_parameter("height_filter_max",  0.5);
  this->declare_parameter("downsample_rate",    1);
  this->declare_parameter("publish_rate",       20.0);
}

AggregatorConfig PointCloudAggregatorNode::loadConfiguration()
{
  AggregatorConfig config;
  config.max_range         = this->get_parameter("max_range").as_double();
  config.min_range         = this->get_parameter("min_range").as_double();
  config.height_filter_min = this->get_parameter("height_filter_min").as_double();
  config.height_filter_max = this->get_parameter("height_filter_max").as_double();
  config.downsample_rate   = this->get_parameter("downsample_rate").as_int();
  config.publish_rate      = this->get_parameter("publish_rate").as_double();
  return config;
}

void PointCloudAggregatorNode::setupSubscriptions()
{
  auto qos = rclcpp::QoS(5)
    .reliability(rclcpp::ReliabilityPolicy::BestEffort)
    .history(rclcpp::HistoryPolicy::KeepLast);

  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "cloud_in", qos,
    std::bind(&PointCloudAggregatorNode::pointcloudCallback, this, std::placeholders::_1)
  );
}

void PointCloudAggregatorNode::setupPublishers()
{
  auto qos = rclcpp::QoS(5)
    .reliability(rclcpp::ReliabilityPolicy::BestEffort)
    .history(rclcpp::HistoryPolicy::KeepLast);

  filtered_pub_    = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/pointcloud/filtered", qos);
  downsampled_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/pointcloud/downsampled", qos);
}

void PointCloudAggregatorNode::pointcloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  try {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    if (cloud->points.empty()) return;

    auto filtered = applyFilters(cloud);
    if (filtered->points.empty()) return;

    sensor_msgs::msg::PointCloud2 out;
    pcl::toROSMsg(*filtered, out);
    out.header = msg->header;
    filtered_pub_->publish(out);

    if (downsampled_pub_->get_subscription_count() > 0 && config_.downsample_rate > 1) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr ds(new pcl::PointCloud<pcl::PointXYZ>);
      ds->points.reserve(filtered->points.size() / config_.downsample_rate + 1);
      for (size_t i = 0; i < filtered->points.size(); i += config_.downsample_rate) {
        ds->points.push_back(filtered->points[i]);
      }
      ds->width = ds->points.size();
      ds->height = 1;
      ds->is_dense = true;
      sensor_msgs::msg::PointCloud2 ds_msg;
      pcl::toROSMsg(*ds, ds_msg);
      ds_msg.header = msg->header;
      downsampled_pub_->publish(ds_msg);
    }

  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "pointcloudCallback error: %s", e.what());
  }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudAggregatorNode::applyFilters(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>);
  out->points.reserve(input_cloud->points.size());

  const float min_r2 = static_cast<float>(config_.min_range * config_.min_range);
  const float max_r2 = static_cast<float>(config_.max_range * config_.max_range);
  const float h_min  = static_cast<float>(config_.height_filter_min);
  const float h_max  = static_cast<float>(config_.height_filter_max);

  for (const auto& pt : input_cloud->points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
    if (pt.z < h_min || pt.z > h_max) continue;
    const float d2 = pt.x * pt.x + pt.y * pt.y;
    if (d2 < min_r2 || d2 > max_r2) continue;
    out->points.push_back(pt);
  }

  out->width    = out->points.size();
  out->height   = 1;
  out->is_dense = true;
  return out;
}

void PointCloudAggregatorNode::publishCallback() {}

void PointCloudAggregatorNode::logConfiguration()
{
  RCLCPP_INFO(this->get_logger(), "  Range : %.2f-%.2fm", config_.min_range, config_.max_range);
  RCLCPP_INFO(this->get_logger(), "  Height: %.2f-%.2fm", config_.height_filter_min, config_.height_filter_max);
  RCLCPP_INFO(this->get_logger(), "  StatisticalOutlierRemoval: DISABLED");
}

}  // namespace lidar_processor_cpp

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<lidar_processor_cpp::PointCloudAggregatorNode>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    std::cerr << "Fatal: " << e.what() << std::endl;
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
