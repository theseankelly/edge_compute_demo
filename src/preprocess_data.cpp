#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "ifm3d_ros2/camera_node.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "pcl_conversions/pcl_conversions.h"

#include <pcl/conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

/*
 * Helper node which subscribes to raw point clouds and publishes
 *  preprocessed, filtered point clouds
 */
class PreprocessData: public rclcpp::Node
{
public:
  PreprocessData()
  : Node("preprocess_data")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "filtered_cloud",
      rclcpp::SensorDataQoS());

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/camera/cloud",
      rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        //
        // Convert the data from the ROS message into a PCL format
        // for use with PCL library APIS
        //
        pcl::PCLPointCloud2 pc2;
        pcl_conversions::toPCL(*msg, pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr
          point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pc2, *point_cloud);

        RCLCPP_INFO(this->get_logger(), "Got a pointcloud (%d, %d)",
          point_cloud->height, point_cloud->width);

        // Drop points beyond 2.0 m
        pcl::PointCloud<pcl::PointXYZ>::Ptr
          cloud_chopped(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> passthrough_filter;
        passthrough_filter.setInputCloud(point_cloud);
        passthrough_filter.setFilterFieldName("x");
        passthrough_filter.setFilterLimits(0, 2.0);
        passthrough_filter.filter(*cloud_chopped);

        // De-noise with a statistical outlier filter
        pcl::PointCloud<pcl::PointXYZ>::Ptr
          cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud_chopped);
        sor.setMeanK(20);
        sor.setStddevMulThresh(0.5);
        sor.filter(*cloud_filtered);

        // Publish the resulting cloud!
        pcl::PCLPointCloud2 pc2_out;
        pcl::toPCLPointCloud2(*cloud_filtered, pc2_out);
        sensor_msgs::msg::PointCloud2 message;
        pcl_conversions::fromPCL(pc2_out, message);
        publisher_->publish(message);
      });
  }
private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  auto camera = std::make_shared<ifm3d_ros2::CameraNode>(options);
  exec.add_node(camera->get_node_base_interface());
  auto preprocess_data = std::make_shared<PreprocessData>();
  exec.add_node(preprocess_data);

  // Activate the camera (a managed node)
  //
  // This is a bit of a hack and should be done through a launch script,
  // but there's no python (and thus no ros launch) available embedded on the
  // O3D303.
  camera->trigger_transition(
    lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  camera->trigger_transition(
    lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  exec.spin();
  rclcpp::shutdown();
  return 0;
}

