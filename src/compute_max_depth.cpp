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
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

/*
 * Helper node which subscribes to raw point clouds and publishes
 * the maximum depth value (x) present in the cloud
 */
class ComputeMaxDepth: public rclcpp::Node
{
public:
  ComputeMaxDepth()
  : Node("compute_max_depth")
  {
    //
    // Create a publisher for the max depth value
    //
    publisher_ = this->create_publisher<std_msgs::msg::Float64>(
      "max_depth",
      rclcpp::SensorDataQoS());

    //
    // Register to be notified when messages arrive on topic /camera/cloud
    //
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/camera/cloud",
      rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {

        //
        // Convert the ROS message to a PCL datastructure
        //
        pcl::PCLPointCloud2 pc2;
        pcl_conversions::toPCL(*msg, pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr
          point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pc2, *point_cloud);

        // Compute the min and max depth values in the cloud
        pcl::PointXYZ min, max;
        pcl::getMinMax3D(*point_cloud, min, max);

        //
        // Print some information to the screen
        //
        RCLCPP_INFO(this->get_logger(),
          "Got a pointcloud (%d, %d) with max x = %f",
          point_cloud->height,
          point_cloud->width,
          max.x);

        //
        // Publish the result to the topic
        //
        auto message = std_msgs::msg::Float64();
        message.data = max.x;
        this->publisher_->publish(message);
      });
  }
private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  auto camera = std::make_shared<ifm3d_ros2::CameraNode>(options);
  exec.add_node(camera->get_node_base_interface());
  auto preprocess_data = std::make_shared<ComputeMaxDepth>();
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

