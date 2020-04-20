#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "ifm3d_ros2/camera_node.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "cv_bridge/cv_bridge.h"

/*
 * Helper node which subscribes point clouds and publishes the mean depth
 */
class ComputeMeanDepth: public rclcpp::Node
{
public:
  ComputeMeanDepth()
  : Node("compute_mean_depth")
  {
    //
    // Create a publisher for 64-bit floating point numbers
    //
    publisher_ = this->create_publisher<std_msgs::msg::Float64>(
        "mean_depth", 10);

    //
    // Register to be notified when messages arrive on topic /camera/xyz_image
    //
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/xyz_image",
      rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::SharedPtr msg) {

        //
        // Convert the ROS message to an OpenCV datastructure
        //
        auto cloud = cv_bridge::toCvShare(msg);

        //
        // Compute the average depth value (channel 0)
        //
        cv::Mat xyz[3];
        cv::split(cloud->image, xyz);
        cv::Scalar mean_depth_value = cv::mean(xyz[0]);

        //
        // Print some information to the screen
        //
        RCLCPP_INFO(this->get_logger(),
          "Got a pointcloud (%d, %d, %d) mean_depth = %f!",
          cloud->image.rows,
          cloud->image.cols,
          cloud->image.channels(),
          mean_depth_value[0]);

        //
        // Publish the result to the topic
        //
        auto message = std_msgs::msg::Float64();
        message.data = mean_depth_value[0];
        this->publisher_->publish(message);
      });
  }
private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  auto camera = std::make_shared<ifm3d_ros2::CameraNode>(options);
  exec.add_node(camera->get_node_base_interface());
  auto compute_mean_depth = std::make_shared<ComputeMeanDepth>();
  exec.add_node(compute_mean_depth);

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

