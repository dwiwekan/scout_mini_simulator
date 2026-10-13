// Keeps the points of a PointCloud2 whose height above robot_frame (base_link) is within
// [min_z, max_z], drops the rest (floor, ceiling), and thins them with a voxel grid.
// Nav2's costmap filters heights in its global frame, where the floor's z depends on who
// publishes odom (EKF: base_link z = 0, Gazebo: z = 0.18), so the crop is done here instead.
// The output keeps the input frame, so Nav2 still raytraces from the sensor origin.
#include <cmath>
#include <string>
#include <unordered_set>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class CloudHeightFilter : public rclcpp::Node
{
public:
  CloudHeightFilter()
  : Node("cloud_height_filter"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    robot_frame_ = declare_parameter<std::string>("robot_frame", "base_link");
    min_z_ = declare_parameter<double>("min_z", -0.13);      // base_link is ~0.18 m above the floor
    max_z_ = declare_parameter<double>("max_z", 0.6);
    max_range_ = declare_parameter<double>("max_range", 10.0);
    voxel_ = declare_parameter<double>("voxel_size", 0.05);   // <= 0 disables thinning

    // Gazebo, the Livox driver and the ZED wrapper publish RELIABLE. A best-effort subscriber
    // loses every multi-MB camera cloud once one UDP fragment is dropped (small rmem_max).
    const bool best_effort = declare_parameter<bool>("best_effort_input", false);
    rclcpp::QoS in_qos(5);
    if (best_effort) in_qos.best_effort(); else in_qos.reliable();

    // output is small (voxel-thinned), sensor-data QoS is what the costmap subscribes with
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("cloud_out", rclcpp::SensorDataQoS());
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "cloud_in", in_qos,
      std::bind(&CloudHeightFilter::onCloud, this, std::placeholders::_1));
  }

private:
  void onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    // sensors are rigidly mounted, so the latest transform is the right one
    tf2::Transform T;
    try {
      tf2::fromMsg(
        tf_buffer_.lookupTransform(robot_frame_, msg->header.frame_id, tf2::TimePointZero).transform, T);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", e.what());
      return;
    }

    std::vector<float> kept;
    kept.reserve(3 * 4096);
    std::unordered_set<uint64_t> occupied;
    const double max_r2 = max_range_ * max_range_;
    sensor_msgs::PointCloud2ConstIterator<float> x(*msg, "x"), y(*msg, "y"), z(*msg, "z");
    for (; x != x.end(); ++x, ++y, ++z) {
      if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) continue;
      if ((*x) * (*x) + (*y) * (*y) + (*z) * (*z) > max_r2) continue;
      const double h = (T * tf2::Vector3(*x, *y, *z)).z();
      if (h < min_z_ || h > max_z_) continue;
      if (voxel_ > 0.0) {
        // 21 bits per axis, offset so negative indices pack too
        auto idx = [this](float v) {return static_cast<uint64_t>(std::floor(v / voxel_) + (1 << 20)) & 0x1FFFFF;};
        if (!occupied.insert(idx(*x) | idx(*y) << 21 | idx(*z) << 42).second) continue;
      }
      kept.insert(kept.end(), {*x, *y, *z});
    }

    sensor_msgs::msg::PointCloud2 out;
    out.header = msg->header;
    sensor_msgs::PointCloud2Modifier mod(out);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(kept.size() / 3);
    sensor_msgs::PointCloud2Iterator<float> ox(out, "x"), oy(out, "y"), oz(out, "z");
    for (size_t i = 0; i < kept.size(); i += 3, ++ox, ++oy, ++oz) {
      *ox = kept[i]; *oy = kept[i + 1]; *oz = kept[i + 2];
    }
    pub_->publish(out);
  }

  std::string robot_frame_;
  double min_z_, max_z_, max_range_, voxel_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CloudHeightFilter>());
  rclcpp::shutdown();
  return 0;
}
