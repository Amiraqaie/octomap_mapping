#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <octomap/octomap.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using sensor_msgs::msg::PointCloud2;

class ScanFreeNode : public rclcpp::Node
{
public:
  ScanFreeNode()
  : Node("scan_free_node")
  {
    world_frame_ = declare_parameter("frame_id", "map");
    const double res = declare_parameter("resolution", 0.1);
    max_range_ = declare_parameter("max_range", 10.0);
    point_stride_ = std::max<int>(1, declare_parameter("point_stride", 1));
    min_range_ = declare_parameter("min_range", 0.1);

    tree_ = std::make_unique<octomap::OcTree>(res);   // only used for key math
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);  // own thread

    pub_ = create_publisher<PointCloud2>("scan_free_point_cloud_centers", rclcpp::QoS(2));
    sub_ = create_subscription<PointCloud2>(
      "cloud_in", rclcpp::SensorDataQoS().keep_last(2),
      std::bind(&ScanFreeNode::onCloud, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "scan_free_node: res %.2f, max_range %.1f, point_stride %d",
      res, max_range_, point_stride_);
  }

private:
  static inline uint64_t pack(const octomap::OcTreeKey & k)
  {
    return static_cast<uint64_t>(k[0]) | (static_cast<uint64_t>(k[1]) << 16) |
           (static_cast<uint64_t>(k[2]) << 32);
  }

  void onCloud(const PointCloud2::ConstSharedPtr cloud)
  {
    if (pub_->get_subscription_count() == 0) {
      return;
    }
    const auto t0 = std::chrono::steady_clock::now();

    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(
        world_frame_, cloud->header.frame_id, cloud->header.stamp,
        rclcpp::Duration::from_seconds(0.2));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", ex.what());
      return;
    }
    const Eigen::Isometry3d T = tf2::transformToEigen(tf);
    const octomap::point3d origin(
      static_cast<float>(T.translation().x()),
      static_cast<float>(T.translation().y()),
      static_cast<float>(T.translation().z()));

    // gather world points first (cheap), so the ray loop can run in parallel
    std::vector<octomap::point3d> pts;
    const size_t n = static_cast<size_t>(cloud->width) * cloud->height;
    pts.reserve(n / point_stride_ + 1);
    try {
      sensor_msgs::PointCloud2ConstIterator<float> ix(*cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iy(*cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iz(*cloud, "z");
      for (size_t i = 0; i < n; ++i, ++ix, ++iy, ++iz) {
        if (point_stride_ > 1 && (i % point_stride_) != 0) {continue;}
        if (!std::isfinite(*ix) || !std::isfinite(*iy) || !std::isfinite(*iz)) {continue;}
        const Eigen::Vector3d w = T * Eigen::Vector3d(*ix, *iy, *iz);
        pts.emplace_back(
          static_cast<float>(w.x()), static_cast<float>(w.y()), static_cast<float>(w.z()));
      }
    } catch (const std::runtime_error & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "need float32 x,y,z (%s)", e.what());
      return;
    }

    std::vector<uint64_t> free_keys, occ_keys;
    free_keys.reserve(pts.size() * 40);
    occ_keys.reserve(pts.size());

#pragma omp parallel
    {
      std::vector<uint64_t> lf, lo;
      octomap::KeyRay ray;
#pragma omp for nowait schedule(static)
      for (long i = 0; i < static_cast<long>(pts.size()); ++i) {
        octomap::point3d p = pts[i];
        const double d = (p - origin).norm();
        if (d < min_range_) {continue;}
        bool hit = true;
        if (max_range_ > 0.0 && d > max_range_) {
          p = origin + (p - origin).normalized() * static_cast<float>(max_range_);
          hit = false;
        }
        if (tree_->computeRayKeys(origin, p, ray)) {
          for (const auto & k : ray) {lf.push_back(pack(k));}
        }
        octomap::OcTreeKey ek;
        if (tree_->coordToKeyChecked(p, ek)) {
          (hit ? lo : lf).push_back(pack(ek));
        }
      }
#pragma omp critical
      {
        free_keys.insert(free_keys.end(), lf.begin(), lf.end());
        occ_keys.insert(occ_keys.end(), lo.begin(), lo.end());
      }
    }

    std::sort(free_keys.begin(), free_keys.end());
    free_keys.erase(std::unique(free_keys.begin(), free_keys.end()), free_keys.end());
    std::sort(occ_keys.begin(), occ_keys.end());
    occ_keys.erase(std::unique(occ_keys.begin(), occ_keys.end()), occ_keys.end());
    std::vector<uint64_t> result;
    result.reserve(free_keys.size());
    std::set_difference(
      free_keys.begin(), free_keys.end(), occ_keys.begin(), occ_keys.end(),
      std::back_inserter(result));

    PointCloud2 msg;
    msg.header.frame_id = world_frame_;
    msg.header.stamp = cloud->header.stamp;
    sensor_msgs::PointCloud2Modifier mod(msg);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(result.size());
    sensor_msgs::PointCloud2Iterator<float> ox(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> oy(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> oz(msg, "z");
    for (const uint64_t v : result) {
      octomap::OcTreeKey k(
        static_cast<uint16_t>(v & 0xFFFF), static_cast<uint16_t>((v >> 16) & 0xFFFF),
        static_cast<uint16_t>((v >> 32) & 0xFFFF));
      const octomap::point3d c = tree_->keyToCoord(k);
      *ox = c.x(); *oy = c.y(); *oz = c.z();
      ++ox; ++oy; ++oz;
    }
    pub_->publish(msg);

    RCLCPP_DEBUG(get_logger(), "scan free: %zu pts -> %zu voxels, %.1f ms",
      pts.size(), result.size(),
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
  }

  std::string world_frame_;
  double max_range_, min_range_;
  int point_stride_;
  std::unique_ptr<octomap::OcTree> tree_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanFreeNode>());
  rclcpp::shutdown();
  return 0;
}