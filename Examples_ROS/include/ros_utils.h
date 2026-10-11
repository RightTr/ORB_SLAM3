#ifndef ROS_UTILS_H
#define ROS_UTILS_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <cstdarg>
#include <cstdio>
#include <cstdint>
#include <boost/bind.hpp>

#if defined(USE_ROS1)
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/UInt8.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#elif defined(USE_ROS2)
#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#else
#error "Define USE_ROS1 or USE_ROS2 for the selected ROS build"
#endif

#if defined(USE_ROS1)
using Image = sensor_msgs::Image;
using Imu = sensor_msgs::Imu;
using ImageConstPtr = sensor_msgs::ImageConstPtr;
using ImuConstPtr = sensor_msgs::ImuConstPtr;
using TransformBroadcaster = tf2_ros::TransformBroadcaster;
inline double seconds(const ros::Time& t) { return t.toSec(); }
inline ros::Time now() { return ros::Time::now(); }
inline bool ok() { return ros::ok(); }
inline void shutdown() { ros::shutdown(); }
inline std::string startup_string(int argc, char** argv, int index, const std::string&) {
  return argc > index ? std::string(argv[index]) : std::string();
}
inline bool startup_bool(int argc, char** argv, int index, const std::string&, bool fallback=false) {
  return argc > index ? std::string(argv[index]) == "true" : fallback;
}
template<class M> class MessageFilterSubscriber : public message_filters::Subscriber<M> {
 public:
  using message_filters::Subscriber<M>::Subscriber;
};
#elif defined(USE_ROS2)
namespace sensor_msgs {
using Image = msg::Image; using Imu = msg::Imu;
using PointCloud2 = msg::PointCloud2;
using ImageConstPtr = Image::ConstSharedPtr; using ImuConstPtr = Imu::ConstSharedPtr;
}
namespace geometry_msgs { using PoseStamped = msg::PoseStamped; using TransformStamped = msg::TransformStamped; using Point = msg::Point; }
namespace nav_msgs { using Path = msg::Path; }
namespace std_msgs { using UInt8 = msg::UInt8; }
namespace visualization_msgs { using Marker = msg::Marker; using MarkerArray = msg::MarkerArray; }

using Image = sensor_msgs::Image; using Imu = sensor_msgs::Imu;
using ImageConstPtr = sensor_msgs::ImageConstPtr; using ImuConstPtr = sensor_msgs::ImuConstPtr;
inline rclcpp::Node::SharedPtr node();
inline double seconds(const builtin_interfaces::msg::Time& t) { return static_cast<double>(t.sec)+static_cast<double>(t.nanosec)*1e-9; }
inline builtin_interfaces::msg::Time now() { return node()->get_clock()->now(); }
inline bool ok() { return rclcpp::ok(); }
inline void shutdown() { rclcpp::shutdown(); }
inline std::string startup_string(int, char**, int, const std::string& name) {
  auto n=node(); if(!n->has_parameter(name)) n->declare_parameter<std::string>(name, "");
  return n->get_parameter(name).get_value<std::string>();
}
inline bool startup_bool(int, char**, int, const std::string& name, bool fallback=false) {
  auto n=node(); if(!n->has_parameter(name)) n->declare_parameter<bool>(name, fallback);
  return n->get_parameter(name).get_value<bool>();
}

inline rclcpp::Node::SharedPtr& node_storage() { static rclcpp::Node::SharedPtr node; return node; }
inline rclcpp::Node::SharedPtr node() { return node_storage(); }
inline void init(int argc, char** argv, const std::string& name) { rclcpp::init(argc, argv); node_storage()=std::make_shared<rclcpp::Node>(name); }
inline void init_node(const std::string& name) { if (!node_storage()) node_storage()=std::make_shared<rclcpp::Node>(name); }

class TransformBroadcaster {
 public:
  TransformBroadcaster() : broadcaster_(*node()) {}
  void sendTransform(const geometry_msgs::TransformStamped& tf) { broadcaster_.sendTransform(tf); }
 private:
  tf2_ros::TransformBroadcaster broadcaster_;
};

namespace ros { class NodeHandle; }
template<class M>
class MessageFilterSubscriber : public message_filters::Subscriber<M> {
 public:
  MessageFilterSubscriber(ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size);
};

namespace ros {
struct Time { static builtin_interfaces::msg::Time now() { return ::now(); } };
struct Duration { explicit Duration(double s) : seconds(s) {} double seconds; };
struct TimerEvent {};
class Subscriber { public: std::shared_ptr<void> hold; };
class Timer { public: std::shared_ptr<void> hold; };
class Publisher {
 public:
  template<class M> void publish(const M& msg) const { if (publish_) publish_(&msg); }
  std::function<void(const void*)> publish_;
};
class NodeHandle {
 public:
  explicit NodeHandle(const std::string& = std::string()) { init_node("orbslam3"); }
  template<class M> Publisher advertise(const std::string& topic, uint32_t queue) {
    auto p=node()->create_publisher<M>(topic,rclcpp::QoS(queue)); Publisher out;
    out.publish_=[p](const void* m){p->publish(*static_cast<const M*>(m));}; return out;
  }
  template<class M> static rclcpp::QoS subscription_qos(uint32_t queue) {
    if constexpr (std::is_same<M, sensor_msgs::msg::Image>::value ||
                  std::is_same<M, sensor_msgs::msg::Imu>::value)
      return rclcpp::QoS(rclcpp::KeepLast(queue)).best_effort();
    return rclcpp::QoS(queue);
  }
  template<class T> void param(const std::string& name, T& value, const T& fallback) {
    auto n=node(); if(!n->has_parameter(name)) n->declare_parameter<T>(name,fallback); value=n->get_parameter(name).get_value<T>();
  }
  template<class T> T param(const std::string& name, const T& fallback) {
    T value{}; param(name,value,fallback); return value;
  }
  template<class M, class Obj> Subscriber subscribe(const std::string& topic, uint32_t queue,
      void (Obj::*callback)(const typename M::ConstSharedPtr&), Obj* object) {
    auto sub=node()->create_subscription<M>(topic,subscription_qos<M>(queue),
      [object,callback](typename M::ConstSharedPtr msg){(object->*callback)(msg);}); return Subscriber{sub};
  }
  template<class Obj, class Ptr> Subscriber subscribe(const std::string& topic, uint32_t queue,
      void (Obj::*callback)(const Ptr&), Obj* object) {
    using M = typename std::remove_const<typename Ptr::element_type>::type;
    auto sub=node()->create_subscription<M>(topic,subscription_qos<M>(queue),
      [object,callback](typename M::ConstSharedPtr msg){(object->*callback)(msg);}); return Subscriber{sub};
  }
  Timer createTimer(Duration d, std::function<void(const TimerEvent&)> callback) {
    auto t=node()->create_wall_timer(std::chrono::duration<double>(d.seconds),[callback]{callback(TimerEvent{});}); return Timer{t};
  }
};
inline void init(int argc,char**argv,const std::string& name){::init(argc,argv,name);}
inline void start(){}
inline void spin(){rclcpp::spin(node());}
}

template<class M>
MessageFilterSubscriber<M>::MessageFilterSubscriber(ros::NodeHandle&, const std::string& topic, uint32_t queue_size)
{
  auto qos = rmw_qos_profile_sensor_data;
  qos.depth = queue_size;
  this->subscribe(node().get(), topic, qos);
}
#endif

inline double seconds(const ImageConstPtr& image) { return seconds(image->header.stamp); }
inline double seconds(const ImuConstPtr& imu) { return seconds(imu->header.stamp); }
inline void vlog(const char* format, va_list args) {
  char buffer[2048]; std::vsnprintf(buffer, sizeof(buffer), format, args);
#if defined(USE_ROS1)
  ROS_ERROR("%s", buffer);
#else
  RCLCPP_ERROR(node()->get_logger(), "%s", buffer);
#endif
}
inline void log_error(const char* format, ...) { va_list args; va_start(args,format); vlog(format,args); va_end(args); }
#endif
