#include "ros/ros_utils.h"
/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <cv_bridge/cv_bridge.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

#include<opencv2/core/core.hpp>

#include"../../include/System.h"
#include"../../include/ImuTypes.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, ros::NodeHandle& nh, const std::string& settings):mpSLAM(pSLAM)
    {
        pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/orbslam3/pose", 10);
        path_pub = nh.advertise<nav_msgs::Path>("/orbslam3/path", 10);
        state_pub = nh.advertise<std_msgs::UInt8>("/orbslam3/tracking_state", 10);
        publish_tf = nh.param("publish_tf", false);
        debug = nh.param("debug", false);
        nh.param<std::string>("map_frame", map_frame, "map");
        nh.param<std::string>("camera_frame", camera_frame, "camera_link");
        nh.param("imu_wait_timeout", imu_timeout, 0.5);
        nh.param("max_imu_gap", max_imu_gap, 0.05);
        nh.param("imu_queue_limit", imu_queue_limit, 5000);
        nh.param("image_queue_limit", image_queue_limit, 20);
        cv::FileStorage fs(settings, cv::FileStorage::READ);
        if (fs.isOpened()) {
            fs["Camera.width"] >> yaml_width; fs["Camera.height"] >> yaml_height;
            if (!fs["Camera1.fx"].empty()) fs["Camera1.fx"] >> yaml_fx; else fs["Camera.fx"] >> yaml_fx;
            if (!fs["Camera1.fy"].empty()) fs["Camera1.fy"] >> yaml_fy; else fs["Camera.fy"] >> yaml_fy;
        }
    }

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);
    void GrabImu(const sensor_msgs::ImuConstPtr& msg);
    void CameraInfo(const sensor_msgs::CameraInfoConstPtr& info);
    void ProcessImu();
    void TrackPair(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD,
                   const std::vector<ORB_SLAM3::IMU::Point>& imu);

    ORB_SLAM3::System* mpSLAM;
    ros::Publisher pose_pub, path_pub, state_pub;
    orb_slam3_ros::TransformBroadcaster tf_broadcaster;
    nav_msgs::Path path;
    std::string map_frame, camera_frame;
    bool publish_tf, debug;
    struct Pair { sensor_msgs::ImageConstPtr rgb, depth; orb_slam3_ros::WallTime arrival; };
    std::deque<Pair> frames;
    std::deque<sensor_msgs::ImuConstPtr> imus;
    double imu_timeout, max_imu_gap, last_imu_sent{-1.0};
    int imu_queue_limit, image_queue_limit;
    int yaml_width{0}, yaml_height{0}; double yaml_fx{0.0}, yaml_fy{0.0}; bool camera_checked{false};
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    const std::string vocabulary = orb_slam3_ros::startup_string(argc, argv, 1, "vocabulary");
    const std::string settings = orb_slam3_ros::startup_string(argc, argv, 2, "settings");
    if(!orb_slam3_ros::valid_arguments(argc, 3, 3) || !orb_slam3_ros::has_required(vocabulary, settings))
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 RGBD path_to_vocabulary path_to_settings" << endl;
        orb_slam3_ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(vocabulary,settings,ORB_SLAM3::System::IMU_RGBD,true);

    ros::NodeHandle nh;
    ImageGrabber igb(&SLAM, nh, settings);

    std::string rgb_topic, depth_topic, camera_info_topic, imu_topic;
    nh.param<std::string>("rgb_topic", rgb_topic, "/camera/rgb/image_raw");
    nh.param<std::string>("depth_topic", depth_topic, "/camera/depth_registered/image_raw");
    nh.param<std::string>("camera_info_topic", camera_info_topic, "/camera/rgb/camera_info");
    nh.param<std::string>("imu_topic", imu_topic, "/imu");
    orb_slam3_ros::MessageFilterSubscriber<sensor_msgs::Image> rgb_sub(nh, rgb_topic, 100);
    orb_slam3_ros::MessageFilterSubscriber<sensor_msgs::Image> depth_sub(nh, depth_topic, 100);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,boost::placeholders::_1,boost::placeholders::_2));
    ros::Subscriber info_sub = nh.subscribe(camera_info_topic, 10, &ImageGrabber::CameraInfo, &igb);
    ros::Subscriber imu_sub = nh.subscribe(imu_topic, 2000, &ImageGrabber::GrabImu, &igb);
    ros::Timer imu_timer = nh.createTimer(ros::Duration(0.005), [&](const ros::TimerEvent&) { igb.ProcessImu(); });

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    orb_slam3_ros::shutdown();

    return 0;
}

void ImageGrabber::CameraInfo(const sensor_msgs::CameraInfoConstPtr& info)
{
    if (camera_checked) return;
    camera_checked = true;
    if ((yaml_width > 0 && static_cast<int>(info->width) != yaml_width) ||
        (yaml_height > 0 && static_cast<int>(info->height) != yaml_height) ||
        (yaml_fx > 0.0 && std::abs(info->K[0] - yaml_fx) > 1.0) ||
        (yaml_fy > 0.0 && std::abs(info->K[4] - yaml_fy) > 1.0))
        orb_slam3_ros::log_warn("CameraInfo differs from ORB-SLAM3 settings; tracking uses YAML calibration");
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    frames.push_back(Pair{msgRGB, msgD, orb_slam3_ros::wall_now()});
    while (frames.size() > static_cast<size_t>(image_queue_limit)) {
        frames.pop_front();
        orb_slam3_ros::log_warn("RGB-D queue full; dropping oldest image pair");
    }
}

void ImageGrabber::GrabImu(const sensor_msgs::ImuConstPtr& msg)
{
    const double t = orb_slam3_ros::seconds(msg->header.stamp);
    auto it = std::upper_bound(imus.begin(), imus.end(), t,
        [](double value, const sensor_msgs::ImuConstPtr& item) { return value < orb_slam3_ros::seconds(item->header.stamp); });
    imus.insert(it, msg);
    while (imus.size() > static_cast<size_t>(imu_queue_limit)) {
        imus.pop_front();
        orb_slam3_ros::log_warn("IMU queue full; discarded oldest sample");
    }
}

void ImageGrabber::ProcessImu()
{
    if (frames.empty()) return;
    const Pair frame = frames.front();
    const double image_time = orb_slam3_ros::seconds(frame.rgb->header.stamp);
    if (imus.empty() || orb_slam3_ros::seconds(imus.back()->header.stamp) <= image_time) {
        if (orb_slam3_ros::elapsed_seconds(frame.arrival) > imu_timeout) {
            frames.pop_front();
            orb_slam3_ros::log_warn("IMU did not cover image timestamp %.9f before timeout; dropping image", image_time);
        }
        return;
    }
    const double previous_imu_sent = last_imu_sent;
    std::vector<ORB_SLAM3::IMU::Point> measurements;
    while (!imus.empty() && orb_slam3_ros::seconds(imus.front()->header.stamp) <= image_time) {
        const auto sample = imus.front(); imus.pop_front();
        const double t = orb_slam3_ros::seconds(sample->header.stamp);
        if (t > last_imu_sent) {
            measurements.emplace_back(cv::Point3f(sample->linear_acceleration.x,sample->linear_acceleration.y,sample->linear_acceleration.z),
                                      cv::Point3f(sample->angular_velocity.x,sample->angular_velocity.y,sample->angular_velocity.z),t);
            last_imu_sent=t;
        }
    }
    if (!imus.empty() && orb_slam3_ros::seconds(imus.front()->header.stamp) > last_imu_sent) {
        const auto sample=imus.front();
        measurements.emplace_back(cv::Point3f(sample->linear_acceleration.x,sample->linear_acceleration.y,sample->linear_acceleration.z),
                                  cv::Point3f(sample->angular_velocity.x,sample->angular_velocity.y,sample->angular_velocity.z),
                                  orb_slam3_ros::seconds(sample->header.stamp));
        last_imu_sent=orb_slam3_ros::seconds(sample->header.stamp);
    }
    bool imu_gap = !measurements.empty() && previous_imu_sent >= 0.0 && measurements.front().t - previous_imu_sent > max_imu_gap;
    for(size_t i=1;i<measurements.size();++i)
        imu_gap = imu_gap || measurements[i].t-measurements[i-1].t>max_imu_gap;
    if(imu_gap) {
            frames.pop_front();
            mpSLAM->ResetActiveMap();
            path.poses.clear();
            orb_slam3_ros::log_warn("IMU gap exceeded %.3f s; dropping frame and resetting active map", max_imu_gap);
            return;
    }
    TrackPair(frame.rgb, frame.depth, measurements);
    frames.pop_front();
}

void ImageGrabber::TrackPair(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD,
                             const std::vector<ORB_SLAM3::IMU::Point>& imu)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        orb_slam3_ros::log_error("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        orb_slam3_ros::log_error("cv_bridge exception: %s", e.what());
        return;
    }
    if (cv_ptrRGB->image.size() != cv_ptrD->image.size()) {
        orb_slam3_ros::log_error("RGB and depth dimensions differ; registered depth is required");
        return;
    }
    const Sophus::SE3f Tcw = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,orb_slam3_ros::seconds(cv_ptrRGB->header.stamp),imu);
    std_msgs::UInt8 state;
    state.data = static_cast<uint8_t>(mpSLAM->GetTrackingState());
    state_pub.publish(state);
    if (debug) orb_slam3_ros::log_info_throttle(5.0, "ORB-SLAM3 tracking state: %d", static_cast<int>(state.data));
    if (state.data != 2 || !Tcw.matrix().allFinite()) return;
    const Sophus::SE3f Twc = Tcw.inverse();
    const Eigen::Vector3f t = Twc.translation();
    const Eigen::Quaternionf q(Twc.rotationMatrix());
    geometry_msgs::PoseStamped pose;
    pose.header = msgRGB->header;
    pose.header.frame_id = map_frame;
    pose.pose.position.x=t.x(); pose.pose.position.y=t.y(); pose.pose.position.z=t.z();
    pose.pose.orientation.x=q.x(); pose.pose.orientation.y=q.y(); pose.pose.orientation.z=q.z(); pose.pose.orientation.w=q.w();
    pose_pub.publish(pose);
    if (path.header.frame_id != map_frame) { path.poses.clear(); path.header.frame_id = map_frame; }
    path.header.stamp = pose.header.stamp;
    path.poses.push_back(pose);
    if (path.poses.size() > 100000) path.poses.erase(path.poses.begin(), path.poses.begin() + 1000);
    path_pub.publish(path);
    if (publish_tf) {
        geometry_msgs::TransformStamped tf;
        tf.header = pose.header; tf.child_frame_id = camera_frame;
        tf.transform.translation.x=t.x(); tf.transform.translation.y=t.y(); tf.transform.translation.z=t.z();
        tf.transform.rotation = pose.pose.orientation;
        tf_broadcaster.sendTransform(tf);
    }
}



