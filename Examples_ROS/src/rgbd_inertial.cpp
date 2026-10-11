#include "ros_utils.h"
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

#include <cv_bridge/cv_bridge.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

#include<opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>

#include <System.h>
#include <ImuTypes.h>
#include"../include/sensor_sync.hpp"
#include"../include/ros_vis.hpp"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, ros::NodeHandle& nh, ros::NodeHandle& pnh,
                 const std::string& settings):mpSLAM(pSLAM)
    {
        pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/orbslam3/pose", 10);
        path_pub = nh.advertise<nav_msgs::Path>("/orbslam3/path", 10);
        state_pub = nh.advertise<std_msgs::UInt8>("/orbslam3/tracking_state", 10);
        publish_tf = pnh.param("publish_tf", false);
        pnh.param<std::string>("map_frame", map_frame, "map");
        pnh.param<std::string>("camera_frame", camera_frame, "camera_link");
        pnh.param("imu_size", imu_size, 5000);
        pnh.param("img_size", img_size, 30);
        pnh.param("rgbd_sync_tolerance", rgbd_sync_tolerance, 0.02);
        std::string queue_mode;
        pnh.param<std::string>("queue_mode", queue_mode, "offline");
        sync_queue.reset(new SensorSynchronizer<Pair>(queue_mode == "offline", img_size,
                                                      imu_size));
        cv::FileStorage fs(settings, cv::FileStorage::READ);
        if (fs.isOpened()) {
            fs["Camera.width"] >> processing_width;
            fs["Camera.height"] >> processing_height;
            if (!fs["Camera.newWidth"].empty()) fs["Camera.newWidth"] >> processing_width;
            if (!fs["Camera.newHeight"].empty()) fs["Camera.newHeight"] >> processing_height;
        }
        visualization.reset(new RosTrackingVisualization(mpSLAM, nh, map_frame,
                                                         processing_width, processing_height, 0.1));
    }

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);
    void GrabImu(const sensor_msgs::ImuConstPtr& msg);
    void ProcessFrames();
    void StopProcessing();
    bool TrackPair(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD,
                   const std::vector<ORB_SLAM3::IMU::Point>& imu);

    ORB_SLAM3::System* mpSLAM;
    ros::Publisher pose_pub, path_pub, state_pub;
    TransformBroadcaster tf_broadcaster;
    nav_msgs::Path path;
    std::string map_frame, camera_frame;
    bool publish_tf;
    struct Pair { sensor_msgs::ImageConstPtr rgb, depth; };
    std::unique_ptr<SensorSynchronizer<Pair>> sync_queue;
    double rgbd_sync_tolerance{0.02};
    int imu_size, img_size;
    int processing_width{0}, processing_height{0};
    std::unique_ptr<RosTrackingVisualization> visualization;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    const std::string vocabulary = startup_string(argc, argv, 1, "vocabulary");
    const std::string settings = startup_string(argc, argv, 2, "settings");
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ros::NodeHandle viewer_params("~");
    ORB_SLAM3::System SLAM(vocabulary,settings,ORB_SLAM3::System::IMU_RGBD,
                           viewer_params.param("use_pangolin", true));

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    ImageGrabber igb(&SLAM, nh, pnh, settings);

    std::string rgb_topic, depth_topic, imu_topic;
    pnh.param<std::string>("rgb_topic", rgb_topic, "/camera/rgb/image_raw");
    pnh.param<std::string>("depth_topic", depth_topic, "/camera/depth_registered/image_raw");
    pnh.param<std::string>("imu_topic", imu_topic, "/imu");
    MessageFilterSubscriber<sensor_msgs::Image> rgb_sub(nh, rgb_topic, 5);
    MessageFilterSubscriber<sensor_msgs::Image> depth_sub(nh, depth_topic, 5);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(5), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,boost::placeholders::_1,boost::placeholders::_2));
    ros::Subscriber imu_sub = nh.subscribe(imu_topic, 2000, &ImageGrabber::GrabImu, &igb);
    std::thread tracking_thread(&ImageGrabber::ProcessFrames, &igb);

    ros::spin();

    igb.StopProcessing();
    tracking_thread.join();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    {
        const double trgb = seconds(msgRGB->header.stamp);
        const double tdepth = seconds(msgD->header.stamp);
        if (std::abs(trgb - tdepth) > rgbd_sync_tolerance) {
            return;
        }
        sync_queue->PushFrame(trgb, Pair{msgRGB, msgD});
    }
}

void ImageGrabber::GrabImu(const sensor_msgs::ImuConstPtr& msg)
{
    const double t = seconds(msg->header.stamp);
    sync_queue->PushImu(ORB_SLAM3::IMU::Point(
        cv::Point3f(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z),
        cv::Point3f(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z), t));
}

void ImageGrabber::StopProcessing()
{
    sync_queue->Stop();
}

void ImageGrabber::ProcessFrames()
{
    while (true) {
        double timestamp = -1.0;
        Pair frame;
        std::vector<ORB_SLAM3::IMU::Point> measurements;
        if (!sync_queue->WaitNext(timestamp, frame, measurements)) return;
        if (!TrackPair(frame.rgb, frame.depth, measurements)) {
            sync_queue->Stop();
            return;
        }
    }
}

bool ImageGrabber::TrackPair(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD,
                             const std::vector<ORB_SLAM3::IMU::Point>& imu)
{
    // Copy the ros image message to cv::Mat.
    const cv_bridge::CvImageConstPtr cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    const cv_bridge::CvImageConstPtr cv_ptrD = cv_bridge::toCvShare(msgD);
    if (cv_ptrRGB->image.size() != cv_ptrD->image.size()) {
        log_error("RGB and depth dimensions differ; registered depth is required");
        return false;
    }
    const Sophus::SE3f Tcw = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,seconds(cv_ptrRGB->header.stamp),imu);
    std_msgs::UInt8 state;
    state.data = static_cast<uint8_t>(mpSLAM->GetTrackingState());
    state_pub.publish(state);
    const std::vector<cv::KeyPoint> keypoints = mpSLAM->GetTrackedKeyPointsUn();
    const std::vector<ORB_SLAM3::MapPoint*> map_points = mpSLAM->GetTrackedMapPoints();
    std::vector<uint8_t> associated(keypoints.size(), 0);
    for (size_t i = 0; i < std::min(keypoints.size(), map_points.size()); ++i)
        associated[i] = map_points[i] != nullptr;
    visualization->PublishTrackedImage(msgRGB, state.data, keypoints, associated);
    if (state.data != 2) return true;
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
    visualization->PublishMapVisualization(msgRGB);
    return true;
}

