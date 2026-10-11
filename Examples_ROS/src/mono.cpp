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
#include<cmath>
#include<cstdint>
#include<thread>

#include <cv_bridge/cv_bridge.h>
#include <Eigen/Geometry>

#include<opencv2/core/core.hpp>

#include <System.h>
#include"../include/sensor_sync.hpp"
#include"../include/ros_vis.hpp"
#include<memory>

using namespace std;

static cv::FileNode settingsNode(cv::FileStorage& settings, const std::string& name)
{
    cv::FileNode node = settings[name];
    if (!node.empty()) return node;
    const std::size_t separator = name.find('.');
    if (separator == std::string::npos) return node;
    const cv::FileNode section = settings[name.substr(0, separator)];
    if (section.empty()) return node;
    return section[name.substr(separator + 1)];
}

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, ros::NodeHandle& nh, const std::string& settings,
                 bool offline, int img_size):mpSLAM(pSLAM), sync_queue(new SensorSynchronizer<sensor_msgs::ImageConstPtr>(offline,img_size,1))
    {
        pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/orbslam3/pose", 10);
        path_pub = nh.advertise<nav_msgs::Path>("/orbslam3/path", 10);
        state_pub = nh.advertise<std_msgs::UInt8>("/orbslam3/tracking_state", 10);
        publish_tf = nh.param("publish_tf", false);
        nh.param<std::string>("map_frame", map_frame, "map");
        nh.param<std::string>("camera_frame", camera_frame, "camera_link");
        cv::FileStorage fs(settings, cv::FileStorage::READ);
        if(fs.isOpened())
        {
            settingsNode(fs, "Camera.width") >> processing_width;
            settingsNode(fs, "Camera.height") >> processing_height;
            cv::FileNode new_width = settingsNode(fs, "Camera.newWidth");
            cv::FileNode new_height = settingsNode(fs, "Camera.newHeight");
            if(!new_width.empty()) new_width >> processing_width;
            if(!new_height.empty()) new_height >> processing_height;
        }
        visualization.reset(new RosTrackingVisualization(mpSLAM, nh, map_frame,
                                                         processing_width, processing_height));
    }

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    void ProcessFrames();
    void TrackImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM3::System* mpSLAM;
    std::unique_ptr<SensorSynchronizer<sensor_msgs::ImageConstPtr>> sync_queue;
    ros::Publisher pose_pub, path_pub, state_pub;
    TransformBroadcaster tf_broadcaster;
    nav_msgs::Path path;
    std::string map_frame, camera_frame;
    bool publish_tf;
    int processing_width{0}, processing_height{0};
    std::unique_ptr<RosTrackingVisualization> visualization;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    const std::string vocabulary = startup_string(argc, argv, 1, "vocabulary");
    const std::string settings = startup_string(argc, argv, 2, "settings");
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ros::NodeHandle viewer_params("~");
    ORB_SLAM3::System SLAM(vocabulary,settings,ORB_SLAM3::System::MONOCULAR,
                           viewer_params.param("use_pangolin", true));

    ros::NodeHandle nodeHandler;
    int img_size = 3;
    std::string queue_mode = "online";
    nodeHandler.param("img_size", img_size, img_size);
    nodeHandler.param<std::string>("queue_mode", queue_mode, queue_mode);
    ImageGrabber igb(&SLAM, nodeHandler, settings, queue_mode == "offline", img_size);
    std::string image_topic;
    nodeHandler.param<std::string>("image_topic", image_topic, "/camera/image_raw");
    ros::Subscriber sub = nodeHandler.subscribe(image_topic, 1, &ImageGrabber::GrabImage,&igb);
    std::thread tracking_thread(&ImageGrabber::ProcessFrames, &igb);

    ros::spin();
    igb.sync_queue->Stop();
    tracking_thread.join();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    sync_queue->PushFrame(seconds(msg->header.stamp), msg);
}

void ImageGrabber::ProcessFrames()
{
    while(ok()) {
        sensor_msgs::ImageConstPtr msg;
        double timestamp = -1.0;
        if(!sync_queue->WaitFrame(timestamp, msg)) return;
        TrackImage(msg);
    }
}

void ImageGrabber::TrackImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    cv_ptr = cv_bridge::toCvShare(msg);

    const Sophus::SE3f Tcw = mpSLAM->TrackMonocular(cv_ptr->image, seconds(cv_ptr->header.stamp));
    std_msgs::UInt8 state;
    state.data = static_cast<uint8_t>(mpSLAM->GetTrackingState());
    state_pub.publish(state);
    const std::vector<cv::KeyPoint> keypoints = mpSLAM->GetTrackedKeyPointsUn();
    const std::vector<ORB_SLAM3::MapPoint*> map_points = mpSLAM->GetTrackedMapPoints();
    std::vector<uint8_t> associated(keypoints.size(), 0);
    for(size_t i=0; i<std::min(keypoints.size(),map_points.size()); ++i)
        associated[i] = map_points[i] != nullptr && !map_points[i]->isBad();
    visualization->PublishTrackedImage(msg, state.data, keypoints, associated);
    visualization->PublishMapVisualization(msg);
    if(state.data != 2) return;

    const Sophus::SE3f Twc = Tcw.inverse();
    const Eigen::Vector3f translation = Twc.translation();
    const Eigen::Quaternionf rotation(Twc.rotationMatrix());
    geometry_msgs::PoseStamped pose;
    pose.header = msg->header;
    pose.header.frame_id = map_frame;
    pose.pose.position.x = translation.x();
    pose.pose.position.y = translation.y();
    pose.pose.position.z = translation.z();
    pose.pose.orientation.x = rotation.x();
    pose.pose.orientation.y = rotation.y();
    pose.pose.orientation.z = rotation.z();
    pose.pose.orientation.w = rotation.w();
    pose_pub.publish(pose);

    if(path.header.frame_id != map_frame)
    {
        path.poses.clear();
        path.header.frame_id = map_frame;
    }
    path.header.stamp = pose.header.stamp;
    path.poses.push_back(pose);
    if(path.poses.size() > 100000) path.poses.erase(path.poses.begin(), path.poses.begin() + 1000);
    path_pub.publish(path);

    if(publish_tf)
    {
        geometry_msgs::TransformStamped transform;
        transform.header = pose.header;
        transform.child_frame_id = camera_frame;
        transform.transform.translation.x = translation.x();
        transform.transform.translation.y = translation.y();
        transform.transform.translation.z = translation.z();
        transform.transform.rotation = pose.pose.orientation;
        tf_broadcaster.sendTransform(transform);
    }
}
