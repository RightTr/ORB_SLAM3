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
#include<chrono>

#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <System.h>
#include"../include/sensor_sync.hpp"
#include<memory>
#include<thread>

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, bool offline, int img_size, double tolerance):mpSLAM(pSLAM),
        sync_queue(new SensorSynchronizer<std::pair<sensor_msgs::ImageConstPtr,sensor_msgs::ImageConstPtr>>(offline,img_size,1)),
        stereo_sync_tolerance(tolerance){}

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);
    void ProcessFrames();
    void TrackStereoFrame(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);

    ORB_SLAM3::System* mpSLAM;
    std::unique_ptr<SensorSynchronizer<std::pair<sensor_msgs::ImageConstPtr,sensor_msgs::ImageConstPtr>>> sync_queue;
    double stereo_sync_tolerance;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    const std::string vocabulary = startup_string(argc, argv, 1, "vocabulary");
    const std::string settings = startup_string(argc, argv, 2, "settings");
    const bool do_rectify = startup_bool(argc, argv, 3, "do_rectify");
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ros::NodeHandle viewer_params("~");
    ORB_SLAM3::System SLAM(vocabulary,settings,ORB_SLAM3::System::STEREO,
                           viewer_params.param("use_pangolin", true));

    int img_size = 3;
    std::string queue_mode = "online";
    double stereo_sync_tolerance = 0.002;
    viewer_params.param("img_size", img_size, img_size);
    viewer_params.param<std::string>("queue_mode", queue_mode, queue_mode);
    viewer_params.param("stereo_sync_tolerance", stereo_sync_tolerance, stereo_sync_tolerance);
    ImageGrabber igb(&SLAM, queue_mode == "offline", img_size, stereo_sync_tolerance);

    igb.do_rectify = do_rectify;

    if(igb.do_rectify)
    {
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(settings, cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
    }

    ros::NodeHandle nh;

    std::string left_topic, right_topic;
    nh.param<std::string>("left_topic", left_topic, "/camera/left/image_raw");
    nh.param<std::string>("right_topic", right_topic, "/camera/right/image_raw");
    MessageFilterSubscriber<sensor_msgs::Image> left_sub(nh, left_topic, 1);
    MessageFilterSubscriber<sensor_msgs::Image> right_sub(nh, right_topic, 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,boost::placeholders::_1,boost::placeholders::_2));
    std::thread tracking_thread(&ImageGrabber::ProcessFrames, &igb);

    ros::spin();
    igb.sync_queue->Stop();
    tracking_thread.join();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    shutdown();

    return 0;
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    const double left_time = seconds(msgLeft->header.stamp), right_time = seconds(msgRight->header.stamp);
    if(std::abs(left_time-right_time) > stereo_sync_tolerance) return;
    sync_queue->PushFrame(left_time, {msgLeft,msgRight});
}

void ImageGrabber::ProcessFrames()
{
    while(ok()) {
        std::pair<sensor_msgs::ImageConstPtr,sensor_msgs::ImageConstPtr> frame;
        double timestamp=-1.0;
        if(!sync_queue->WaitFrame(timestamp,frame)) return;
        TrackStereoFrame(frame.first,frame.second);
    }
}

void ImageGrabber::TrackStereoFrame(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    cv_ptrLeft = cv_bridge::toCvShare(msgLeft);

    cv_bridge::CvImageConstPtr cv_ptrRight;
    cv_ptrRight = cv_bridge::toCvShare(msgRight);

    if(do_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        mpSLAM->TrackStereo(imLeft,imRight,seconds(cv_ptrLeft->header.stamp));
    }
    else
    {
        mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,seconds(cv_ptrLeft->header.stamp));
    }

}
