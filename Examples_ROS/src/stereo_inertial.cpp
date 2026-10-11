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
#include<vector>
#include<thread>

#include<cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <System.h>
#include <ImuTypes.h>
#include"../include/sensor_sync.hpp"
#include<memory>

using namespace std;
using StereoFrame = std::pair<sensor_msgs::ImageConstPtr, sensor_msgs::ImageConstPtr>;

class ImuGrabber
{
public:
    explicit ImuGrabber(SensorSynchronizer<std::pair<sensor_msgs::ImageConstPtr, sensor_msgs::ImageConstPtr>>* queue)
        : sync_queue(queue) {}
    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);
    SensorSynchronizer<std::pair<sensor_msgs::ImageConstPtr, sensor_msgs::ImageConstPtr>>* sync_queue;
};

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, ImuGrabber *pImuGb,
                 SensorSynchronizer<std::pair<sensor_msgs::ImageConstPtr, sensor_msgs::ImageConstPtr>>* queue,
                 const bool offline_mode, int frame_limit, const bool bRect, const bool bClahe,
                 double stereo_tolerance): mpSLAM(pSLAM), mpImuGb(pImuGb), sync_queue(queue),
                 stereo_sync(*queue, offline_mode, frame_limit, stereo_tolerance),
                 do_rectify(bRect), mbClahe(bClahe){}

    void GrabImageLeft(const sensor_msgs::ImageConstPtr& msg);
    void GrabImageRight(const sensor_msgs::ImageConstPtr& msg);
    cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);
    void SyncWithImu();

    ORB_SLAM3::System* mpSLAM;
    ImuGrabber *mpImuGb;
    SensorSynchronizer<std::pair<sensor_msgs::ImageConstPtr, sensor_msgs::ImageConstPtr>>* sync_queue;
    StereoFrameSynchronizer<sensor_msgs::ImageConstPtr> stereo_sync;

    const bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;

    const bool mbClahe;
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "Stereo_Inertial");
  ros::NodeHandle n("~");
  const std::string vocabulary = startup_string(argc, argv, 1, "vocabulary");
  const std::string settings = startup_string(argc, argv, 2, "settings");
  const bool do_rectify = startup_bool(argc, argv, 3, "do_rectify");
  const bool bEqual = startup_bool(argc, argv, 4, "do_equalize");
  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM3::System SLAM(vocabulary,settings,ORB_SLAM3::System::IMU_STEREO,
                         n.param("use_pangolin", true));

  int img_size = 3, imu_size = 5000;
  double stereo_sync_tolerance = 0.002;
  std::string queue_mode = "online";
  n.param("img_size", img_size, img_size);
  n.param("imu_size", imu_size, imu_size);
  n.param("stereo_sync_tolerance", stereo_sync_tolerance, stereo_sync_tolerance);
  n.param<std::string>("queue_mode", queue_mode, queue_mode);
  const bool offline = queue_mode == "offline";
  SensorSynchronizer<StereoFrame> sync_queue(offline, img_size, imu_size);
  ImuGrabber imugb(&sync_queue);
  ImageGrabber igb(&SLAM,&imugb,&sync_queue,offline,img_size,do_rectify,bEqual,stereo_sync_tolerance);

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

  // Maximum delay, 5 seconds
  std::string imu_topic, left_topic, right_topic;
  n.param<std::string>("imu_topic", imu_topic, "/imu");
  n.param<std::string>("left_topic", left_topic, "/camera/left/image_raw");
  n.param<std::string>("right_topic", right_topic, "/camera/right/image_raw");
  ros::Subscriber sub_imu = n.subscribe(imu_topic, 1000, &ImuGrabber::GrabImu, &imugb);
  ros::Subscriber sub_img_left = n.subscribe(left_topic, 100, &ImageGrabber::GrabImageLeft,&igb);
  ros::Subscriber sub_img_right = n.subscribe(right_topic, 100, &ImageGrabber::GrabImageRight,&igb);

  std::thread sync_thread(&ImageGrabber::SyncWithImu,&igb);

  ros::spin();
  sync_queue.Stop();
  shutdown();
  sync_thread.join();
  SLAM.Shutdown();
  SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
  SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
  SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

  return 0;
}



void ImageGrabber::GrabImageLeft(const sensor_msgs::ImageConstPtr &img_msg)
{
  if (!stereo_sync.PushLeft(seconds(img_msg->header.stamp), img_msg))
    log_error("Left image queue overflow; wrapper paused, restart after fixing input rate");
}

void ImageGrabber::GrabImageRight(const sensor_msgs::ImageConstPtr &img_msg)
{
  if (!stereo_sync.PushRight(seconds(img_msg->header.stamp), img_msg))
    log_error("Right image queue overflow; wrapper paused, restart after fixing input rate");
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg)
{
  // Copy the ros image message to cv::Mat.
  const cv_bridge::CvImageConstPtr cv_ptr =
      cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
  return cv_ptr->image.clone();
}

void ImageGrabber::SyncWithImu()
{
  while(ok())
  {
    StereoFrame frame;
    double timestamp = -1.0;
    vector<ORB_SLAM3::IMU::Point> measurements;
    if(!sync_queue->WaitNext(timestamp, frame, measurements)) return;

    cv::Mat imLeft = GetImage(frame.first);
    cv::Mat imRight = GetImage(frame.second);
    if(imLeft.empty() || imRight.empty()) {
      log_error("Could not convert stereo images after dequeuing the sensor batch; stopping the worker");
      sync_queue->Stop();
      return;
    }
    if(mbClahe) { mClahe->apply(imLeft,imLeft); mClahe->apply(imRight,imRight); }
    if(do_rectify) {
      cv::Mat rectifiedLeft, rectifiedRight;
      cv::remap(imLeft,rectifiedLeft,M1l,M2l,cv::INTER_LINEAR);
      cv::remap(imRight,rectifiedRight,M1r,M2r,cv::INTER_LINEAR);
      imLeft = std::move(rectifiedLeft);
      imRight = std::move(rectifiedRight);
    }
    if(imLeft.size() != imRight.size()) {
      log_error("Stereo image dimensions differ after dequeuing the sensor batch; stopping the worker");
      sync_queue->Stop();
      return;
    }
    mpSLAM->TrackStereo(imLeft,imRight,timestamp,measurements);
  }
}

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
  const double timestamp = seconds(imu_msg->header.stamp);
  sync_queue->PushImu(ORB_SLAM3::IMU::Point(
      cv::Point3f(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z),
      cv::Point3f(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z), timestamp));
}
