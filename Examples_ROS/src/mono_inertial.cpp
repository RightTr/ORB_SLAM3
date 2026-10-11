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
#include<queue>
#include<thread>
#include<mutex>

#include<cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>

#include <System.h>
#include <ImuTypes.h>
#include"../include/sensor_sync.hpp"
#include<memory>

using namespace std;

class ImuGrabber
{
public:
    explicit ImuGrabber(SensorSynchronizer<sensor_msgs::ImageConstPtr>* queue): sync_queue(queue) {}
    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);
    SensorSynchronizer<sensor_msgs::ImageConstPtr>* sync_queue;
};

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, ImuGrabber *pImuGb, SensorSynchronizer<sensor_msgs::ImageConstPtr>* queue,
                 const bool bClahe): mpSLAM(pSLAM), mpImuGb(pImuGb), sync_queue(queue), mbClahe(bClahe){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);
    void SyncWithImu();

    ORB_SLAM3::System* mpSLAM;
    ImuGrabber *mpImuGb;
    SensorSynchronizer<sensor_msgs::ImageConstPtr>* sync_queue;

    const bool mbClahe;
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "Mono_Inertial");
  ros::NodeHandle n("~");
  const std::string vocabulary = startup_string(argc, argv, 1, "vocabulary");
  const std::string settings = startup_string(argc, argv, 2, "settings");
  const bool bEqual = startup_bool(argc, argv, 3, "do_equalize");

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(vocabulary,settings,ORB_SLAM3::System::IMU_MONOCULAR,
                           n.param("use_pangolin", true));

  int img_size = 3, imu_size = 5000;
  std::string queue_mode = "online";
  n.param("img_size", img_size, img_size);
  n.param("imu_size", imu_size, imu_size);
  n.param<std::string>("queue_mode", queue_mode, queue_mode);
  SensorSynchronizer<sensor_msgs::ImageConstPtr> sync_queue(queue_mode == "offline", img_size,
                                                            imu_size);
  ImuGrabber imugb(&sync_queue);
  ImageGrabber igb(&SLAM,&imugb,&sync_queue,bEqual);

  // Maximum delay, 5 seconds
  std::string imu_topic, image_topic;
  n.param<std::string>("imu_topic", imu_topic, "/imu");
  n.param<std::string>("image_topic", image_topic, "/camera/image_raw");
  ros::Subscriber sub_imu = n.subscribe(imu_topic, 1000, &ImuGrabber::GrabImu, &imugb);
  ros::Subscriber sub_img0 = n.subscribe(image_topic, 100, &ImageGrabber::GrabImage,&igb);

  std::thread sync_thread(&ImageGrabber::SyncWithImu,&igb);

  ros::spin();
  sync_queue.Stop();
  shutdown();
  sync_thread.join();
  SLAM.Shutdown();
  SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

  return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr &img_msg)
{
  sync_queue->PushFrame(seconds(img_msg->header.stamp), img_msg);
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
    sensor_msgs::ImageConstPtr image;
    double timestamp = -1.0;
    vector<ORB_SLAM3::IMU::Point> measurements;
    if(!sync_queue->WaitNext(timestamp, image, measurements)) return;
    cv::Mat im = GetImage(image);
    if(im.empty()) {
      log_error("Could not convert image after dequeuing its sensor batch; stopping the worker");
      sync_queue->Stop();
      return;
    }
    if(mbClahe) mClahe->apply(im,im);
    mpSLAM->TrackMonocular(im,timestamp,measurements);
  }
}

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
  const double t = seconds(imu_msg->header.stamp);
  sync_queue->PushImu(ORB_SLAM3::IMU::Point(
      cv::Point3f(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z),
      cv::Point3f(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z), t));
}
