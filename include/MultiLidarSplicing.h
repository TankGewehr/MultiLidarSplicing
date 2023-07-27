#pragma once

#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include "sensors/Lidar.h"

class MultiLidarSplicing
{
private:
    Lidar lidar_front_left;
    Lidar lidar_front_middle;
    Lidar lidar_front_right;
    Lidar lidar_rear_left;
    Lidar lidar_rear_middle;
    Lidar lidar_rear_right;

    std::string frame_id;
    std::string publish_topic;

    ros::NodeHandle ros_nodehandle;
    ros::Publisher point_cloud_publisher;

public:
    MultiLidarSplicing(std::string lidar_front_left,
                       std::string lidar_front_middle,
                       std::string lidar_front_right,
                       std::string lidar_rear_left,
                       std::string lidar_rear_middle,
                       std::string lidar_rear_right,
                       std::string frame_id,
                       std::string publish_topic);
    ~MultiLidarSplicing();

    void run();

    void callback(const sensor_msgs::PointCloud2::ConstPtr &lidar_front_left_msg,
                  const sensor_msgs::PointCloud2::ConstPtr &lidar_front_middle_msg,
                  const sensor_msgs::PointCloud2::ConstPtr &lidar_front_right_msg,
                  const sensor_msgs::PointCloud2::ConstPtr &lidar_rear_left_msg,
                  const sensor_msgs::PointCloud2::ConstPtr &lidar_rear_middle_msg,
                  const sensor_msgs::PointCloud2::ConstPtr &lidar_rear_right_msg);
};