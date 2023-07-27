#include "MultiLidarSplicing.h"

MultiLidarSplicing::MultiLidarSplicing(std::string lidar_front_left,
                                       std::string lidar_front_middle,
                                       std::string lidar_front_right,
                                       std::string lidar_rear_left,
                                       std::string lidar_rear_middle,
                                       std::string lidar_rear_right,
                                       std::string frame_id,
                                       std::string publish_topic) : lidar_front_left(lidar_front_left),
                                                                    lidar_front_middle(lidar_front_middle),
                                                                    lidar_front_right(lidar_front_right),
                                                                    lidar_rear_left(lidar_rear_left),
                                                                    lidar_rear_middle(lidar_rear_middle),
                                                                    lidar_rear_right(lidar_rear_right)
{
    this->frame_id = frame_id;
    this->publish_topic = publish_topic;
    this->point_cloud_publisher = this->ros_nodehandle.advertise<sensor_msgs::PointCloud2>(this->publish_topic, 10);
}

MultiLidarSplicing::~MultiLidarSplicing() = default;

void MultiLidarSplicing::run()
{
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_front_left(this->ros_nodehandle, this->lidar_front_left.getChannel(), 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_front_middle(this->ros_nodehandle, this->lidar_front_middle.getChannel(), 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_front_right(this->ros_nodehandle, this->lidar_front_right.getChannel(), 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_rear_left(this->ros_nodehandle, this->lidar_rear_left.getChannel(), 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_rear_middle(this->ros_nodehandle, this->lidar_rear_middle.getChannel(), 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_rear_right(this->ros_nodehandle, this->lidar_rear_right.getChannel(), 10);

    message_filters::Synchronizer<
        message_filters::sync_policies::ApproximateTime<
            sensor_msgs::PointCloud2,
            sensor_msgs::PointCloud2,
            sensor_msgs::PointCloud2,
            sensor_msgs::PointCloud2,
            sensor_msgs::PointCloud2,
            sensor_msgs::PointCloud2>>
        lidar_sync(
            message_filters::sync_policies::ApproximateTime<
                sensor_msgs::PointCloud2,
                sensor_msgs::PointCloud2,
                sensor_msgs::PointCloud2,
                sensor_msgs::PointCloud2,
                sensor_msgs::PointCloud2,
                sensor_msgs::PointCloud2>(10),
            lidar_front_left,
            lidar_front_middle,
            lidar_front_right,
            lidar_rear_left,
            lidar_rear_middle,
            lidar_rear_right);

    lidar_sync.registerCallback(boost::bind(&MultiLidarSplicing::callback, this, _1, _2, _3, _4, _5, _6));
    ros::spin();
}

void MultiLidarSplicing::callback(const sensor_msgs::PointCloud2::ConstPtr &lidar_front_left_msg,
                                  const sensor_msgs::PointCloud2::ConstPtr &lidar_front_middle_msg,
                                  const sensor_msgs::PointCloud2::ConstPtr &lidar_front_right_msg,
                                  const sensor_msgs::PointCloud2::ConstPtr &lidar_rear_left_msg,
                                  const sensor_msgs::PointCloud2::ConstPtr &lidar_rear_middle_msg,
                                  const sensor_msgs::PointCloud2::ConstPtr &lidar_rear_right_msg)
{
    auto startTime = std::chrono::high_resolution_clock::now();

    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_front_left_point_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_front_middle_point_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_front_right_point_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_rear_left_point_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_rear_middle_point_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_rear_right_point_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::fromROSMsg(*lidar_front_left_msg, *lidar_front_left_point_cloud);
    pcl::fromROSMsg(*lidar_front_middle_msg, *lidar_front_middle_point_cloud);
    pcl::fromROSMsg(*lidar_front_right_msg, *lidar_front_right_point_cloud);
    pcl::fromROSMsg(*lidar_rear_left_msg, *lidar_rear_left_point_cloud);
    pcl::fromROSMsg(*lidar_rear_middle_msg, *lidar_rear_middle_point_cloud);
    pcl::fromROSMsg(*lidar_rear_right_msg, *lidar_rear_right_point_cloud);

    // pcl::transformPointCloud(*lidar_front_left_point_cloud, *lidar_front_left_point_cloud, this->lidar_front_left.getExtrinsic());
    // pcl::transformPointCloud(*lidar_front_middle_point_cloud, *lidar_front_middle_point_cloud, this->lidar_front_middle.getExtrinsic());
    // pcl::transformPointCloud(*lidar_front_right_point_cloud, *lidar_front_right_point_cloud, this->lidar_front_right.getExtrinsic());
    // pcl::transformPointCloud(*lidar_rear_left_point_cloud, *lidar_rear_left_point_cloud, this->lidar_rear_left.getExtrinsic());
    // pcl::transformPointCloud(*lidar_rear_middle_point_cloud, *lidar_rear_middle_point_cloud, this->lidar_rear_middle.getExtrinsic());
    // pcl::transformPointCloud(*lidar_rear_right_point_cloud, *lidar_rear_right_point_cloud, this->lidar_rear_right.getExtrinsic());

    // *point_cloud += *lidar_front_left_point_cloud;
    // *point_cloud += *lidar_front_middle_point_cloud;
    // *point_cloud += *lidar_front_right_point_cloud;
    // *point_cloud += *lidar_rear_left_point_cloud;
    // *point_cloud += *lidar_rear_middle_point_cloud;
    // *point_cloud += *lidar_rear_right_point_cloud;

    pcl::transformPointCloud(*lidar_front_middle_point_cloud, *lidar_front_middle_point_cloud, this->lidar_front_middle.getExtrinsic());
    *point_cloud += *lidar_front_middle_point_cloud;

    pcl::transformPointCloud(*lidar_rear_middle_point_cloud, *lidar_rear_middle_point_cloud, this->lidar_rear_middle.getExtrinsic());
    *lidar_rear_left_point_cloud += *lidar_rear_middle_point_cloud;

    pcl::transformPointCloud(*lidar_rear_left_point_cloud, *lidar_rear_left_point_cloud, this->lidar_rear_left.getExtrinsic());
    *lidar_front_left_point_cloud += *lidar_rear_left_point_cloud;

    pcl::transformPointCloud(*lidar_front_left_point_cloud, *lidar_front_left_point_cloud, this->lidar_front_left.getExtrinsic());
    *point_cloud += *lidar_front_left_point_cloud;

    pcl::transformPointCloud(*lidar_rear_right_point_cloud, *lidar_rear_right_point_cloud, this->lidar_rear_right.getExtrinsic());
    *lidar_front_right_point_cloud += *lidar_rear_right_point_cloud;

    pcl::transformPointCloud(*lidar_rear_right_point_cloud, *lidar_rear_right_point_cloud, this->lidar_rear_right.getExtrinsic());
    *point_cloud += *lidar_rear_right_point_cloud;

    sensor_msgs::PointCloud2 point_cloud_msg;
    pcl::toROSMsg(*point_cloud, point_cloud_msg);
    point_cloud_msg.header = lidar_front_middle_msg->header;
    point_cloud_msg.header.frame_id = this->frame_id;
    point_cloud_publisher.publish(point_cloud_msg);

    auto endTime = std::chrono::high_resolution_clock::now();
    std::cout << "[" << lidar_front_middle_msg->header.stamp << "]: "
              << "(" << std::chrono::duration_cast<std::chrono::nanoseconds>(endTime - startTime).count() / 1000000000.0 << ") "
              << this->publish_topic << std::endl;
}