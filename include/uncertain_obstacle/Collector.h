//
// Created by hongsun on 19. 4. 9.
//

#ifndef SRC_COLLECTOR_H
#define SRC_COLLECTOR_H

#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/PCLPointCloud2.h>
#include "pcl_conversions/pcl_conversions.h"

class Collector {
public:

    ros::NodeHandle nh_pc2;
    ros::NodeHandle nh_pc2_pub;
    ros::Publisher pc2_pub;
    ros::Publisher ori_pc2_pub;

    message_filters::Subscriber<sensor_msgs::PointCloud2> *pc2_sub;
    tf::MessageFilter<sensor_msgs::PointCloud2> *tf_pc2_sub;
    tf::TransformListener tf_Listener;



public:
    Collector() {
        pc2_sub = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_pc2, "/camera/depth/points", 10);
        tf_pc2_sub = new tf::MessageFilter<sensor_msgs::PointCloud2>(*pc2_sub, tf_Listener, "odom", 10);
        tf_pc2_sub->registerCallback(boost::bind(&Collector::pointCloud2Callback, this, _1));
        pc2_pub = nh_pc2_pub.advertise<sensor_msgs::PointCloud2>("/obj_point_cloud", 10);
        ori_pc2_pub = nh_pc2_pub.advertise<sensor_msgs::PointCloud2>("/origin_point_cloud", 10);
    }

    ~Collector() {
        if(tf_pc2_sub)
            delete tf_pc2_sub;
        if(pc2_sub)
            delete pc2_sub;
    }

    void pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr &msg);

    template<typename T>
    void fromROSMsg(const sensor_msgs::PointCloud2 &cloud, pcl::PointCloud<T> &pcl_cloud);
};

#endif //SRC_COLLECTOR_H
