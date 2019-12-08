//
// Created by hongsun on 19. 4. 12.
//

#ifndef UNCERTAIN_OBSTACLE_PROJECTOR_H
#define UNCERTAIN_OBSTACLE_PROJECTOR_H

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
#include "uncertain_obstacle/Matrix.h"

class Projector {
public:

    ros::NodeHandle nh_pc2;
    ros::NodeHandle nh_tfm;
    ros::Publisher tfm_pub;

    message_filters::Subscriber<sensor_msgs::PointCloud2> *pc2_sub;
    tf::MessageFilter<sensor_msgs::PointCloud2> *tf_pc2_sub;
    tf::TransformListener tf_Listener;


public:
    Projector() {
        pc2_sub = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_pc2, "/obj_point_cloud", 10);
        tf_pc2_sub = new tf::MessageFilter<sensor_msgs::PointCloud2>(*pc2_sub, tf_Listener, "camera_depth_optical_frame", 10);
        tf_pc2_sub->registerCallback(boost::bind(&Projector::pointCloud2Callback, this, _1));
        tfm_pub = nh_tfm.advertise<uncertain_obstacle::Matrix>("/transform_info", 10);
    }

    ~Projector() {
        if(tf_pc2_sub)
            delete tf_pc2_sub;
        if(pc2_sub)
            delete pc2_sub;
    }

    void pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr &msg);

};
#endif //UNCERTAIN_OBSTACLE_PROJECTOR_H
