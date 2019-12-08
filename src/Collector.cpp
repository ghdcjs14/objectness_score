//
// Created by hongsun on 19. 4. 9.
//
#include <objectness_score/Collector.h>

void Collector::pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr &msg) {

    tf::StampedTransform sensorToWorldTf;
    try {
        tf_Listener.lookupTransform("odom", msg->header.frame_id, msg->header.stamp, sensorToWorldTf);
    } catch(tf::TransformException& e) {
        ROS_ERROR_STREAM("Transform error of sensor data: " << e.what() << ", quitting callback");
        return;
    }

    Eigen::Matrix4f sensorToWorld;
    pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);
//        std::cout << sensorToWorldTf.getOrigin().x() << " // "
//        << sensorToWorldTf.getOrigin().y() << " // "
//        << tf::getYaw(sensorToWorldTf.getRotation()) << std::endl;

    // sensor_msgs::PointCloud2 -> pcl::PointCloud
    // pcl::PointCloud<pcl::PointXYZ> -> Segmentation ERROR
    // change template to pcl::PointCloud<pcl::PointXYZRGB>
    pcl::PointCloud<pcl::PointXYZRGB> pc;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    try {
        pcl::fromROSMsg(*msg, pc);
//        pcl::fromROSMsg(*msg, *cloud);
//        Collector::fromROSMsg(*msg, *cloud);
    } catch(pcl::PCLException& e) {
        ROS_ERROR_STREAM("FromROSMsg error: " << e.what() << ", quitting callback");
        return;
    }

    // Convert pointcloud from sensor frame to world frame
    pcl::transformPointCloud(pc,pc,sensorToWorld);


    // Convert to ROS msg
    pcl::PCLPointCloud2 pcl_pc2;
    pcl::toPCLPointCloud2(pc,pcl_pc2);

    sensor_msgs::PointCloud2 msg_pc2;
    pcl_conversions::fromPCL(pcl_pc2, msg_pc2);
    msg_pc2.header.frame_id = "odom";
    msg_pc2.header.stamp = msg->header.stamp;

    pc2_pub.publish(msg_pc2);
    ori_pc2_pub.publish(msg);
}

template<typename T>
void Collector::fromROSMsg(const sensor_msgs::PointCloud2 &cloud, pcl::PointCloud<T> &pcl_cloud)
{

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(cloud, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, pcl_cloud);
}

//int main(int argc, char** argv)
//{
//    ros::init(argc, argv, "collector");
//
//    Collector collector;
//
//    try{
//        ros::spin();
//    }
//    catch (std::runtime_error& e){
//        ROS_ERROR("Exception: %s", e.what());
//        return -1;
//    }
//
//    return 0;
//}
