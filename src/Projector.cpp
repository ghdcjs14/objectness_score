//
// Created by hongsun on 19. 4. 12.
//

#include <objectness_score/Projector.h>

void Projector::pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr &msg) {

    tf::StampedTransform worldToSensorTf;
    try {
        tf_Listener.lookupTransform("camera_depth_optical_frame", msg->header.frame_id, msg->header.stamp, worldToSensorTf);
    } catch(tf::TransformException& e) {
        ROS_ERROR_STREAM("Transform error of sensor data: " << e.what() << ", quitting callback");
        return;
    }

    Eigen::Matrix4f worldToSensor;
    pcl_ros::transformAsMatrix(worldToSensorTf, worldToSensor);
//        std::cout << sensorToWorldTf.getOrigin().x() << " // "
//        << sensorToWorldTf.getOrigin().y() << " // "
//        << tf::getYaw(sensorToWorldTf.getRotation()) << std::endl;

//    std::cout << "worldTOSensor " << worldToSensor << std::endl;

    objectness_score::Matrix transform_msg;

    for(int i=0; i<4; i++) {
        for(int j=0; j<4; j++) {
            transform_msg.transform.push_back(worldToSensor(i,j));
        }
    }


    tfm_pub.publish(transform_msg);
}

