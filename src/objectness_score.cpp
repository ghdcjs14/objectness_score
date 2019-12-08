#include <time.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <iostream>
#include <vector>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf2_msgs/TFMessage.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/ImageMarker.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/CameraInfo.h>
#include <Eigen/Dense>
#include "objectness_score/Result.h"

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <objectness_score/Collector.h>

// FPFH
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/filters/filter.h>

#include <objectness_score/Projector.h>

// Surface
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <shape_msgs/Mesh.h>
#include <pcl_msgs/PolygonMesh.h>
#include <pcl/PolygonMesh.h>


using namespace std;
using namespace Eigen;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class WorldBox {
public:
    float x_offset;
    float y_offset;
    float z_offset;

    float x_rightTopPt;
    float y_rightTopPt;
    float z_rightTopPt;

    float x_leftBottomPt;
    float y_leftBottomPt;
    float z_leftBottomPt;

    float x_endPt;
    float y_endPt;
    float z_endPt;
};

class BoxFromWorld {
public:
    int x_offset;
    int y_offset;
    int width;
    int height;
};

class Obstacle {
public:
    int idx;
    int classId;
    float obsScore;
    float origin_depth;
    float depth;
    string className;
    sensor_msgs::RegionOfInterest box;
    WorldBox worldBox;
    BoxFromWorld boxFromWorld;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs;

    Obstacle(){}
    Obstacle(int idx, int classId, float obsScore, string className, sensor_msgs::RegionOfInterest box) {
        this->idx = idx;
        this->classId = classId;
        this->obsScore = obsScore;
        this->className = className;
        this->box = box;
        this->worldBox.x_offset = box.x_offset;
        this->worldBox.y_offset = box.y_offset;
        this->boxFromWorld.x_offset = box.x_offset;
        this->boxFromWorld.y_offset = box.y_offset;
        this->depth = 1;
        this->origin_depth = 1;
    }
};

vector<Obstacle> obstacles;
nav_msgs::OccupancyGrid Occupancy_Map;
nav_msgs::Odometry odom;
nav_msgs::Path path;

ros::Publisher path_pub;
ros::Publisher map_pub;
ros::Publisher pos_pub;
ros::Publisher marker_pub;
ros::Publisher mesh_pub;
ros::Publisher obs_pc2_pub;


const int UNKNOWN = -1;
const int FREE = 0;
const int OBSTACLE = 50;
const int OCCUPIED = 100;

const float RESOLUTION = 0.25f;  // meter/pixel, 1 pixel is 25cm
const int MAX_X = 40;
const int MAX_Y = 40;
const int MAP_ORIGIN_X = -5;    // meter
const int MAP_ORIGIN_Y = -5;    // meter

int **Grid_Map;

cv::Mat Depth_Map;              // 0.8 ~ 4 m
Matrix3f K;                     // Intrinsics
Matrix4f Rt_World_To_Sensor;    // Extrinsics

pcl::PointCloud<pcl::PointXYZRGB> Pcl_PC;
pcl::PointCloud<pcl::PointXYZRGB> Cam_Pcl_PC;
pcl::PointCloud<pcl::FPFHSignature33>::Ptr FPFHS;

// Real Kinect
//('D = ', [0.229848593769665, -0.4949684644431577, -0.02354454817547323, -0.023518774158603324, 0.0])
//('K = ', [530.4211126502115, 0.0, 281.11114519976735, 0.0, 529.746879079305, 244.4823971390676, 0.0, 0.0, 1.0])
//('R = ', [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
//('P = ', [-196.09580993652344, 0.0, -96.66840851784627, 0.0, 0.0, -274.26727294921875, 601.5634902485908, 0.0, 0.0, 0.0, 1.0, 0.0])

// Gazebo camera;
const float FX = 554.254691191187f, FY = 554.254691191187f, TX = 320.5f, TY=240.5f;
float Pos_X=0.0f, Pos_Y=0.0f;
float Ori_Theta=0.0f;

// Point Cloud 2
const int WIDTH = 640;
const int HEIGHT = 480;

bool bFlagFpfh = false;

void setObstacleInOccupancyMap(double x, double y){
    int grid_x, grid_y;

    grid_x = (int)(MAX_X/2 + x);
    grid_y = (int)(MAX_Y/2 + y);

//    cout << x << ", " << y << endl;
    if(grid_x > 0 && grid_x < MAX_X && grid_y > 0  && grid_y < MAX_Y)
        Grid_Map[grid_x][grid_y] = OBSTACLE;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        // get time
        clock_t begin, end;
        begin = clock();    // micro second...

        /// Start function...
        cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

        if(!obstacles.empty()) {
            // Covert parameters
            Vector4f worldOffset, worldEndPt;
            Vector3f imgOffset, imgEndPt;
            MatrixXf projection(3,4);
            int img_x_offset, img_y_offset, img_x_endPt, img_y_endPt;

            projection << 1,0,0,0,
                    0,1,0,0,
                    0,0,1,0;

            // Marker
            visualization_msgs::Marker worldObs;
            worldObs.points.clear();
            worldObs.header.frame_id="odom";
            worldObs.header.stamp=ros::Time::now();
            worldObs.ns="Obstacles";
            worldObs.id=0;
            worldObs.type=visualization_msgs::Marker::POINTS;
            worldObs.scale.x=0.3;
            worldObs.scale.y=0.3;
            worldObs.scale.z=0.3;
            worldObs.color.r=1;
            worldObs.color.a=1;

            // Set Obstacle's 3D Marker
            geometry_msgs::Point p;

            // Draw the detected objects
            vector<Obstacle>::iterator itr;
            for(itr=obstacles.begin(); itr!=obstacles.end(); ++itr) {

                // The ROI of Mask R-CNN
//                cv::rectangle(image,cv::Point(itr->box.x_offset,itr->box.y_offset),
//                        cv::Point((itr->box.x_offset+itr->box.width),(itr->box.y_offset+itr->box.height)),(0,0,255),2);

//                cv::putText(image, itr->className.c_str(), cv::Point(itr->box.x_offset,itr->box.y_offset),
//                            CV_FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255,255,255));

//                ROS_INFO("box_offset (%d, %d)", itr->box.x_offset, itr->box.y_offset);
//                ROS_INFO("box_endPt (%d, %d)", itr->box.x_offset+itr->box.width, itr->box.y_offset+itr->box.height);


                //// Get the image points from the world points
                worldOffset << itr->worldBox.x_offset, itr->worldBox.y_offset, itr->worldBox.z_offset,1;
                worldEndPt << itr->worldBox.x_endPt, itr->worldBox.y_endPt, itr->worldBox.z_endPt,1;

                // Change the world coordinates to the image coordinates
                MatrixXf k_proj(3,4), k_proj2(3,4);
                k_proj = K*projection;
                k_proj2 = K*projection;
//                cout << "k_proj : " << endl;
//                cout << k_proj << endl;
//                cout << k_proj2 <<endl;

                MatrixXf k_proj_rt(3,4), k_proj_rt2(3,4);
                k_proj_rt = k_proj * Rt_World_To_Sensor;
                k_proj_rt2 = k_proj2 * Rt_World_To_Sensor;
//                cout << "k_proj_rt : " << endl;
//                cout << k_proj_rt << endl;
//                cout << k_proj_rt2 <<endl;

                imgOffset = k_proj_rt*worldOffset;
//                imgOffset = K*projection*Rt_World_To_Sensor*worldOffset;
//                cout << "imgOffset Rt: " << Rt_World_To_Sensor <<endl;
                imgEndPt = k_proj_rt2*worldEndPt;
//                imgEndPt = K*projection*Rt_World_To_Sensor*worldEndPt;
//                cout << "imgEndPt Rt: " << Rt_World_To_Sensor <<endl;

//                cout << "imgOffset: " << imgOffset << endl;
//                cout << "imgEndPt: " << imgEndPt << endl;

//                cout << "world_offset: "  << itr->worldBox.x_offset <<", "<< itr->worldBox.y_offset << ", " << itr->worldBox.z_offset << endl;
//                cout << "world_endPt: "  << itr->worldBox.x_endPt <<", "<< itr->worldBox.y_endPt << ", " << itr->worldBox.z_endPt << endl;
//                cout << "Rt_World_To_Sensor: " << Rt_World_To_Sensor << endl;

                img_x_offset = (int)(imgOffset(0,0)/imgOffset(2,0));
                img_y_offset = (int)(imgOffset(1,0)/imgOffset(2,0));
//                img_x_endPt = (int)(imgEndPt(0,0)/imgEndPt(2,0));
//                img_x_endPt = (int)(imgEndPt(1,0)/imgEndPt(2,0));
                // if z-axis is the same
                img_x_endPt = (int)(imgEndPt(0,0)/imgEndPt(2,0));
                img_y_endPt = (int)(imgEndPt(1,0)/imgEndPt(2,0));

//                ROS_INFO("box_from_world_offset (%d, %d)", img_x_offset, img_y_offset);
//                ROS_INFO("box_from_world_endPt (%d, %d)", img_x_endPt, img_y_endPt);
//
//                cout << "box_from_world_offset "<< img_x_offset<< img_y_offset<<endl;
//                cout << "box_from_world_endPt "<< img_x_endPt<< img_y_endPt<<endl;


                //// 1. Check if the detected obstacles are in the image plan
//                if(img_x_offset > 0 && img_x_endPt < WIDTH
//                   && img_y_offset > 0 && img_y_endPt < HEIGHT) {

                    cv::rectangle(image,cv::Point(img_x_offset,img_y_offset),
                                  cv::Point(img_x_endPt,img_y_endPt),(0,0,255),2);

                    cv::putText(image, itr->className.c_str(), cv::Point(img_x_offset,(img_y_offset+15)),
                            CV_FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255,255,255));
//                }



                // Set Obstacle's 3D Marker
                geometry_msgs::Point p;
                if(!pcl_isnan(p.x)) {
                    p.x = itr->worldBox.x_offset;
                    p.y = itr->worldBox.y_offset;
                    p.z = itr->worldBox.z_offset;
//                    cout << "world_point (" <<p.x<<","<<p.y<<","<<p.z<< ")" <<endl;
                }

                // Set Obstacle's 3D Marker
//                geometry_msgs::Point p2;
//                if(!pcl_isnan(p2.x)) {
//                    p2.x = itr->worldBox.x_endPt;
//                    p2.y = itr->worldBox.y_endPt;
//                    p2.z = itr->worldBox.z_endPt;
//                    cout << "world_point (" <<p.x<<","<<p.y<<","<<p.z<< ")" <<endl;
//                }

                worldObs.points.push_back(p);
//                worldObs.points.push_back(p2);

                // Set Obstacle's 2D point in Occupancy Map
                setObstacleInOccupancyMap(itr->worldBox.x_offset,itr->worldBox.y_offset);
            }
            marker_pub.publish(worldObs);
        }

        // get time
        end = clock();
        cout<<"imageCallback time duration(ms) : " << (double)(end-begin)/1000 << endl;

        cv::imshow("rgb_view", image);
        cv::waitKey(30);

    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void depthCallback(const sensor_msgs::ImageConstPtr& msg) {

    // get time
    clock_t begin, end;
    begin = clock();

    try {
        cv::Mat adjMap, obstacleRoi;
        cv::Mat depthMap = cv_bridge::toCvShare(msg, "32FC1")->image;
        Depth_Map = depthMap;
        double depth = 0;
        // double min, max;

        if(!obstacles.empty()) {
            vector<Obstacle>::iterator itr;
            for(itr=obstacles.begin(); itr < obstacles.end(); ++itr) {

                if(itr->box.x_offset>0 && itr->box.y_offset>0
                        && itr->box.x_offset+itr->box.width < depthMap.rows && itr->box.y_offset+itr->box.height < depthMap.cols) {

                    cv::Rect obstacleRoiRect(itr->box.x_offset,itr->box.y_offset,itr->box.width,itr->box.height);
                    obstacleRoi = depthMap(obstacleRoiRect);

//                    cv::patchNaNs(obstacleRoi);
                    //cv::minMaxIdx(obstacleRoi, &min, &max);

//                    cout << obstacleRoi << endl;
                    /// Set obstacle' depth info.
                    depth = cv::mean(obstacleRoi).val[0];
                    depth = depth < 0.8 || depth > 4.0 ? 0 : depth;
                    itr->depth = depth;
//                    ROS_INFO("depth: %.2f",itr->depth);
                }
            }
        }

        // imshow() support 16bit unsigned images
        // map is [0, 255*256], adjMap is [0, 255]
//        cv::normalize(depthMap,adjMap,0,255,CV_MINMAX);
//        cv::imshow("depth_view", adjMap);
//        cv::waitKey(30);

    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to '32FC1'.", msg->encoding.c_str());
    }

    end = clock();
    cout<<"depthCallback time duration(ms) : " << (double)(end-begin)/1000 << endl;
}

void setWorldBoxInfo(Obstacle* obstacle, int x_offset, int y_offset, int width, int height) {

    obstacle->worldBox.x_offset = Pcl_PC.points[(5+y_offset)*WIDTH + (x_offset+5)].x;
    obstacle->worldBox.y_offset = Pcl_PC.points[(5+y_offset)*WIDTH + (x_offset+5)].y;
    obstacle->worldBox.z_offset = Pcl_PC.points[(5+y_offset)*WIDTH + (x_offset+5)].z;

    obstacle->worldBox.x_endPt = Pcl_PC.points[(y_offset+height-5)*WIDTH + (x_offset+width-5)].x;
    obstacle->worldBox.y_endPt = Pcl_PC.points[(y_offset+height-5)*WIDTH + (x_offset+width-5)].y;
    obstacle->worldBox.z_endPt = Pcl_PC.points[(y_offset+height-5)*WIDTH + (x_offset+width-5)].z;
}

double euclidean(vector<Obstacle>::iterator maintained_obstacle, Obstacle* obstacle) {

    double diff_x = maintained_obstacle->worldBox.x_offset - obstacle->worldBox.x_offset;
    double diff_y = maintained_obstacle->worldBox.y_offset - obstacle->worldBox.y_offset;
    double diff_z = maintained_obstacle->worldBox.z_offset - obstacle->worldBox.z_offset;

    double dist = sqrt(pow(diff_x,2) + pow(diff_y,2) + pow(diff_z,2));

    return dist;
}

bool rule(pair<int,double> a, pair<int,double> b) {
    return a.second < b.second;
}

void sortByValue(map<int,double>& dist) {
    vector<pair<int, double> > vec(dist.begin(), dist.end());

    cout << "dist_size: " << dist.size() << endl;
    copy(dist.begin(), dist.end(), vec.begin());

    sort(vec.begin(), vec.end(), rule);

//    for(int i=0; i<vec.size(); i++) {
//        cout << "idx: " << vec[i].first << ", value: " << vec[i].second << endl;
//    }
}

void searchKNN(int k, Obstacle* obstacle, vector<Obstacle>* knnObs) {

    if(obstacles.size() <= k) {
        for(int i=0; i<obstacles.size(); i++) {
            knnObs->push_back(obstacles[i]);
        }

    } else {
        map<int, double> dist;

        // 1. caculate the distances of detected objects
        vector<Obstacle>::iterator itr;
        for(itr=obstacles.begin(); itr < obstacles.end(); ++itr) {
            dist.insert(make_pair(itr->idx, euclidean(itr, obstacle)));
        }

        // 2. sort the distances
        sortByValue(dist);

        // 3. insert k-nearest neighbors
        map<int,double>::iterator itr_map;
        for(int i=0; i<k; i++, ++itr_map) {
            knnObs->push_back(obstacles[itr_map->first]);
        }

        vector<Obstacle>::iterator itr2;
        for(itr2=knnObs->begin(); itr2 < knnObs->end(); ++itr2) {
            cout << "idx: " << itr2->idx << " classNm: " << itr2->className << endl;
        }
    }
}

void checkObstacles(Obstacle* obstacle) {

    // get time
    clock_t begin, end;
    begin = clock();

    vector<Obstacle>::iterator itr;
    float maxIou = 0.0f;
    int maxIdx = 0;
    float maxTotal = 0;

    int x_offset = obstacle->box.x_offset;
	int y_offset = obstacle->box.y_offset;
	int width = obstacle->box.width;
	int height = obstacle->box.height;
	double depth = 0;

    // Covert parameters
    Vector4f worldOffset, worldEndPt;
    Vector3f imgOffset, imgEndPt;
    MatrixXf projection(3,4);
    int img_x_offset, img_y_offset, img_x_endPt, img_y_endPt;

    projection << 1,0,0,0,
            0,1,0,0,
            0,0,1,0;

	// Check Obstacles' Depth
    if(x_offset>0 && y_offset>0
       && x_offset+width < Depth_Map.rows && y_offset+height < Depth_Map.cols) {

        cv::Rect obstacleRoiRect(x_offset,y_offset,width,height);
        cv::Mat obstacleRoi = Depth_Map(obstacleRoiRect);

        //cout << obstacleRoi << endl;
        cv::patchNaNs(obstacleRoi);
        /// Set obstacle' depth info.
        depth = cv::mean(obstacleRoi).val[0];
        depth = depth < 0.8 || depth > 3.0 ? 0 : depth;
    }

    if(depth == 0 ) {
        return;
    }

//    cout << "depth: " << depth << endl;
    obstacle->depth = depth;

    //// Store World ROI Information
    setWorldBoxInfo(obstacle, x_offset, y_offset, width, height);

	// First obstacles
    if(itr == obstacles.end() && depth>0) {

        setObstacleInOccupancyMap(obstacle->worldBox.x_offset, obstacle->worldBox.y_offset);
        obstacles.push_back(*obstacle);

        ++itr;
        ROS_INFO("%s_%d is added(%.2f,%.2f,%.2f)", obstacle->className.c_str(), obstacle->idx,
                 obstacle->worldBox.x_offset,obstacle->worldBox.y_offset,obstacle->worldBox.z_offset);
        return;
    }

    // Select K-Nearest Neighbor objects
    int k = 3;
    vector<Obstacle> knnObs;
    searchKNN(k, obstacle, &knnObs);

    // Compare pre-detected obstacles
//    for(itr=obstacles.begin(); itr < obstacles.end(); ++itr) {
    for(itr=knnObs.begin(); itr < knnObs.end(); ++itr) {

        //// Get the image points from the world points
        worldOffset << itr->worldBox.x_offset, itr->worldBox.y_offset, itr->worldBox.z_offset,1;
        worldEndPt << itr->worldBox.x_endPt, itr->worldBox.y_endPt, itr->worldBox.z_endPt,1;

        // Change the world coordinates to the image coordinates
        imgOffset = K*projection*Rt_World_To_Sensor*worldOffset;
        imgEndPt = K*projection*Rt_World_To_Sensor*worldEndPt;

        img_x_offset = (int)(imgOffset(0,0)/imgOffset(2,0));
        img_y_offset = (int)(imgOffset(1,0)/imgOffset(2,0));
        img_x_endPt = (int)(imgEndPt(0,0)/imgEndPt(2,0));
        img_y_endPt = (int)(imgEndPt(1,0)/imgEndPt(2,0));

        //// 1. Check if the detected obstacles are in the image plan
        if(img_x_offset > 0 && img_x_endPt < WIDTH
                && img_y_offset > 0 && img_y_endPt < HEIGHT) {


            // 1-1. if IOU is over 50%
            // determine the (x,y) of intersection rectangle
            float xA = max(x_offset, img_x_offset);
            float yA = max(y_offset, img_y_offset);
            float xB = min(x_offset+width, img_x_endPt);
            float yB = min(y_offset+height, img_y_endPt);

            float interArea = max(0.0f,xB-xA+1)*max(0.0f,yB-yA+1);
            float boxAArea = width * height;
            float boxBArea = (img_x_endPt-img_x_offset) * (img_y_endPt-img_y_offset);

            float iou = interArea / (boxAArea + boxBArea - interArea);

            if(iou>maxIou) {
                maxIou = iou;
                maxIdx = std::distance(obstacles.begin(), itr);
            }

            float alpha = 0.5;
            float epsilon = 1;
            float minDepth = 0.8;
            float maxDepth = 4;

            ROS_INFO("IOU: %.2f", iou);
            //// Calculate the Objectness Score
            float obs_norm_d = obstacle->depth - minDepth / (maxDepth - minDepth);
            float obs_total = (alpha * obstacle->obsScore + (1-alpha) * obs_norm_d)/(iou * 100 + epsilon);

            if(obs_total > maxTotal)
                maxTotal = obs_total;

            if(iou > 0.9) {

                float itr_norm_d = itr->depth - minDepth / (maxDepth - minDepth);
                float itr_total = (alpha * obstacle->obsScore + (1-alpha) * itr_norm_d)/(iou * 100 + epsilon);

                if(obs_total > itr_total) {
                    ROS_INFO("IOU: %.2f", iou);
                    ROS_INFO("%s_%d is replaced(%.2f,%.2f,%.2f)", obstacle->className.c_str(), obstacle->idx,
                             obstacle->worldBox.x_offset,obstacle->worldBox.y_offset,obstacle->worldBox.z_offset);

                    setObstacleInOccupancyMap(obstacle->worldBox.x_offset, obstacle->worldBox.y_offset);

                    obstacles.at(maxIdx) = *obstacle;
                }
            }

//            if(maxIou > 0.8 && obstacle->obsScore > obstacles.at(maxIdx).obsScore && obstacle->depth>0) {
//                ROS_INFO("%s_%d is replaced", obstacle->className.c_str(), obstacle->idx);
//
//                obstacles.at(maxIdx) = *obstacle;
//            }
        }
    }

    ROS_INFO("Max Total = %.2f", maxTotal);
    if(maxTotal > 0.8 && obstacle->depth>0) {
        ROS_INFO("%s_%d is added(%.2f,%.2f,%.2f)", obstacle->className.c_str(), obstacle->idx,
                 obstacle->worldBox.x_offset,obstacle->worldBox.y_offset,obstacle->worldBox.z_offset);

        setObstacleInOccupancyMap(obstacle->worldBox.x_offset, obstacle->worldBox.y_offset);

        obstacles.push_back(*obstacle);
    }

    end = clock();
    cout<<"Time duration : " << (float)(end-begin)/CLOCKS_PER_SEC << endl;
}

void publishObstaclePointCloud(sensor_msgs::RegionOfInterest box) {

    PointCloud::Ptr msg (new PointCloud);
    msg->header.frame_id = "odom";
//    msg->header.stamp = ros::Time::now();
    msg->height = box.height;
    msg->width = box.width;
//    msg->points.push_back(pcl::PointXYZ(1.0,2.0,3.0));
    for(int i=0; i<box.width; i++) {
        for(int j=0; j<box.height; j++) {

            msg->points.push_back(pcl::PointXYZ(
                    Pcl_PC.points[(box.y_offset + i)*WIDTH + (box.x_offset + j)].x,
                    Pcl_PC.points[(box.y_offset + i)*WIDTH + (box.x_offset + j)].y,
                    Pcl_PC.points[(box.y_offset + i)*WIDTH + (box.x_offset + j)].z));
        }
    }
    obs_pc2_pub.publish(msg);
}

void maskRcnnCallback(const objectness_score::Result::ConstPtr& msg) {

    // get time
    clock_t begin, end;
    begin = clock();

    // Parameters are made by vector(array)
    int numClass = msg->boxes.size();
    int classIds[numClass];
    string classNames[numClass];
    float scores[numClass];
    sensor_msgs::RegionOfInterest boxes[numClass];

    // Initialization
    for(int i=0; i<numClass; i++) {
        boxes[i] = msg->boxes[i];
        classIds[i] = msg->class_ids[i];
        classNames[i] = msg->class_names[i];
        scores[i] = msg->scores[i];

        checkObstacles(new Obstacle(i, msg->class_ids[i], msg->scores[i], msg->class_names[i], msg->boxes[i]));

        publishObstaclePointCloud(msg->boxes[i]);

    }

    end = clock();
    cout<<"maskRcnnCallback Time duration(ms) : " << (double)(end-begin)/1000 << endl;
}

// calculate z-axis rotation from Odom_Quaternion(z, w) -180~+180
double quaternion_to_enler(double z, double w)
{
  double sqw, sqz;
  sqw = w * w;
  sqz = z * z;
  return atan2(2.0 * (z * w), (-sqz + sqw));
}

double rad2deg(double x) {
    return ((x) * (180.0) / M_PI);
}

double deg2rad(double x) {
    return ((x) * M_PI / (180.0));
}

void publishOdom(double posX, double posY, geometry_msgs::Quaternion orientation) {

    geometry_msgs::Quaternion odom_quat = orientation;
    odom.header.frame_id = "odom";
    odom.header.stamp = ros::Time::now();
    odom.pose.pose.position.x = posX;
    odom.pose.pose.position.y = posY;
    odom.pose.pose.orientation = odom_quat;
    pos_pub.publish(odom);
}

void publishPath(double posX, double posY) {
    geometry_msgs::PoseStamped pose;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "odom";
    pose.pose.position.x = posX;
    pose.pose.position.y = posY;
    path.poses.push_back(pose);
    path_pub.publish(path);
}

void publishOccupancyMap() {
    Occupancy_Map.header.frame_id = "odom";
    Occupancy_Map.header.stamp = ros::Time::now();
    Occupancy_Map.info.width = MAX_X;
    Occupancy_Map.info.height = MAX_Y;
    Occupancy_Map.info.resolution = RESOLUTION;
    Occupancy_Map.info.origin.position.x = MAP_ORIGIN_X; // meter
    Occupancy_Map.info.origin.position.y = MAP_ORIGIN_Y;
    Occupancy_Map.info.origin.position.z = 0.0;
    Occupancy_Map.info.origin.orientation.x = 0.0;
    Occupancy_Map.info.origin.orientation.y = 0.0;
    Occupancy_Map.info.origin.orientation.z = 0.0;
    Occupancy_Map.info.origin.orientation.w = 1.0;

    Occupancy_Map.data.clear();
    for(int i=0; i<MAX_X; i++) {
        for(int j=0; j<MAX_Y; j++) {

            if(Grid_Map[j][i] == UNKNOWN) {
                Occupancy_Map.data.push_back(UNKNOWN);
            }
            else if(Grid_Map[j][i] == OCCUPIED) {
                Occupancy_Map.data.push_back(OCCUPIED);
            }
            else if(Grid_Map[j][i] == OBSTACLE) {
                Occupancy_Map.data.push_back(OBSTACLE);
            }
            else {
                Occupancy_Map.data.push_back(FREE);
            }
        }
    }
    map_pub.publish(Occupancy_Map);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {

    Pos_X = msg->pose.pose.position.x;
    Pos_Y = msg->pose.pose.position.y;

    geometry_msgs::Quaternion orientation = msg->pose.pose.orientation;
    double z = msg->pose.pose.orientation.z;
    double w = msg->pose.pose.orientation.w;
    double temp_th = quaternion_to_enler(z,w);
    Ori_Theta = rad2deg(temp_th);

    int grid_x, grid_y;
    grid_x = (int)(MAX_X/2 + Pos_X/RESOLUTION);
    grid_y = (int)(MAX_Y/2 + Pos_Y/RESOLUTION);


    if(grid_x > 0 && grid_x < MAX_X && grid_y > 0  && grid_y < MAX_Y)
        Grid_Map[grid_x][grid_y] = FREE;

    // publish Odometer, Path, Occupancy Map
    publishOdom(Pos_X, Pos_Y, orientation);
	publishPath(Pos_X, Pos_Y);
    publishOccupancyMap();

}

void camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg) {

    // get time
    clock_t begin, end;
    begin = clock();

    // Parameters are made by vector(array)
    for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++) {
                K(i,j) = (float)msg->K[i*3+j];
        }
    }

    end = clock();
    cout<<"camInfoCallback time duration(ms) : " << (double)(end-begin)/1000 << endl;
}

void pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr &msg) {

    clock_t begin, end;
    begin = clock();

    // sensor_msgs::PointCloud2 -> pcl::PointCloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *cloud_rgb);

    Pcl_PC = *cloud_rgb;

    end = clock();
    cout<<"pointCloud2Callback time duration(ms) : " << (double)(end-begin)/1000 << endl;
}

void transformCallback(const objectness_score::MatrixConstPtr &msg) {
    clock_t begin, end;
    begin = clock();

    for(int i=0; i<4; i++) {
        for(int j=0; j<4; j++) {
            Rt_World_To_Sensor(i,j) = msg->transform[i*4 + j];
        }
    }

    end = clock();
    cout<<"transformCallback time duration(ms) : " << (double)(end-begin)/1000 << endl;
    cout<<"=================================================================="<< endl;

//    cout << "RT_WORLD_TO_SENSOR "<< Rt_World_To_Sensor << endl;
}

void initMap() {
    Grid_Map = new int*[MAX_X];
    for(int i = 0; i<MAX_X; i++){
        Grid_Map[i] = new int [MAX_Y] ;
    }

    for(int i=0; i<MAX_X; i++) {
        for(int j=0; j<MAX_Y; j++) {
            Grid_Map[i][j] = UNKNOWN;
        }
    }
}

void clearMap() {
    for(int i=0; i<MAX_X; i++) {
        delete []Grid_Map[i];
    }
    delete [] Grid_Map;
}

int main(int argc, char* argv[]) {

    initMap();

    ros::init(argc, argv, "objectness_score");

    Collector collector;    // Publish the Point Cloud(World Coordinates)
    Projector projector;    // Publish the Transformation Matrix(Image Coordinates)

    ros::NodeHandle nh_rgbd;
    ros::NodeHandle nh_mask_rcnn;
    ros::NodeHandle nh_odom;
    ros::NodeHandle nh_cam_info;
    ros::NodeHandle nh_marker_pub;
    ros::NodeHandle nh_odom_pub;
    ros::NodeHandle nh_map_pub;
    ros::NodeHandle nh_path_pub;
    ros::NodeHandle nh_pc2;
    ros::NodeHandle nh_pc2_pub;
    ros::NodeHandle nh_tfm;
    ros::NodeHandle nh_mesh_pub;
    ros::NodeHandle nh_obs_pc2_pub;

    cv::namedWindow("rgb_view");
//    cv::namedWindow("depth_view");
    cv::startWindowThread();

    //// Image Transport
    image_transport::ImageTransport it(nh_rgbd);
    image_transport::Subscriber rgb_sub = it.subscribe("/camera/rgb/image_raw", 1, imageCallback);
    image_transport::Subscriber depth_sub = it.subscribe("/camera/depth/image_raw", 1, depthCallback);

    //// Subscriber
    ros::Subscriber mask_rcnn_sub = nh_mask_rcnn.subscribe("/mask_rcnn/result", 1, maskRcnnCallback);
    ros::Subscriber cam_info_sub = nh_cam_info.subscribe("/camera/rgb/camera_info", 1, camInfoCallback);
    ros::Subscriber odom_sub = nh_odom.subscribe("/odom", 1, odomCallback);

    ros::Subscriber pc2_sub = nh_pc2.subscribe<sensor_msgs::PointCloud2>("/obj_point_cloud", 1, pointCloud2Callback);
    ros::Subscriber tfm_sub = nh_tfm.subscribe<objectness_score::Matrix>("/transform_info", 1, transformCallback);

    //// Publisher
    marker_pub = nh_marker_pub.advertise<visualization_msgs::Marker>("/obstacles_points",1);
    pos_pub = nh_odom_pub.advertise<nav_msgs::Odometry>("/nav_odometry",1);
    path_pub = nh_map_pub.advertise<nav_msgs::Path>("/path_moved",1);
    map_pub = nh_map_pub.advertise<nav_msgs::OccupancyGrid>("/map_occupancy",1);
    mesh_pub = nh_mesh_pub.advertise<shape_msgs::Mesh>("/mesh",1);

    obs_pc2_pub = nh_obs_pc2_pub.advertise<sensor_msgs::PointCloud2>("/obs_pc2",100);

    ROS_INFO("Uncertain Obstacle Detection Started...");


    ros::spin();

    cv::destroyWindow("rgb_view");
//    cv::destroyWindow("depth_view");

    clearMap();

    return 0;
}
