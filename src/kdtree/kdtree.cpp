#include <iostream>
#include <vector>
#include <string.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> //Quaternion
#include <geometry_msgs/TransformStamped.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>


class kdtree
{
private:
    ros::NodeHandle nh;

    //publisher
    ros::Publisher cloud_pub;
    //subscriber
    ros::Subscriber cloud_sub;

    //function
    void tf_broadcast();
    void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_message);

    std::string output_frame;
    std::string source_frame;

    double searchPoint_x;
    double searchPoint_y;
    double searchPoint_z;

    std::string search_method;

    int neighbor_number;
    double radius;
    int max_nn;

public:
    kdtree();
};

kdtree::kdtree()
{
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/output", 1);
    cloud_sub = nh.subscribe("/camera/depth_registered/points", 1, &kdtree::cloud_callback, this);

    ros::NodeHandle pnh("~");
    pnh.param<std::string>("output_frame", output_frame, "output");
    pnh.param<std::string>("source_frame", source_frame, "camera_depth_optical_frame");

    pnh.param<double>("searchPoint_x", searchPoint_x, 0.0);
    pnh.param<double>("searchPoint_y", searchPoint_y, 0.0);
    pnh.param<double>("searchPoint_z", searchPoint_z, 0.5);

    pnh.param<std::string>("search_method", search_method, "radius");

    pnh.param<int>("neighbor_number", neighbor_number, 1000);
    pnh.param<double>("radius", radius, 0.1);
    pnh.param<int>("max_nn", max_nn, 100);
}

void kdtree::tf_broadcast()
{
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.child_frame_id = output_frame;
    transformStamped.header.frame_id = source_frame;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.transform.translation.x = 2.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    transformStamped.transform.rotation.w = q.w();
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();

    static tf2_ros::TransformBroadcaster br;
    br.sendTransform(transformStamped);

}

void kdtree::cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_message)
{

    //convert PointCloud2(ROS type) to PointCloud(pcl)
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg(*cloud_message, cloud);

    //kdtree
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud(cloud.makeShared());

    pcl::PointXYZRGB searchPoint;
    searchPoint.x = searchPoint_x;
    searchPoint.y = searchPoint_y;
    searchPoint.z = searchPoint_z;

    std::vector<int> pointIdx(neighbor_number);
    std::vector<float> pointSquaredDistance(neighbor_number);
    if(search_method == "neighbor"){
        if(kdtree.nearestKSearch(searchPoint, neighbor_number, pointIdx, pointSquaredDistance)>0){
            for (size_t i = 0; i < pointIdx.size (); ++i){
                cloud.points[pointIdx[i]].r = 255;
                cloud.points[pointIdx[i]].g = 0;
                cloud.points[pointIdx[i]].b = 255;
            }
        }

    }else if(search_method == "radius"){
        if (kdtree.radiusSearch(searchPoint, radius, pointIdx, pointSquaredDistance, max_nn) > 0 ){
            for (size_t i = 0; i < pointIdx.size (); ++i){
                cloud.points[pointIdx[i]].r = 255;
                cloud.points[pointIdx[i]].g = 0;
                cloud.points[pointIdx[i]].b = 255;
            }
        }

    }else{
        ROS_ERROR("search method must be set 'neighbor' or 'radius'");
        return;
    }

    

    //convert PointCloud to PointCloud2(ROS type)
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud, output);
    //pcl_conversions::moveFromPCL(cloud, output);
    output.header.frame_id = output_frame;
    cloud_pub.publish(output);

    //shift position
    tf_broadcast();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "kdtree");
    kdtree kdtree;
    ros::spin();
}