#include <iostream>
#include <string.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> //Quaternion
#include <geometry_msgs/TransformStamped.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>


class removeOutlier
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

    double cloud_quantity;
    double stddev;

public:
    removeOutlier();
};

removeOutlier::removeOutlier()
{
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/output", 1);
    cloud_sub = nh.subscribe("/camera/depth_registered/points", 1, &removeOutlier::cloud_callback, this);

    ros::NodeHandle pnh("~");
    pnh.param<std::string>("output_frame", output_frame, "output");
    pnh.param<std::string>("source_frame", source_frame, "camera_depth_optical_frame");

    pnh.param<double>("cloud_quantity", cloud_quantity, 50);
    pnh.param<double>("standart_deviation", stddev, 1.0);
}

void removeOutlier::tf_broadcast()
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

void removeOutlier::cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_message)
{

    //convert PointCloud2(ROS type) to PCLPointCloud2
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2Ptr cloudPtr(cloud);
    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_message, *cloud);

    //down sampling
    pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloudPtr);
    sor.setMeanK(cloud_quantity);
    sor.setStddevMulThresh(stddev);
    sor.filter(*cloudPtr);

    //convert PCLPointCloud2 to PointCloud2(ROS type)
    sensor_msgs::PointCloud2 output;
    pcl_conversions::moveFromPCL(*cloudPtr, output);
    output.header.frame_id = output_frame;
    cloud_pub.publish(output);

    //shift position
    tf_broadcast();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "removeOutlier");
    removeOutlier remove_outlier;
    ros::spin();
}