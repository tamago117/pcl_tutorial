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
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/segmentation/sac_segmentation.h>


class extractIndices
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

    //downsampling
    double voxel_xsize;
    double voxel_ysize;
    double voxel_zsize;

    int maxIteration;
    double distanceThreshold;
    double extractRate;

public:
    extractIndices();
};

extractIndices::extractIndices()
{
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/output", 1);
    cloud_sub = nh.subscribe("/camera/depth_registered/points", 1, &extractIndices::cloud_callback, this);

    ros::NodeHandle pnh("~");
    pnh.param<std::string>("output_frame", output_frame, "output");
    pnh.param<std::string>("source_frame", source_frame, "camera_depth_optical_frame");

    pnh.param<double>("voxel_xsize", voxel_xsize, 0.1);
    pnh.param<double>("voxel_ysize", voxel_ysize, 0.1);
    pnh.param<double>("voxel_zsize", voxel_zsize, 0.1);

    pnh.param<int>("maxIteration", maxIteration, 1000);
    pnh.param<double>("distanceThreshold", distanceThreshold, 0.01);
    pnh.param<double>("extractRate", extractRate, 0.15);
}

void extractIndices::tf_broadcast()
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

void extractIndices::cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_message)
{

    //convert PointCloud2(ROS type) to PCLPointCloud2
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2Ptr cloudPtr(cloud);
    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_message, *cloud);

    //down sampling
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloudPtr);
    sor.setLeafSize(voxel_xsize, voxel_ysize, voxel_zsize);
    sor.filter(*cloudPtr);

    //convert PCLPointCloud2 to PointCloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromPCLPointCloud2(*cloudPtr, *cloud_filtered);

    //set coefficients
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    //segmentation
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    //optional
    seg.setOptimizeCoefficients(true);
    //mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIteration);
    seg.setDistanceThreshold(distanceThreshold);

    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_extracted(new pcl::PointCloud<pcl::PointXYZRGB>());

    int nr_points = (int)cloud_filtered->points.size();
    // While extractRate of the original cloud is still there
    while(cloud_filtered->points.size() > extractRate*nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size () == 0)
        {
            ROS_INFO("Could not estimate a planar model for the given dataset.");
            break;
        }

        // Extract the inliers
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*cloud_extracted);
        cloud_filtered.swap(cloud_extracted);

    }

    //convert PointCloud to PointCloud2(ROS type)
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_extracted, output);
    //pcl_conversions::moveFromPCL(cloud_extracted, output);
    output.header.frame_id = output_frame;
    cloud_pub.publish(output);

    //shift position
    tf_broadcast();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "extractIndices");
    extractIndices extract_indicies;
    ros::spin();
}