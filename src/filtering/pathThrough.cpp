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
#include <pcl/filters/passthrough.h>


class pathThrough
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

    bool set_xfilter;
    bool set_yfilter;
    bool set_zfilter;

    double filter_xmin;
    double filter_xmax;
    double filter_ymin;
    double filter_ymax;
    double filter_zmin;
    double filter_zmax;


public:
    pathThrough();
};

pathThrough::pathThrough()
{
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/output", 1);
    cloud_sub = nh.subscribe("/camera/depth_registered/points", 1, &pathThrough::cloud_callback, this);

    ros::NodeHandle pnh("~");
    pnh.param<std::string>("output_frame", output_frame, "output");
    pnh.param<std::string>("source_frame", source_frame, "camera_depth_optical_frame");

    pnh.param<bool>("set_xfilter", set_xfilter, "false");
    pnh.param<bool>("set_yfilter", set_yfilter, "false");
    pnh.param<bool>("set_zfilter", set_zfilter, "false");

    pnh.param<double>("filter_xmin", filter_xmin, -5.0);
    pnh.param<double>("filter_xmax", filter_xmax, 5.0);
    pnh.param<double>("filter_ymin", filter_ymin, -5.0);
    pnh.param<double>("filter_ymax", filter_ymax, 5.0);
    pnh.param<double>("filter_zmin", filter_zmin, 0.0);
    pnh.param<double>("filter_zmax", filter_zmax, 2.0);
}

void pathThrough::tf_broadcast()
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

void pathThrough::cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_message)
{
    //path through
    //https://pcl.readthedocs.io/projects/tutorials/en/latest/passthrough.html#passthrough

    //convert PointCloud2(ROS type) to PCLPointCloud2
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;
    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_message, *cloud);

    //filter
    pcl::PassThrough<pcl::PCLPointCloud2> pass;
    pass.setInputCloud(cloudPtr);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(filter_zmin, filter_zmax);
    pass.filter(cloud_filtered);

    //convert PCLPointCloud2 to PointCloud2(ROS type)
    sensor_msgs::PointCloud2 output;
    pcl_conversions::moveFromPCL(cloud_filtered, output);
    output.header.frame_id = output_frame;
    cloud_pub.publish(output);

    //shift position
    tf_broadcast();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pathThrough");
    pathThrough path_through;
    ros::spin();
}