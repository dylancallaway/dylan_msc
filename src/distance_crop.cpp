#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <dylan_msc/obj.h>

#include <pcl_conversions/pcl_conversions.h> // Required for pcl::fromROSMsg
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/passthrough.h>


typedef pcl::PointCloud<pcl::PointXYZ> PCLCloud;

ros::Publisher pcl_pub;

// Function called for each frame received
void frame_cb(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    static PCLCloud::Ptr cloud(new PCLCloud), cloud_filtered(new PCLCloud);

    // Convert PointCloud2 ROS msg to PCL PointCloud
    pcl::fromROSMsg(*msg, *cloud);

    // Passthrough filter
    // Removes points outside of a certain range
    // Filter z axis (depth)
    pcl::PassThrough<pcl::PointXYZ> pass; // create filter object
    pass.setInputCloud(cloud);            // set incoming cloud as input to filter
    pass.setFilterFieldName("z");         // filter on z axis (depth)
    pass.setFilterLimits(0.0f, 2.0f);
    pass.filter(*cloud_filtered); // returns cloud_filtered, the cloud with points outside of limits removed

    // Repeat for y axis (height)
    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-0.5f, 0.2f);
    pass.filter(*cloud_filtered);

    // Publish filtered point cloud
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_filtered, output);
    output.header = msg->header;
    pcl_pub.publish(output);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "distance_crop");
    ros::NodeHandle node;

    pcl_pub = node.advertise<PCLCloud>("/cropped_points", 2);

    ros::Subscriber sub = node.subscribe("/camera/depth/color/points", 2, frame_cb);

    ros::spin();

    return 0;
}
