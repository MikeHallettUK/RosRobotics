#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>

ros::Publisher pub;
laser_geometry::LaserProjection projector_;
tf2_ros::Buffer tfBuffer;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
    sensor_msgs::PointCloud2 cloud;
    ros::Duration(0.05).sleep(); // wait for first odom @ 30 Hz after scan ... = fixes extrapolation into the future error.
                                 // @ 0.04 seconds ... still getting some warnings ....
                                 // inscan->time_increment // needs to be negative to account for clockwise spin of lidar
    try
    {
        projector_.transformLaserScanToPointCloud("odom", *scan_in, cloud, tfBuffer);
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
    }

    pub.publish(cloud);         // Publish the data.
    }

    int main(int argc, char** argv)
    {
        // Initialize ROS
        ros::init(argc, argv, "tf_laser");
        ros::NodeHandle nh;
        tf2_ros::TransformListener tfListener(tfBuffer);

        // Create a ROS publisher for the output point cloud
        pub = nh.advertise<sensor_msgs::PointCloud2>("tf_pc", 1);

        ros::Subscriber sub = nh.subscribe("scan", 1000, scanCallback);
        ros::spin();
    }
