#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <signal.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/GridCells.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
using costmap_2d::Costmap2DROS;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::INSCRIBED_INFLATED_OBSTACLE;

const NODE_NAME = "listener";


int main(int argc, char** argv)
{
        // Initialize ROS
        ros::init(argc, argv, NODE_NAME);
        ros::NodeHandle n;



        nh = &n;
        node_is_active = true;
        signal(SIGINT, sigIntHandler);
        ROS_INFO("Initializing costmap server node...");
        ROS_INFO("listen for transforms...");
        tf::TransformListener tf(ros::Duration(10));
        ROS_INFO("Create costmap...");
        costmap_ = new costmap_2d::Costmap2DROS(COSTMAP_NAME, tf);
        ROS_INFO("Create update timer...");
        update_timer_ = n.createTimer(ros::Duration(map_update_period_), &timer_cb);

        sleep(2);
        ros::MultiThreadedSpinner mts(4);
        mts.spin();
}