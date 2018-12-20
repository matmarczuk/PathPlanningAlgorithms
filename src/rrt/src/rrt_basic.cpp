#include "ros/ros.h"
#include <rrt/rrt.h>

#include "std_msgs/String.h"
#include <ctime>
#include <math.h>
#include "sensor_msgs/PointCloud.h"
#include <visualization_msgs/Marker.h>

#define START_NUMBER 0
#define RANGE 100
#define NODES_NUMBER 3000
#define EPSILON 5.0
#define AIM

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rrt_basic");

    Node start;
    start.pos.x = 0;
    start.pos.y = 0;
    start.parent = NULL;

    Node goal;
    goal.pos.x = 100;
    goal.pos.y = 100;

    RRT_tree RRT(start,goal);
    RRT.init();

    ros::Rate loop_rate(0.25);

    while(ros::ok())
    {
        RRT.build();
        RRT.find_goal();
        RRT.find_path();

        loop_rate.sleep();
        ros::spinOnce();
    }



    return 0;
}
