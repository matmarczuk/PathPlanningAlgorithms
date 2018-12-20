#include "ros/ros.h"
#include <rrt/rrt.h>

#include "std_msgs/String.h"
#include <ctime>
#include <math.h>
#include "sensor_msgs/PointCloud.h"
#include <visualization_msgs/Marker.h>

#define RANGE 100
#define NODES_NUMBER 1000
#define EPSILON 5.0

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rrt_basic");

    Node start(0,0);
    Node goal(100,100);

    RRT_tree RRT(start,goal,NODES_NUMBER,EPSILON);
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
