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

    Point pt1;  pt1.x = 30; pt1.y = 30;
    Point pt2;  pt2.x = 60; pt2.y = 30;
    Point pt3;  pt3.x = 30; pt3.y = 60;
    Point pt4;  pt4.x = 60; pt4.y = 60;

    Obstacle obstacle1(pt1,pt2,pt3,pt4);

    std::vector<Obstacle> obst_vec;
    obst_vec.push_back(obstacle1);

    RRT_tree RRT(start,goal,obst_vec,NODES_NUMBER,EPSILON);
    RRT.init();

    ros::Rate loop_rate(0.5);

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
