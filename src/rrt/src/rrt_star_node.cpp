#include <ros/ros.h>
#include <rrt/rrt_star.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rrt_star");
    ros::NodeHandle nh;

    RRT_star_tree RRT_star(Node(0,0),Node(100,100),100,3.5,1.2);

    RRT_star.show();

}
