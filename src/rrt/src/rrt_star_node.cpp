#include <ros/ros.h>
#include <rrt/rrt_star.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rrt_star");
    ros::NodeHandle nh;
    ros::Rate loop_rate(0.25);

    RRT_star_tree RRT_star(Node(0,0),Node(100,100),1000,5,10.0);
    RRT_star.init();

    while(ros::ok())
    {
        RRT_star.build();
        RRT_star.find_goal();
        RRT_star.find_path();

        loop_rate.sleep();
        ros::spinOnce();
    }

}
