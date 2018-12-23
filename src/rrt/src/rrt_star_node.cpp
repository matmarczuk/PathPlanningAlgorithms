#include <ros/ros.h>
#include <rrt/rrt_star.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rrt_star");
    ros::NodeHandle nh;
    ros::Rate loop_rate(0.25);

    Point pt1;  pt1.x = 30; pt1.y = 40;
    Point pt2;  pt2.x = 60; pt2.y = 40;
    Point pt3;  pt3.x = 30; pt3.y = 70;
    Point pt4;  pt4.x = 60; pt4.y = 70;

    Obstacle obstacle1(pt1,pt2,pt3,pt4);

    std::vector<Obstacle> obst_vec;
    obst_vec.push_back(obstacle1);

    RRT_star_tree RRT_star(Node(0,50),Node(100,50),obst_vec,3000,5.0,7.0);
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
