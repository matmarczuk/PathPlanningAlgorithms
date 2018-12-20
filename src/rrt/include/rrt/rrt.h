#ifndef RRT_BASIC_H
#define RRT_BASIC_H
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <math.h>
#include <ros/ros.h>

struct Node
{
   geometry_msgs::Point pos;
   Node *parent;
   float cost;

   Node(int x, int y);
   Node();
};

class RRT_tree
{
protected:
    Node start;
    Node goal;
    int nodes_number;
    float build_range;
    ros::Publisher rrt_pub;

    std::vector<Node> nodes_vec;
    visualization_msgs::Marker edge_list;
    visualization_msgs::Marker path_list;

public:
    RRT_tree(Node start_node,Node goal_node,int nodes_num,float build_ran);

    void init();
    bool pts_equal(geometry_msgs::Point p1,geometry_msgs::Point p2);
    geometry_msgs::Point step_from_to(geometry_msgs::Point p1, geometry_msgs::Point p2);
    geometry_msgs::Point get_rand_point(int down_num, int range);
    void draw_edge(geometry_msgs::Point pt1,geometry_msgs::Point pt2,visualization_msgs::Marker& marker);
    float dist(geometry_msgs::Point p1, geometry_msgs::Point p2);

    Node* find_closest_node(geometry_msgs::Point rand_point);
    bool check_colision(Node new_node);

    void build();
    void clear();
    void find_goal();
    void find_path();

};


#endif // RRT_BASIC_H
