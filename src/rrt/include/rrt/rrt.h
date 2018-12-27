#ifndef RRT_BASIC_H
#define RRT_BASIC_H
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <math.h>
#include <ros/ros.h>

typedef geometry_msgs::Point Point;

struct Obstacle
{
    std::vector<Point> pos_vec;
    Obstacle(Point corn1, Point corn2, Point corn3, Point corn4);
    Obstacle(){}
};
struct Edge
{
    float a,b;

};

struct Node
{
   Point pos;
   Node *parent;
   float cost;
   int inter_num;

   Node(int x, int y);
   Node(){}
};

class RRT_tree
{
protected:
    Node start;
    Node goal;

    std::vector<Obstacle> obst_vec;

    int nodes_number;
    float build_range;
    ros::Publisher rrt_pub;

    std::vector<Node> nodes_vec;
    visualization_msgs::Marker edge_list;
    visualization_msgs::Marker path_list;
    visualization_msgs::Marker obst_list;

public:
    RRT_tree(Node start_node,Node goal_node,std::vector<Obstacle> obst,int nodes_num,float build_ran);

    void init();
    bool pts_equal(Point p1,Point p2);
    Point step_from_to(Point p1, Point p2);
    Point get_rand_point(int down_num, int range);
    void draw_edge(Node n1,Node n2,visualization_msgs::Marker& marker);
    float dist(Point p1, Point p2);
    void draw_obst();

    Node* find_closest_node(Point rand_point);
    bool check_colision(Node new_node);

    void build();
    void clear();
    void find_goal();
    void find_path();

};


#endif // RRT_BASIC_H
