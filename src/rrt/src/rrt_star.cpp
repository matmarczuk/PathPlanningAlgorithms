#include <rrt/rrt_star.h>

RRT_star_tree::RRT_star_tree(Node start_node,Node goal_node,int nodes_num,float build_ran,float neigh_range)
    : RRT_tree(start_node,goal_node,nodes_num,build_ran), neighborhood_range(neigh_range)
{

}
void RRT_star_tree::show()
{
    ROS_INFO("start_node x: %f y: %f range: %f",start.pos.x,start.pos.y,neighborhood_range);
}
