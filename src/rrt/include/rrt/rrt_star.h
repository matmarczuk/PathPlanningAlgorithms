#ifndef RRT_STAR_H
#define RRT_STAR_H

#include <rrt/rrt.h>


class RRT_star_tree : public RRT_tree
{
    float neighborhood_range;
public:
    RRT_star_tree(Node start_node,Node goal_node,std::vector<Obstacle> obst_vec,int nodes_num,float build_ran,float neigh_range);
    void show();

    float calc_cost(Node parent_node, Point rand_point);

    void build();
    Node * find_neighbor(Point rand_point);
    void rewire();



};

#endif // RRT_STAR_H
