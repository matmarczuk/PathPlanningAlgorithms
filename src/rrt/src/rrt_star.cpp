#include <rrt/rrt_star.h>

RRT_star_tree::RRT_star_tree(Node start_node,Node goal_node,int nodes_num,float build_ran,float neigh_range)
    : RRT_tree(start_node,goal_node,nodes_num,build_ran), neighborhood_range(neigh_range)
{

}
void RRT_star_tree::show()
{
    ROS_INFO("start_node x: %f y: %f range: %f",start.pos.x,start.pos.y,neighborhood_range);
}

void RRT_star_tree::build()
{
    clear();

    for(int i = 0;i<nodes_number;i++)
    {
        geometry_msgs::Point rand_point = get_rand_point(0,100);
        Node new_node;
        Node *tmp;
        Node *neighbour;

        //find the closest
        tmp = find_closest_node(rand_point);
        new_node.pos = step_from_to(tmp->pos,rand_point);
        neighbour = find_neighbor(new_node.pos);

        if(tmp != neighbour)
            tmp = neighbour;

        new_node.parent = tmp;

        if(!check_colision(new_node))
        {
            i--; //have a one more try
            continue;
        }

        nodes_vec.push_back(new_node);

        draw_edge(tmp->pos,new_node.pos,edge_list);
        rrt_pub.publish(edge_list);
    }

}
float RRT_star_tree::calc_cost(Node parent_node, geometry_msgs::Point rand_point)
{
    float cost = 0;

    cost = dist(parent_node.pos,rand_point); //todo more efficient cost function

    return parent_node.cost + cost;
}
Node * RRT_star_tree::find_neighbor(geometry_msgs::Point rand_point)
{
    Node *tmp = &nodes_vec[0];

    for(int i = 0;i<nodes_vec.size();i++)
    {
        if(dist(nodes_vec[i].pos,rand_point) > neighborhood_range)
            continue;

        if(calc_cost(nodes_vec[i],rand_point) < calc_cost(*tmp,rand_point))
            tmp = &nodes_vec[i];
    }

    return tmp;
}
