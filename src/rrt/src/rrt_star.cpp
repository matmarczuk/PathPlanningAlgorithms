#include <rrt/rrt_star.h>

RRT_star_tree::RRT_star_tree(Node start_node,Node goal_node,std::vector<Obstacle> obst_vec,int nodes_num,float build_ran,float neigh_range)
    : RRT_tree(start_node,goal_node,obst_vec,nodes_num,build_ran), neighborhood_range(neigh_range)
{

}
void RRT_star_tree::show()
{
    ROS_INFO("start_node x: %f y: %f range: %f",start.pos.x,start.pos.y,neighborhood_range);
}

void RRT_star_tree::build()
{
    clear();
    Node new_node;

    for(int i = 0;i<nodes_number;i++)
    {
        geometry_msgs::Point rand_point = get_rand_point(0,100);

        Node *tmp;
        Node *neighbour;


        //find the closest
        tmp = find_closest_node(rand_point);
        new_node.pos = step_from_to(tmp->pos,rand_point);
        neighbour = find_neighbor(new_node.pos);

        if(tmp != neighbour && neighbour != NULL)
            tmp = neighbour;

        new_node.parent = tmp;
        new_node.cost = calc_cost(*tmp,new_node.pos);

        if(!check_colision(new_node))
        {
            i--; //have a one more try
            continue;
        }

        nodes_vec.push_back(new_node);
        draw_edge(*tmp,new_node,edge_list);

        rewire();

    }

    rrt_pub.publish(edge_list);

}
float RRT_star_tree::calc_cost(Node parent_node, Point rand_point)
{
    float cost = 0;

    cost = dist(parent_node.pos,rand_point); //todo more efficient cost function

    return parent_node.cost + cost;
}
Node * RRT_star_tree::find_neighbor(Point rand_point)
{
    Node *tmp = NULL;
    float min_cost = 140;
    for(int i = 0;i<nodes_vec.size();i++)
    {
        if(dist(nodes_vec[i].pos,rand_point) > neighborhood_range)
            continue;

        if(float cost = calc_cost(nodes_vec[i],rand_point) < min_cost)
        {
            tmp = &nodes_vec[i];
            min_cost = cost;
        }
    }

    return tmp;
}

void RRT_star_tree::rewire()
{
    Node new_node = nodes_vec.back(); //get latest added node

    std::vector<Node *> node_ptr;

    for(int i = 0;i<nodes_vec.size() - 1;i++)
    {
        if(dist(nodes_vec[i].pos,new_node.pos) > neighborhood_range)
            continue;

        node_ptr.push_back(&nodes_vec[i]);
    }

    for(int i = 0;i<node_ptr.size();i++)
    {
        ROS_INFO("calc cost %f current cost %f",calc_cost(*(node_ptr[i]),new_node.pos),node_ptr[i]->cost);
        if(calc_cost(*(node_ptr[i]),new_node.pos) < node_ptr[i]->cost)
        {
            //rewire
            ROS_INFO("Rewire");
            node_ptr[i]->parent = &new_node;

            edge_list.points.erase(edge_list.points.begin() + 2*(node_ptr[i]->inter_num) ); //erase old edge
            edge_list.points.erase(edge_list.points.begin() + 2*(node_ptr[i]->inter_num)+1 ); //erase old edge

            draw_edge(*node_ptr[i],new_node,edge_list);

        }
    }



}
