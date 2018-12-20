#include <rrt/rrt.h>


Node::Node(int x, int y)
{
    pos.x = x;
    pos.y = y;
    cost = 0;
    parent = NULL;
}
Node::Node()
{

}

RRT_tree::RRT_tree(Node start_node,Node goal_node,int nodes_num,float build_ran)
    : start(start_node),goal(goal_node),nodes_number(nodes_num),build_range(build_ran)
{

}

void RRT_tree::init()
{
    ros::NodeHandle nh;
    rrt_pub = nh.advertise<visualization_msgs::Marker>("rrt_basic",1000);

    srand(time(NULL));

    edge_list.header.frame_id = "my_frame";
    edge_list.action = visualization_msgs::Marker::ADD;
    edge_list.type = visualization_msgs::Marker::LINE_LIST;
    edge_list.scale.x = 0.1;
    edge_list.scale.y = 0.1;
    edge_list.color.g = 1.0f;
    edge_list.color.a = 1.0;
    edge_list.id = 2;


    path_list.header.frame_id = "my_frame";
    path_list.action = visualization_msgs::Marker::ADD;
    path_list.type = visualization_msgs::Marker::LINE_LIST;
    path_list.scale.x = 0.2;
    path_list.scale.y = 0.2;
    path_list.color.g = 0.0f;
    path_list.color.a = 1.0;
    path_list.color.r = 1.0;
    path_list.id = 3;
}

bool RRT_tree::pts_equal(geometry_msgs::Point p1,geometry_msgs::Point p2)
{
    if(p1.x == p2.x && p1. y== p2.y)
        return true;
    else
        return false;
}
geometry_msgs::Point RRT_tree::step_from_to(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
    if(dist(p1, p2) < build_range)
        return p2;
    else
    {
        float theta = atan2(p2.y - p1.y, p2.x - p1.x);
        geometry_msgs::Point approx_point;
        approx_point.x = p1.x + build_range * cos(theta);
        approx_point.y = p1.y + build_range * sin(theta);
        return approx_point;
    }

}
geometry_msgs::Point RRT_tree::get_rand_point(int down_num, int range)
{
    geometry_msgs::Point random;
    random.x =( std::rand() % range ) + down_num;
    random.y =( std::rand() % range ) + down_num;
    return random;
}
void RRT_tree::draw_edge(geometry_msgs::Point pt1,geometry_msgs::Point pt2,visualization_msgs::Marker& marker)
{
    geometry_msgs::Point point;

    point.x = pt1.x;
    point.y = pt1.y;
    marker.points.push_back(point);
    point.x = pt2.x;
    point.y = pt2.y;
    marker.points.push_back(point);
}

float RRT_tree::dist(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}
void RRT_tree::build()
{
    clear();
    for(int i = 0;i<nodes_number;i++)
    {
        geometry_msgs::Point rand_point = get_rand_point(0,100);
        Node new_node;
        Node *tmp;

        //find the closest
        tmp = find_closest_node(rand_point);

        new_node.pos = step_from_to(tmp->pos,rand_point);
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

void RRT_tree::find_goal()
{
    float min_distance = 100;
    int closest_index = 0;

    for(int i = 0;i<nodes_number;i++)
    {
        if(dist(nodes_vec[i].pos,goal.pos)<min_distance)
        {
            min_distance = dist(nodes_vec[i].pos,goal.pos);
            closest_index = i;
        }
    }

    goal = nodes_vec[closest_index];
}
void RRT_tree::find_path()
{
    Node *tmp = &goal;
    while(tmp->parent != NULL)
    {
        draw_edge(tmp->pos,tmp->parent->pos,path_list);
        rrt_pub.publish(path_list);
        tmp = tmp->parent;
    }
}
void RRT_tree::clear()
{
    edge_list.points.clear();
    path_list.points.clear();

    nodes_vec.clear();
    this->nodes_vec.push_back(start);//leave start point
}
Node * RRT_tree::find_closest_node(geometry_msgs::Point rand_point)
{
    Node *tmp = &nodes_vec[0];

    for(int j = 0;j<nodes_vec.size();j++)
    {
        if(dist(nodes_vec[j].pos,rand_point) < dist(tmp->pos,rand_point))
            tmp = &nodes_vec[j];
    }

   return tmp;
}
bool RRT_tree::check_colision(Node new_node)
{
    //todo obstacle list and crossing
    if(new_node.pos.x>30 && new_node.pos.x<50 && new_node.pos.y>30 && new_node.pos.y<60)
    {
        return false;
    }

    return true;
}
