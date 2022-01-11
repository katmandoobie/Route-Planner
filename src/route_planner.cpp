#include "route_planner.h"
#include <algorithm>
#include <iostream>
using namespace std;

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    
    //use input data to set start and end node
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node   = &m_Model.FindClosestNode(end_x, end_y);   
}



//CalculateHValue by finding distance from node to end node
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}



//adds neighbors to current node
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    //find neighbors next to current node
    current_node->FindNeighbors();
    //loop through each neighbor
    for(auto n : current_node->neighbors)
    {
        //set h_value by using calcHVal function
        //set g_value by incrementing current g_value with distance to current node from neighbor
        //set parent to the current node
        //set neighbor to visited
        //push back neighbor into open list
        n->h_value = this->CalculateHValue(n);
        n->g_value = current_node->g_value + n->distance(*current_node);
        n->parent = current_node;
        n->visited = true;
        open_list.push_back(n);
    }
}



//find the next node and return it
RouteModel::Node *RoutePlanner::NextNode() {
    //use <algorithm> sort function to sort open_list from smallest (h+g) to largest
    sort(open_list.begin(),open_list.end(), [](const RouteModel::Node *a, const RouteModel::Node *b){return (a->h_value+a->g_value) > (b->h_value+b->g_value);});
    //set next node to last node in open list
    auto nxt = open_list.back();
    //remove that last node
    open_list.pop_back();
    return nxt;
}



//construct the final path when the end point has been reach and return the path
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    //while their is a node
    while(current_node)
    {
        //if there is a parent node then find distance from current node to parent node
        if(current_node->parent) distance = distance + current_node->distance(*(current_node->parent));
        //insert the node into the beginning of the path
        path_found.insert(path_found.begin(), *current_node);
        //set current node to the next parent node
        current_node = current_node->parent;
    }


    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}



//Run A* 
void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    start_node->visited = true;//start node is set to visited
    open_list.push_back(start_node);//push the start node in open list 

    while(open_list.size() > 0) //loop while their is still nodes in open_list
    {
        current_node = NextNode();//get the next node 
        //if the current node is our end point then set the 'path' to the final path and break
        if((current_node->x == end_node->x) && (current_node->y == end_node->y))
        {
            m_Model.path = ConstructFinalPath(current_node);
            break;
        }
        //if not the end then find neighboring nodes
        AddNeighbors(current_node);
    }
}