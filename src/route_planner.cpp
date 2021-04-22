#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    RoutePlanner::start_node = &m_Model.FindClosestNode(start_x, start_y);
    RoutePlanner::end_node = &m_Model.FindClosestNode(end_x, end_y);
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    float result;
    RouteModel::Node end_node = *RoutePlanner::end_node;
    result = node->distance(end_node);
    return result;
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (RouteModel::Node *neighbor : current_node->neighbors) {
        neighbor->parent = current_node;
        neighbor->g_value = current_node->g_value + neighbor->distance(*current_node);
        neighbor->h_value = CalculateHValue(neighbor);
        neighbor->visited = true;
        RoutePlanner::open_list.push_back(neighbor);
    }
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.
bool Compare(RouteModel::Node *first, RouteModel::Node *last) {
    std::cout << first << "\n";
    std::cout << last << "\n";
    int f1 = first->g_value + first->h_value;
    int f2 = last->g_value + last->g_value;
    if (f1 > f2) {
        std::cout << "True " << first->x << ","  << first->y << "\n";
        return true;
    } else {
        std::cout << "false " << last->x << ","  << last->y << "\n";
        return false;
    }
}

RouteModel::Node *RoutePlanner::NextNode() {
    sort(RoutePlanner::open_list.begin(), RoutePlanner::open_list.end(), Compare);
    RouteModel::Node* next_node = open_list.back();
    open_list.pop_back();
    return next_node;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    path_found.insert(path_found.begin(), *current_node);
    RouteModel::Node last_node;

    // TODO: Implement your solution here.
    while (current_node->parent) {
        RouteModel::Node* parent = current_node->parent;        
        distance = (distance + parent->distance(*current_node));
        path_found.insert(path_found.begin(), *parent); 
        current_node = parent;
    }

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    start_node->visited = true;
    current_node = start_node;
    open_list.push_back(current_node);

    // TODO: Implement your solution here.
    while (current_node != end_node) {
        AddNeighbors(current_node);
        current_node = NextNode();
    }
    m_Model.path = ConstructFinalPath(current_node);
}