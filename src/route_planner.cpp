#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y)
  :m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = &model.FindClosestNode(start_x,start_y);
    end_node = &model.FindClosestNode(end_x,end_y);
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


// Expand the current node by adding all unvisited neighbors to the open list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (auto v: current_node->neighbors) {
        v->parent = current_node;
        v->h_value = CalculateHValue(v);
        v->g_value = current_node->g_value + current_node->distance(*v);
        v->visited = true;
        open_list.push_back(v);
    }
}


// Compare the F values of two cells
bool Compare(const RouteModel::Node* a, const RouteModel::Node* b) {
    float f1 = a->g_value + a->h_value;
    float f2 = b->g_value + b->h_value;
    return f1 > f2; 
}

// Sort the two-dimensional vector of ints in descending order.
void CellSort(std::vector<RouteModel::Node*>* v) {
    std::sort(v->begin(), v->end(), Compare);
}

// Sort the open list and return the next node.
RouteModel::Node *RoutePlanner::NextNode() {
    CellSort(&open_list);
    auto next_node = open_list.back();
    open_list.pop_back();
    return next_node;
}


// Return the final path found from your A* search.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    while (current_node != start_node) {
        distance += current_node->distance(*current_node->parent);
        path_found.push_back(*current_node);
        current_node = current_node->parent;
    }
    path_found.push_back(*current_node);
    std::reverse(path_found.begin(),path_found.end()); 

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// A* Search algorithm here.
void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    
    start_node->visited = true;
    open_list.push_back(start_node);
  
    while (!open_list.empty()) {
        current_node = NextNode();
        if (current_node == end_node) {
            m_Model.path = ConstructFinalPath(current_node);
            return;
        } else {
            AddNeighbors(current_node);
        }
    }

}