#include <iostream>
#include <vector>
#include <algorithm> //std::vector reverse function
#include <queue> //priority queue
#include <cmath>
#include <unordered_map>

// Used datatypes:
//
//  std::pair<T1,T2>
//      Object containing 2 elements of type T1 (.first) and T2 (.second)
//      A pair may also be represented as [first, second]
//
//  std::priority_queue<T,std::vector<T>,Comparator>
//      List of objects of type T oredered by Comparator.
//      Comparator can be a struct or class defining the comparison operator for the T type
//
//  std::unordered_map<T1,T2>
//      Hashmap container, it allows us to identify an object of type T2 by another type T1
//      for instance, T1 may be integer or string.

using namespace std;

// Class representing a node in the A* search
class Node {
public:
    int x, y; // Node coordinates in the gridmap
    double g, h; // Costs: g (distance from start), h (heuristic)
    Node* parent; // Parent node for path reconstruction
    
    // Construction of the node
    Node(int x, int y, double g, double h, Node* parent = nullptr)
        : x(x), y(y), g(g), h(h), parent(parent) {}
    
    // A* evaluation function (just return g + h)
    double f() const { return g + h; } 
};

// Comparator for the priority queue (min-heap based on f value)
//  Note: priority queue automatically order object bas
struct CompareNodes {
    bool operator()(const Node* a, const Node* b) const {
        return a->f() > b->f();
    }
};

// Function to get valid neighbors of a cell in the grid
//  Neighbors are returned as a list of pairs (i.e., cells in the gridmap)
vector<pair<int, int>> get_neighbors(int x, int y, int rows, int cols) {
    vector<pair<int, int>> neighbors;

    // 4-connected grid
    vector<pair<int, int>> deltas = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};
    
    //for each pair in deltas
    for (auto [dx, dy] : deltas) {
        int nx = x + dx; 
        int ny = y + dy;
        if (nx >= 0 && ny >= 0 && nx < rows && ny < cols)
            //append to the end of the vector
            neighbors.emplace_back(nx, ny);
    }
    return neighbors;
}

// Heuristic function
double heuristic(int x1, int y1, int x2, int y2) {
    //Manhattan distance
    return abs(x1 - x2) + abs(y1 - y2);
}

// A* algorithm implementation
vector<pair<int, int>> a_star(vector<vector<int>>& grid, pair<int, int> start, pair<int, int> goal) {

    //-- Initialization phase
    int rows = grid.size(), cols = grid[0].size();
    priority_queue<Node*, vector<Node*>, CompareNodes> open_set; //frontier
    
    
    // Create the start node
    Node* start_node = new Node(start.first, start.second, 0, heuristic(start.first, start.second, goal.first, goal.second));
    // Put start node into the frontier
    open_set.push(start_node);

    // We also create a map to associate an ID to each node
    unordered_map<int, Node*> all_nodes;
    // Here we use position of the node to compute a unique indentifier.
    //  This is a trick to rapidly retrieve the Node structure from its pose
    all_nodes[start.first * cols + start.second] = start_node;
    //--
    
    // While frontier (queue) is not empty
    while (!open_set.empty()) {
        // Get best node
        Node* current = open_set.top(); 
        // Remove it from the queue
        open_set.pop();
        
        // If the current node is the goal, reconstruct the path
        if (current->x == goal.first && current->y == goal.second) {
            vector<pair<int, int>> path;
            while (current) {
                path.emplace_back(current->x, current->y);
                current = current->parent;
            }
            reverse(path.begin(), path.end());
            return path;
        }
        
        // Expand the neighbors of the current node
        vector<pair<int, int>> neighbors = get_neighbors(current->x, current->y, rows, cols);
        for (auto [nx, ny] : neighbors) {

            // Skip obstacle cells
            if (grid[nx][ny] == 1) continue; 

            double new_g = current->g + 1;
            //compute index of the expanded node
            int index = nx * cols + ny;
            
            // If the neighbor has not been visited yet or has a lower cost
            if (!all_nodes.count(index) || new_g < all_nodes[index]->g) {
                // Add the node to the frontier (and to the map)
                Node* new_node = new Node(nx, ny, new_g, heuristic(nx, ny, goal.first, goal.second), current);
                open_set.push(new_node);
                all_nodes[index] = new_node;
            }
        }
    }
    return {}; // No path found
}

int main() {
    // Define the example grid (0: free, 1: obstacle)
    //  x -> rows
    //  y -> columns
    vector<vector<int>> grid = {
        {0, 0, 0, 0, 0, 0, 1, 0, 1, 0},
        {0, 1, 1, 1, 0, 0, 0, 0, 1, 0},
        {0, 1, 1, 1, 0, 0, 1, 0, 0, 0},
        {0, 1, 1, 1, 0, 0, 1, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 1, 0, 0, 0}
    };
    
    pair<int, int> start = {0, 0}; // Start point
    pair<int, int> goal = {4, 9}; // Destination
    
    // Run A*
    vector<pair<int, int>> path = a_star(grid, start, goal);
    
    // Output the found path
    if (!path.empty()) {
        cout << "Path found:\n";
        for (auto [x, y] : path)
            cout << "(" << x << ", " << y << ") -> ";
        cout << "Goal\n";
    } else {
        cout << "No path found!\n";
    }
    
    return 0;
}
