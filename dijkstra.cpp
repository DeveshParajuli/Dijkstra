#include <iostream>
#include <vector>
#include <queue>
#include <climits>
#include <stack>

using namespace std;

typedef pair<int, char> pic; // Pair representing (distance, node label)

class Graph {
    int V;
    vector<vector<pic>> adjList; // Use pairs of (vertex, weight)

public:
    Graph(int vertices) : V(vertices) {
        adjList.resize(V);
    }

    void addEdge(char u, char v, int weight) {
        adjList[u - 'A'].emplace_back(v, weight);
        adjList[v - 'A'].emplace_back(u, weight); // Assuming an undirected graph
    }

    void display(char src, char dest) {
        priority_queue<pic, vector<pic>, greater<pic>> pq; // Use pairs for the priority queue
        vector<int> dist(V, INT_MAX);
        vector<char> parent(V, ' ');

        pq.push({0, src - 'A'});
        dist[src - 'A'] = 0;

        while (!pq.empty()) {
            int u = pq.top().second;
            pq.pop();

            // Visualize the current state of the algorithm
            displayStep(u, dist, parent);

            for (const auto& neighbor : adjList[u]) {
                int v = neighbor.first - 'A';
                int weight = neighbor.second;

                if (dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight;
                    parent[v] = char(u + 'A');
                    pq.push({dist[v], v});
                }
            }
        }

        // Display the shortest distance from source to destination
        cout << "\nShortest distance from node " << src << " to node " << dest << ": " << dist[dest - 'A'] << endl;

        // Display the shortest path
        displayPath(src, dest, parent);
    }

    void displayStep(int currentVertex, const vector<int>& distances, const vector<char>& parent) {
        cout << "\nStep: Current Vertex = " << char(currentVertex + 'A') << endl;
        cout << "Distances: | ";
        for (int i = 0; i < V; ++i) {
            if (distances[i] == INT_MAX) {
                cout << "INF ";
            } else {
                cout << distances[i] << " ";
            }
            cout<< " | ";
        }
        cout<<endl;
        cout << "\nPaths: ";
        for (int i = 0; i < V; ++i) {
            if (parent[i] != ' ') {
                stack<char> path;
                char current = char(i + 'A');
                while (current != ' ') {
                    path.push(current);
                    current = parent[current - 'A'];
                }
                while (!path.empty()) {
                    cout << path.top();
                    path.pop();
                    if (!path.empty()) {
                        cout << " -> ";
                    }
                }
            }
            cout << " | ";
        }
        cout << "\n"<<endl;
    }

    void displayPath(char src, char dest, const vector<char>& parent) {
        stack<char> path;
        char current = dest;

        while (current != src) {
            path.push(current);
            current = parent[current - 'A'];
        }

        path.push(src);

        cout << "\nShortest path: ";
        while (!path.empty()) {
            cout << path.top();
            path.pop();
            if (!path.empty()) {
                cout << " -> ";
            }
        }
        cout << endl;
    }
};

int main() {
    // Example usage
    int vertices = 6;
    Graph g(vertices);

    // Adding edges to the graph
    g.addEdge('A', 'B', 4);
    g.addEdge('A', 'C', 2);
    g.addEdge('B', 'C', 5);
    g.addEdge('B', 'D', 10);
    g.addEdge('C', 'D', 3);
    g.addEdge('D', 'E', 7);
    g.addEdge('E', 'F', 2);

    // Get user input for source and destination nodes
    char source, destination;
    cout << "Enter source node: ";
    cin >> source;
    cout << "Enter destination node: ";
    cin >> destination;

    // Check for valid input
    if (source < 'A' || source >= char('A' + vertices) || destination < 'A' || destination >= char('A' + vertices)) {
        cerr << "Invalid source or destination node. Exiting program." << endl;
        return 1;
    }

    // Display Dijkstra's algorithm
    g.display(source, destination);

    return 0;
}
