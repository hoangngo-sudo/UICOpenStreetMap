#include "application.h"

#include <iostream>
#include <limits>
#include <map>
#include <queue> // priority_queue
#include <set>
#include <stack>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "dist.h"
#include "graph.h"
#include "json.hpp"

using namespace std;
using json = nlohmann::json;

class prioritize {
 public:
  bool operator()(const pair<long long, double>& p1,
                  const pair<long long, double>& p2) const {
    return p1.second > p2.second;
  }
};

double INF = numeric_limits<double>::max();

void buildGraph(istream &input, graph<long long, double> &G, vector<BuildingInfo> &buildings,
                unordered_map<long long, Coordinates> &coords) {
  json j;   // Parse the JSON input stream
  try {
    input >> j;
  } catch (json::parse_error& error) {
    cerr << "Error parsing JSON: " << error.what() << endl;
    return;
  }

  // Process buildings
  if (j.contains("buildings") && j["buildings"].is_array()) {
    for (const auto& building_json : j["buildings"]) {
      // Extract building information
      long long id = building_json["id"];
      double lat = building_json["lat"];
      double lon = building_json["lon"];
      string name = building_json["name"];
      string abbr = building_json["abbr"];

      Coordinates location(lat, lon);

      // Add vertex to the graph
      G.addVertex(id);

      // Add BuildingInfo to the buildings vector
      buildings.emplace_back(id, location, name, abbr);
    }
  }

  // Process waypoints (non-building vertices)
  if (j.contains("waypoints") && j["waypoints"].is_array()) {
    for (const auto& waypoint_json : j["waypoints"]) {
      // Extract waypoint information
      long long id = waypoint_json["id"];
      double lat = waypoint_json["lat"];
      double lon = waypoint_json["lon"];

      Coordinates location(lat, lon); //

      // Add vertex to the graph
      G.addVertex(id);

      // Add waypoint coordinates to the coords map
      coords[id] = location;
    }
  }

    // Process footways (edges between waypoints)
  if (j.contains("footways") && j["footways"].is_array()) {
    for (const auto& footway_json : j["footways"]) {
      if (footway_json.is_array() && footway_json.size() >= 2) {
            // Process all consecutive pairs of nodes in the footway
        for (size_t i = 0; i < footway_json.size() - 1; i++) {
          long long id1 = footway_json[i];
          long long id2 = footway_json[i + 1];

          // Check if both waypoint IDs exist in the coords map
          if (coords.count(id1) && coords.count(id2)) {
              Coordinates c1 = coords[id1];
              Coordinates c2 = coords[id2];

              // Calculate the distance between waypoints
              double distance = distBetween2Points(c1, c2);

              // Add undirected edge to the graph
              G.addEdge(id1, id2, distance);
              G.addEdge(id2, id1, distance);
          }
        }
      }
    }
  }


  // Link buildings to nearby waypoints
  const double MAX_DIST_MILES = 0.036;

  for (const auto& building : buildings) {
    Coordinates b_coords = building.location;

    for (const auto& pair : coords) {
      long long waypoint_id = pair.first;
      Coordinates w_coords = pair.second;

      // Calculate the distance between building and waypoint
      double distance = distBetween2Points(b_coords, w_coords);

      // If distance is within the threshold, add an undirected edge
      if (distance <= MAX_DIST_MILES) {
        G.addEdge(building.id, waypoint_id, distance);
        G.addEdge(waypoint_id, building.id, distance);
      }
    }
  }
}

BuildingInfo getBuildingInfo(const vector<BuildingInfo> &buildings,
                             const string &query) {
  for (const BuildingInfo &building : buildings) {
    if (building.abbr == query) {
      return building;
    } else if (building.name.find(query) != string::npos) {
      return building;
    }
  }
  BuildingInfo fail;
  fail.id = -1;
  return fail;
}

BuildingInfo getClosestBuilding(const vector<BuildingInfo> &buildings,
                                Coordinates c) {
  double minDestDist = INF;
  BuildingInfo ret = buildings.at(0);
  for (const BuildingInfo &building : buildings) {
    double dist = distBetween2Points(building.location, c);
    if (dist < minDestDist) {
      minDestDist = dist;
      ret = building;
    }
  }
  return ret;
}

vector<long long> dijkstra(const graph<long long, double> &G, long long start,
                           long long target,
                           const set<long long> &ignoreNodes) {
  if (start == target) {
    return {start};
  }

  // Declare a C++ STL priority queue to store the worklist
  priority_queue<pair<long long, double>, vector<pair<long long, double>>, prioritize> worklist;
  
  // Declare a C++ STL map to store the shortest distance found so far from the start vertex to every other vertex.
  map<long long, double> dist;

  // Declare a C++ STL map to store the predecessor of each vertex in the shortest path found so far. 
  map<long long, long long> pred;
  
  // Declare a C++ STL set to store the visited vertices
  set<long long> visited;

  for (const auto &vertex : G.getVertices()) { // for each vertex in the graph
    dist[vertex] = INF; // initialize the distance to infinity
    pred[vertex] = -1; // initialize the predecessor to -1 for n/a 
  }
  dist[start] = 0;
  // Push the starting node onto the priority queue with distance 0
  worklist.push({start, 0});

  while (!worklist.empty()) {
    pair<long long, double> curr = worklist.top();
    worklist.pop();
    long long currVertex = curr.first;
    double currDist = curr.second;

    if (visited.find(currVertex) != visited.end() || currDist > dist[currVertex]) {
      continue; 
    }

    visited.insert(currVertex);

    if (currVertex == target) {
      break;
    }

    set<long long> neighbors = G.neighbors(currVertex);
    for (long long neighbor : neighbors) {
      if (ignoreNodes.find(neighbor) != ignoreNodes.end() && 
          neighbor != start && neighbor != target) {
        continue; 
      }

      double weight;
      G.getWeight(currVertex, neighbor, weight);
      double newDist = weight + currDist;

      if (newDist < dist[neighbor]) {
        dist[neighbor] = newDist;
        pred[neighbor] = currVertex;
        worklist.push({neighbor, newDist});
      }
    }
  }

  if (pred[target] == -1 && start != target) {
    return {};
  }

  vector<long long> path;
  for (long long v = target; v != -1; v = pred[v]) {
    path.push_back(v);
  }
  reverse(path.begin(), path.end());
  return path;
}

double pathLength(const graph<long long, double> &G,
                  const vector<long long> &path) {
  double length = 0.0;
  double weight;
  for (size_t i = 0; i + 1 < path.size(); i++) {
    bool res = G.getWeight(path.at(i), path.at(i + 1), weight);
    if (!res) {
      return -1;
    }
    length += weight;
  }
  return length;
}

void outputPath(const vector<long long> &path) {
  for (size_t i = 0; i < path.size(); i++) {
    cout << path.at(i);
    if (i != path.size() - 1) {
      cout << "->";
    }
  }
  cout << endl;
}

// Honestly this function is just a holdover from an old version of the project
void application(const vector<BuildingInfo> &buildings,
                 const graph<long long, double> &G) {
  string person1Building, person2Building;

  set<long long> buildingNodes;
  for (const auto &building : buildings) {
    buildingNodes.insert(building.id);
  }

  cout << endl;
  cout << "Enter person 1's building (partial name or abbreviation), or #> ";
  getline(cin, person1Building);

  while (person1Building != "#") {
    cout << "Enter person 2's building (partial name or abbreviation)> ";
    getline(cin, person2Building);

    // Look up buildings by query
    BuildingInfo p1 = getBuildingInfo(buildings, person1Building);
    BuildingInfo p2 = getBuildingInfo(buildings, person2Building);
    Coordinates P1Coords, P2Coords;
    string P1Name, P2Name;

    if (p1.id == -1) {
      cout << "Person 1's building not found" << endl;
    } else if (p2.id == -1) {
      cout << "Person 2's building not found" << endl;
    } else {
      cout << endl;
      cout << "Person 1's point:" << endl;
      cout << " " << p1.name << endl;
      cout << " " << p1.id << endl;
      cout << " (" << p1.location.lat << ", " << p1.location.lon << ")" << endl;
      cout << "Person 2's point:" << endl;
      cout << " " << p2.name << endl;
      cout << " " << p2.id << endl;
      cout << " (" << p2.location.lon << ", " << p2.location.lon << ")" << endl;

      Coordinates centerCoords = centerBetween2Points(p1.location, p2.location);
      BuildingInfo dest = getClosestBuilding(buildings, centerCoords);

      cout << "Destination Building:" << endl;
      cout << " " << dest.name << endl;
      cout << " " << dest.id << endl;
      cout << " (" << dest.location.lat << ", " << dest.location.lon << ")"
           << endl;

      vector<long long> P1Path = dijkstra(G, p1.id, dest.id, buildingNodes);
      vector<long long> P2Path = dijkstra(G, p2.id, dest.id, buildingNodes);

      // This should NEVER happen with how the graph is built
      if (P1Path.empty() || P2Path.empty()) {
        cout << endl;
        cout << "At least one person was unable to reach the destination "
                "building. Is an edge missing?"
             << endl;
        cout << endl;
      } else {
        cout << endl;
        cout << "Person 1's distance to dest: " << pathLength(G, P1Path);
        cout << " miles" << endl;
        cout << "Path: ";
        outputPath(P1Path);
        cout << endl;
        cout << "Person 2's distance to dest: " << pathLength(G, P2Path);
        cout << " miles" << endl;
        cout << "Path: ";
        outputPath(P2Path);
      }
    }

    //
    // another navigation?
    //
    cout << endl;
    cout << "Enter person 1's building (partial name or abbreviation), or #> ";
    getline(cin, person1Building);
  }
}
