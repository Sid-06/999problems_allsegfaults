#pragma once
#include "srtp.hpp"
#include<unordered_map>
#include<vector>
#include<iostream>
#include<utility>
#include<queue>
#include<set>
#include "../Phase-2/K_shortest_paths.h"
#include "knn.h"


#include "nlohmann/json.hpp"

using json = nlohmann::json;
///////////////////////////////////////////////////////////////
struct Node{
    int id;
    double lat,lon;
    std::set<std::string> poi;
    Node(int i, double la, double lo, const json& pois) 
        : id(i), lat(la), lon(lo) {
        for (const auto& p : pois) {
            poi.insert(p);
        }
}
};
struct Edge{
    int id;
    int u,v;
    double length;
    double avg_time;
    std::vector<double>speed_profile;
    bool oneway;
    std::string road_type;
    Edge(){}
    Edge(int i, int u_, int v_, double len, double time, 
         const json& speed, bool one, const std::string& type) 
        : id(i), u(u_), v(v_), length(len), avg_time(time), 
          oneway(one), road_type(type) {
        for (const auto& s : speed) {
            speed_profile.push_back(s);
        }
    }
};

class Graph
{
private:
int N;
std::vector<Node*> Nodes;
std::vector<std::vector<Edge*>> adj;
std::vector<std::vector<Edge*>> adj_r;
public:
  Graph(const json& graph_json);
  json process_query(const json& query);
  json handleRemoveEdge(const json& query);
  json handleModifyEdge(const json& query);
  std::vector<int> handleShortesPath(SRTP sp,bool &possible,double & mtbd);

  std::vector<int> Djikstra(SRTP sp,bool &possible,double & mtbd);
  std::vector<int> handleKnn(KNN knn);
  std::vector<std::pair<std::vector<int>,double>> k_shortest_paths_heustirics(KSP_H ksp);
  std::vector<std::pair<std::vector<int>,double>> k_shortest_paths_exact(KSP_E ksp);
  size_t getNodeCount() const { return Nodes.size(); }
    
    void printSampleData() const {
        // Print first 5 nodes
        std::cout << "\nFirst 5 nodes:\n";
        for(int i = 0; i < std::min(5, (int)Nodes.size()); i++) {
            std::cout << "Node " << Nodes[i]->id << ": lat=" << Nodes[i]->lat 
                     << ", lon=" << Nodes[i]->lon << "\n";
        }
        
        // Print first 5 edges of first node
        std::cout << "\nFirst 5 edges of node 0:\n";
        if(!adj.empty() && !adj[0].empty()) {
            for(int i = 0; i < std::min(5, (int)adj[0].size()); i++) {
                std::cout << "Edge " << adj[0][i]->id << ": " 
                         << adj[0][i]->u << " -> " << adj[0][i]->v 
                         << " (length: " << adj[0][i]->length << ")\n";
            }
        }
    }
};