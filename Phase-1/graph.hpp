#pragma once
#include "srtp.hpp"
#include<unordered_map>
#include<vector>
#include<iostream>
#include<utility>
#include<queue>
#include<set>
#include "../Phase-2/K_shortest_paths.h"
#include "../Phase-1/knn.h"


#include <nlohmann/json.hpp>

using json = nlohmann::json;
///////////////////////////////////////////////////////////////
struct Node{
    int id;
    double lat,lon;
    std::set<std::string> poi;
};
struct Edge{
    int id;
    int u,v;
    double length;
    double avg_time;
    std::vector<double>speed_profile;
    bool oneway;
    std::string road_type;
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
  json handleRemoveEdge(const json& query);
  json handleModifyEdge(const json& query);
  std::vector<int> handleShortesPath(SRTP sp,bool &possible,double & mtbd);
  std::vector<int> Djikstra(SRTP sp,bool &possible,double & mtbd);
  std::vector<int> handleKnn(KNN knn);
  std::vector<std::pair<std::vector<int>,double>> k_shortest_paths_heustirics(KSP_H ksp);
  std::vector<std::pair<std::vector<int>,double>> k_shortest_paths_exact(KSP_E ksp);
};