#include<unordered_map>
#include<vector>
#include<iostream>
#include<utility>
#include<queue>
#include<set>


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
struct SRTP{
    int id;
    int source;
    int target;
    std::string mode;
    std::vector<int> forbidden_nodes;
    std::vector<int> forbidden_road_types;
};
// struct KNN{
//     int id;
//     std::string poi;
//     double lat,lon;
//     int k;
//     std::string metric;
// };
class Graph
{
private:
int N;
std::vector<Node*> Nodes;
std::vector<std::vector<Edge*>> adj;
public:
  Graph(const json& graph_json);
  json handleRemoveEdge(const json& query);
  json handleModifyEdge(const json& query);
  std::vector<int> handleShortesPath(SRTP sp,bool &possible,double & mtbd);
  std::vector<int> handleKnn(KNN knn);
};