#include "../common/graph.hpp"   
#include "knn.h"              
#include <vector>                // for std::vector
#include <cmath>                 // distance computations
#include <algorithm>             // sort distances
#include <queue>                 // priority queue


double INF = 1e5 + 1;



std::vector<int> Graph::handleKnn(KNN knn){
    std::vector<int> final_ids;
    double lat = knn.lat;
    double lon = knn.lon;
    std::string poi = knn.poi;
    int k = knn.k;

    if(knn.metric == "shortest_path"){
        double a  = 1e5 , lat_n , lon_n;
        int id = 1;
        for(int i=0;i<this->N;i++){
            double dx = lat-this->Nodes[i]->lat;
            double dy = lon-this->Nodes[i]->lon;
            double d =  std::sqrt(dx*dx + dy*dy);
            a = (a<d)?a:d;
            if(a > d){
                a = d;
                lat_n = this->Nodes[i]->lat;
                lon_n = this->Nodes[i]->lon;
                id = this->Nodes[i]->id;
            }
        }

        std::vector<std::pair<double,int>> distances;
        for(int i=0;i<this->N;i++){
            distances.push_back({INF,i+1});
        }
        std::vector<bool> visited(N,false);
        distances[id-1].first = 0;
        std::priority_queue< std::pair<double,int>, std::vector<std::pair<double,int>>, std::greater<std::pair<double,int>>> q;
        q.push({0,id-1});
        visited[id-1] = true;
        while(!q.empty() && k > 0){
            int o = q.top().second;
            q.pop();
            if(visited[o]) continue;
            distances[o].first = true;
            for( auto node : this->adj[o]){
                int v = node->v;
                int w = node->length;
                if(distances[o].first + w < distances[v-1].first){
                    distances[v-1].first = distances[o].first + w;
                    q.push({distances[v-1].first,v-1});
                }
            }
        }
        std::sort(distances.begin(),distances.end());
        for(int i=0;i<k;i++){
            final_ids.push_back(distances[i].second);
        }
    }
    else if(knn.metric == "euclidean"){
        std::vector<std::pair<double, int>> distances;
        for (int i = 0;i<this->N;i++) {
            double dx = lat-this->Nodes[i]->lat;
            double dy = lon-this->Nodes[i]->lon;
            double d = std::sqrt(dx*dx + dy*dy);
            distances.push_back({d, this->Nodes[i]->id});
        }
        std::sort(distances.begin(),distances.end());
        int a = (k >= distances.size())?distances.size():k;
        for(int i = 0;i<a;i++){
            final_ids.push_back(distances[i].second);
        }
    }

    return final_ids;
}

