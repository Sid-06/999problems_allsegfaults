#include "graph.hpp"
#include <fstream>
#include <algorithm>
#include <cmath>
#include <chrono>
#include<random>
#define INF 1000000000

void Graph::precompute_alt(){
    if(alt_initialized)return;
    landmarks.clear();
    dist_from_landmark.assign(NUM_LANDMARKS,std::vector<double>(N,INF));
    dist_to_landmark.assign(NUM_LANDMARKS,std::vector<double>(N,INF));
    std::mt19937 rng(42);
    std::uniform_int_distribution<int> uni(0,N-1);
    std::set<int>selected;
    while(landmarks.size()<NUM_LANDMARKS){
        int node_idx = uni(rng);
        if(selected.find(node_idx)==selected.end()){
            landmarks.push_back(node_idx);
            selected.insert(node_idx);
        }
    }
    for(int k=0;k<NUM_LANDMARKS;++k){
        int L = landmarks[k];   
        {
            std::priority_queue<std::pair<double,int>,
            std::vector<std::pair<double,int>>,
            std::greater<std::pair<double, int>>> pq;

            dist_from_landmark[k][L]=0;
            pq.push({0,L});
            while(!pq.empty()){
                double d = pq.top().first;
                int u = pq.top().second;
                pq.pop();
               if(d>dist_from_landmark[k][u]) continue;
               for(auto* edge : adj[u])
               {
                if (dist_from_landmark[k][u] + edge->length < dist_from_landmark[k][edge->v]) 
                {
                    dist_from_landmark[k][edge->v] = dist_from_landmark[k][u] + edge->length;
                    pq.push({dist_from_landmark[k][edge->v], edge->v});
                }
              
                }
            }
        }
        {
            std::priority_queue<std::pair<double, int>, 
                                std::vector<std::pair<double, int>>, 
                                std::greater<std::pair<double, int>>> pq;
            
            dist_to_landmark[k][L] = 0;
            pq.push({0, L});
            while (!pq.empty()) {
                double d = pq.top().first;
                int u = pq.top().second;
                pq.pop();
                if (d > dist_to_landmark[k][u]) continue;
                for (auto* edge : adj_r[u]) {
                    int prev_node = edge->u; 
                    if (dist_to_landmark[k][u] + edge->length < dist_to_landmark[k][prev_node]) 
                    {
                        dist_to_landmark[k][prev_node] = dist_to_landmark[k][u] + edge->length;
                        pq.push({dist_to_landmark[k][prev_node], prev_node});
                    }
                }
            }
        }
    }
    alt_initialized = true;
}

double Graph::get_alt_heuristic(int u , int target){
    if(!alt_initialized) return 0.0;
    double max_h = 0.0;
    for(int k=0; k <NUM_LANDMARKS; ++k){
        double dlt  = dist_from_landmark[k][target];
        double dlu  = dist_from_landmark[k][u];
        double dul  = dist_from_landmark[k][u];
        double dtl  = dist_from_landmark[k][target];

        if((dlt != INF) && (dlu !=INF)){
            max_h = max_h  > dlt - dlu ? max_h : dlt-dlu;
        }
        if(dul!=INF && dtl!=INF){
            max_h = std::max(max_h,dul-dtl);
        }
    }
    return std::max(0.0,max_h);
}


double Graph::weighted_a_star(int source, int target,double epsilon,double timeout){
    double w = 1.0 + (epsilon/100.0);
    std::priority_queue<std::pair<double, int>, 
        std::vector<std::pair<double, int>>, 
        std::greater<std::pair<double, int>>> pq;
    std::vector<double> g_score(N, INF);
    g_score[source] = 0.0;
    pq.push({0.0, source}); 
    while(!pq.empty()){
        double current_f = pq.top().first;
        int u = pq.top().second;
        pq.pop();if(u==target){
            return g_score[target];
        }
        if(current_f > g_score[u] + w*get_alt_heuristic(u,target)) continue;
        for(auto *edge : adj[u]){
            int v = edge->v;
            double weight = edge->length;
            if(g_score[u]+weight < g_score[v]){
                g_score[v] = g_score[u] + weight;
                double h = get_alt_heuristic(v,target);
                double f = g_score[u]+w*h;
                pq.push({f,v});
            }
        }
    }
    return (g_score[target] == INF) ? -1.0 : g_score[target];
}
