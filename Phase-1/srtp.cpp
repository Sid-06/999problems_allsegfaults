#include "graph.hpp"
#define INF 1000000000
double calc_time(double curr_time,double length, std::vector<int>speed_profile){
    int t = static_cast<int>(curr_time/15);
    if(curr_time + length/(double)speed_profile[t] > ((floor(curr_time/15)+1)*15)){
        length -= speed_profile[t]*( ((floor(curr_time/15)+1)*15) - curr_time);
        curr_time = (floor(curr_time/15)+1)*15;
        return calc_time(curr_time,length,speed_profile);
    }
    else{
        return curr_time + length/(double)speed_profile[t];
    }
}
std::vector<int> Graph::handleShortesPath(SRTP sp,bool &possible,double & mtbd){
    int n = Nodes.size();
   std::priority_queue<std::pair<double,int>,int> min_heap;
   std::vector<int>dist(n);
   std::vector<int>parent(n,-1);
   dist[sp.source]=0;
   min_heap.push(std::make_pair(0,sp.source));
   while(!min_heap.empty()){
        auto [d,u] = min_heap.top();
        min_heap.pop();
        if(d > dist[u]) continue;
        for(auto &e: adj[u]){
            if(!e.forbidden_road_type[e->road_type] && !e.forbidden_nodes[e.v]){
                if(e.v == e.target) possible = true;
                int v = e->v;
                double length = e->length;
                if(dist[v]> dist[u]+length){
                    dist[v] = dist[u] + length;
                    parent[v] = u;
                    min_heap.push(std::make_pair(dist[v],u));
            }
            }
        }
   }
   std::vector<int>path;
   if(!possible) return path;
   for(int i = sp.target; i!=-1; i = parent[i]){
        path.push_back(i);
   }
   std::reverse(path.begin(),path.end());
   return path;
}

std::vector<int> Graph::Djikstra(SRTP sp,bool &possible,double & mtbd){
   int n = Nodes.size();
   std::priority_queue<std::pair<double,int>,int> min_heap;
   std::vector<int>time(n);
   std::vector<int>parent(n,-1);
   time[sp.source]=0;
   min_heap.push(std::make_pair(0,sp.source));

   while(!min_heap.empty()){
        auto [t,u] = min_heap.top();
        min_heap.pop();
        if(t > time[u]) continue;
        for(auto &e: adj[u]){
            if(!e.forbidden_road_type[e->road_type] && !e.forbidden_nodes[e.v]){
                if(e.v == e.target)possible = true;
                int v = e->v;
                double tt = calc_time(t,e.length,e.speed_profile);
                if(time[v]> time[u]+tt){
                    time[v] = time[u] + tt;
                    parent[v] = u;
                    min_heap.push(std::make_pair(time[v],u));
            }
            }
        }
   }
   std::vector<int>path;
   if(!possible) return path;
   for(int i = sp.target; i!=-1; i = parent[i]){
        path.push_back(i);
   }
   std::reverse(path.begin(),path.end());
   return path;
}
