#include "graph.hpp"
#include "srtp.hpp"
#define INF 1000000000
double calc_time(double curr_time,double length, std::vector<int>speed_profile)
{
    while(length>0)
    {
        int t = static_cast<int>(curr_time/15);
        double next_slot = ((floor(curr_time/15)+1)*15);
        double time_left = next_slot - curr_time;
        double distance_possible = speed_profile[t]*time_left;
        if(length <= distance_possible)
        {
            return curr_time + length/(double)speed_profile[t];
        }
        length-=distance_possible;
        curr_time = next_slot;
    }
    return curr_time;
}
std::vector<int> Graph::handleShortesPath(SRTP sp,bool &possible,double & mtbd){
    int n = Nodes.size();
   std::priority_queue<std::pair<double,int>,
                       std::vector<std::pair<double,int>>,
                       std::greater<std::pair<double,int>>> min_heap;
   std::vector<int>dist(n,INF);
   std::vector<int>parent(n,-1);
   dist[sp.source]=0;
   min_heap.push(std::make_pair(0,sp.source));
   while(!min_heap.empty()){
        auto [d,u] = min_heap.top();
        min_heap.pop();
        if(d > dist[u]) continue;
        for(auto &e: adj[u]){
            if(! (sp.forbidden_road_types[e->road_type]) && !sp.forbidden_nodes[e->v])
            {
                if(e->v == sp.target) possible = true;
                int v = e->v;
                double length = e->length;
                if(dist[v]> dist[u]+length)
                {
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
   std::vector<int>time(n,INF);
   std::vector<int>parent(n,-1);
   time[sp.source]=0;
   min_heap.push(std::make_pair(0,sp.source));

   while(!min_heap.empty()){
        auto [t,u] = min_heap.top();
        min_heap.pop();
        if(t > time[u]) continue;
        for(auto &e: adj[u]){
            if(!sp.forbidden_road_types[e->road_type] && !sp.forbidden_nodes[e->v]){
                if(e->v == sp.target)possible = true;
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
