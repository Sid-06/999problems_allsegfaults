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
    int n = Nodes.size(); possible = false;
    double min_path=INF;
    int meeting_point = -1;
    std::priority_queue<std::pair<double,int>,
                        std::vector<std::pair<double,int>>,
                        std::greater<std::pair<double,int>>> h_f,h_r;
    std::vector<double>d_f(n,INF);
    std::vector<double>d_r(n,INF);
    std::vector<int>parent_f(n,-1);
    std::vector<int>parent_r(n,-1);
    std::vector<bool>vis_f(n,false);
    std::vector<bool>vis_r(n,false);
    d_f[sp.source] = 0;
    d_r[sp.target] = 0;
    h_f.push(std::make_pair(0,sp.source));
    h_r.push(std::make_pair(0,sp.target));

    while((!h_f.empty() && !h_r.empty())){
        if(h_f.top().first + h_r.top().first >= min_path){ break;}
        if(h_f.top().first <= h_r.top().first){
            auto [d,u] = h_f.top(); h_f.pop();
            if(d > d_f[u]) continue;
            if(vis_f[u]) continue;
            vis_f[u] = true;
            for(auto &e:adj[u]){
                if (sp.forbidden_edges.count({u, e->v})) {
                     continue;
                    }
                if(! (sp.forbidden_road_types[e->road_type]) && !sp.forbidden_nodes[e->v])
                {
                    int v = e->v;
                    double length = e->length;
                    if(d_f[v]> d_f[u]+length)
                        {
                            d_f[v] = d_f[u] + length;
                            parent_f[v] = u;
                            h_f.push(std::make_pair(d_f[v],v));
                        }
                    
                    if(vis_r[v]){
                        possible = true;
                        if(d_f[u]+length+d_r[v] < min_path){
                            min_path = d_f[u]+length+d_r[v];
                            meeting_point = v;
                        }
                    }
                }
            }
        }
        else{
        auto [d,u] = h_r.top(); h_r.pop();
        if(d>d_r[u]) continue;
        if(vis_r[u]) continue;
        vis_r[u] = true;
        for(auto &e:adj_r[u]){
            if (sp.forbidden_edges.count({e->v, u})) { 
                continue;
                 }
             if(! (sp.forbidden_road_types[e->road_type]) && !sp.forbidden_nodes[e->v])
            {
                int v = e->v;
                double length = e->length;
                if(d_r[v]> d_r[u]+length)
                {
                    d_r[v] = d_r[u] + length;
                    parent_r[v] = u;
                    h_r.push(std::make_pair(d_r[v],v));
                }
                if(vis_f[v]){
                    possible = true;
                    if(d_r[u]+length+d_f[v] < min_path){
                        min_path = d_r[u]+length+d_f[v];
                        meeting_point = v;                 
                    }
                }
                }
            }
        }
    }
    std::vector<int>path;
    if(!possible) return path;
    std::vector<int> path_f;
    for (int cur = meeting_point; cur != -1; cur = parent_f[cur]) path_f.push_back(cur);
    std::reverse(path_f.begin(), path_f.end()); 
    std::vector<int> path_r;
    for (int cur = meeting_point; cur != -1; cur = parent_r[cur]) path_r.push_back(cur);
    std::vector<int> full = path_f;
    for (size_t i = 1; i < path_r.size(); ++i) full.push_back(path_r[i]);
    mtbd = min_path;
    return full;
}

// std::vector<int> Graph::Djikstra(SRTP sp,bool &possible,double & mtbd){
//    int n = Nodes.size();
//    std::priority_queue<std::pair<double,int>,int> min_heap;
//    std::vector<int>time(n,INF);
//    std::vector<int>parent(n,-1);
//    time[sp.source]=0;
//    min_heap.push(std::make_pair(0,sp.source));

//    while(!min_heap.empty()){
//         auto [t,u] = min_heap.top();
//         min_heap.pop();
//         if(t > time[u]) continue;
//         for(auto &e: adj[u]){
//             if(!sp.forbidden_road_types[e->road_type] && !sp.forbidden_nodes[e->v]){
//                 if(e->v == sp.target)possible = true;
//                 int v = e->v;
//                 double tt = calc_time(t,e.length,e.speed_profile);
//                 if(time[v]> time[u]+tt){
//                     time[v] = time[u] + tt;
//                     parent[v] = u;
//                     min_heap.push(std::make_pair(time[v],u));
//             }
//             }
//         }
//    }
//    std::vector<int>path;
//    if(!possible) return path;
//    for(int i = sp.target; i!=-1; i = parent[i]){
//         path.push_back(i);
//    }
//    std::reverse(path.begin(),path.end());
//    return path;
// }
