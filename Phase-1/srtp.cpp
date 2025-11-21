#include "graph.hpp"
#include "srtp.hpp"
#define INF 1000000000
double calc_time(double curr_time,double length, std::vector<double>speed_profile)
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
 std::vector<double> Graph::euclidean_distance_to(int target)  {
        const double R = 6371.0; // km
        int n=Nodes.size();
        std::vector<double> dist(n, std::numeric_limits<double>::infinity());
        if (target < 0 || target >= n) return dist;
        const Node* t = Nodes[target];
        if (!t) return dist;

        auto rad = [](double deg){ return deg * M_PI / 180.0; };
        double lat2 = rad(t->lat), lon2 = rad(t->lon);

        for (int i = 0; i < n; ++i) {
            const Node* ni = Nodes[i];
            if (!ni) continue;
            double lat1 = rad(ni->lat), lon1 = rad(ni->lon);
            double dlat = lat2 - lat1, dlon = lon2 - lon1;
            double a = std::sin(dlat*0.5); a = a*a;
            double b = std::sin(dlon*0.5); b = b*b;
            double h = a + std::cos(lat1)*std::cos(lat2)*b;
            dist[i] = 2.0 * R * std::atan2(std::sqrt(h), std::sqrt(std::max(0.0, 1.0-h)));
        }
        return dist;
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



std::vector<int> Graph::A_starimpl(SRTP sp, bool &possible, double & mtbd){
    int n = Nodes.size();
    possible = false; // Ensure it starts as false
    
    std::priority_queue<std::pair<double,int>,
                        std::vector<std::pair<double,int>>,
                        std::greater<std::pair<double,int>>> min_heap;
    
    std::vector<double> dist(n, INF);
    std::vector<int> parent(n, -1);
    
    // Pre-calculate heuristics
    std::vector<double> h = euclidean_distance_to(sp.target);
    
    dist[sp.source] = 0;
    // Push Start Node: (f_score, u) -> (0 + h, source)
    min_heap.push(std::make_pair(dist[sp.source] + h[sp.source], sp.source));
    
    while(!min_heap.empty()){
        auto [d, u] = min_heap.top();
        min_heap.pop();
        
        // ERROR WAS HERE: You must compare d (f-score) vs dist + h (f-score)
        if (d > dist[u] + h[u]) continue;
        
        // If we reached the target, we are done. 
        if (u == sp.target) {
            possible = true; // Set your required flag here
            break;
        }
        
        for(auto &e: adj[u]){
             if(! (sp.forbidden_road_types[e->road_type]) && !sp.forbidden_nodes[e->v])
             {
                 int v = e->v;
                 double length = e->length;
                 
                 // Standard Relaxation
                 if(dist[v] > dist[u] + length){
                     dist[v] = dist[u] + length;
                     parent[v] = u;
                     // Push new f-score (dist[v] + h[v])
                     min_heap.push(std::make_pair(dist[v] + h[v], v));
                 }
             }
        }
    }
    
    std::vector<int> path;
    
    // Use your 'possible' flag check
    if(!possible) return path;
    
    for(int i = sp.target; i != -1; i = parent[i]){
        path.push_back(i);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<int> Graph::A_star_time(SRTP sp, bool &possible, double & mtbd){
      int n = Nodes.size();
    possible = false;
    std::priority_queue<std::pair<double,int>,
        std::vector<std::pair<double,int>>,
        std::greater<std::pair<double,int>>> min_heap;

    std::vector<double> arr(n, std::numeric_limits<double>::infinity());
    std::vector<int> parent(n, -1);

    if (sp.source < 0 || sp.source >= n || sp.target < 0 || sp.target >= n)
        return {};

    arr[sp.source] = 0.0;
    min_heap.push(std::make_pair(0.0, sp.source));

    while(!min_heap.empty()){
        auto [t, u] = min_heap.top();
        min_heap.pop();
        if (t > arr[u]) continue;

        if (u == sp.target) {
            possible = true;
            break;
        }

        for(auto &e: adj[u]){
            if (sp.forbidden_road_types.count(e->road_type) && sp.forbidden_road_types[e->road_type]) continue;
            if (sp.forbidden_nodes.count(e->v) && sp.forbidden_nodes[e->v]) continue;

            // calc_time returns absolute arrival time given current time
            double arrival = calc_time(arr[u], e->length, e->speed_profile);
            if (!std::isfinite(arrival)) continue;

            if (arrival < arr[e->v]) {
                arr[e->v] = arrival;
                parent[e->v] = u;
                min_heap.push(std::make_pair(arrival, e->v)); // push v not u
            }
        }
    }

    std::vector<int> path;
    if (!possible) return path;

    mtbd = arr[sp.target];
    for (int cur = sp.target; cur != -1; cur = parent[cur]) path.push_back(cur);
    std::reverse(path.begin(), path.end());
    return path;
}




//////////////////////////////////////////////djikstra time based   
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
//////////////////////////////////////////////////////////////A*
// std::vector<int> Graph::A_starimpl(SRTP sp, bool &possible, double & mtbd){
//     int n = Nodes.size();
//     possible = false; // Ensure it starts as false
    
//     std::priority_queue<std::pair<double,int>,
//                         std::vector<std::pair<double,int>>,
//                         std::greater<std::pair<double,int>>> min_heap;
    
//     std::vector<double> dist(n, INF);
//     std::vector<int> parent(n, -1);
    
//     // Pre-calculate heuristics
//     std::vector<double> h = euclidean_distance_to(sp.target);
    
//     dist[sp.source] = 0;
//     // Push Start Node: (f_score, u) -> (0 + h, source)
//     min_heap.push(std::make_pair(dist[sp.source] + h[sp.source], sp.source));
    
//     while(!min_heap.empty()){
//         auto [d, u] = min_heap.top();
//         min_heap.pop();
        
//         // ERROR WAS HERE: You must compare d (f-score) vs dist + h (f-score)
//         if (d > dist[u] + h[u]) continue;
        
//         // If we reached the target, we are done. 
//         if (u == sp.target) {
//             possible = true; // Set your required flag here
//             break;
//         }
        
//         for(auto &e: adj[u]){
//              if(! (sp.forbidden_road_types[e->road_type]) && !sp.forbidden_nodes[e->v])
//              {
//                  int v = e->v;
//                  double length = e->length;
                 
//                  // Standard Relaxation
//                  if(dist[v] > dist[u] + length){
//                      dist[v] = dist[u] + length;
//                      parent[v] = u;
//                      // Push new f-score (dist[v] + h[v])
//                      min_heap.push(std::make_pair(dist[v] + h[v], v));
//                  }
//              }
//         }
//     }
    
//     std::vector<int> path;
    
//     // Use your 'possible' flag check
//     if(!possible) return path;
    
//     for(int i = sp.target; i != -1; i = parent[i]){
//         path.push_back(i);
//     }
//     std::reverse(path.begin(), path.end());
//     return path;
// }

///////////////////////////////////////////////////single direction djikstra



// std::vector<int> Graph::A_starimpl(SRTP sp,bool &possible,double & mtbd){
//  int n = Nodes.size();
// std::priority_queue<std::pair<double,int>,
//                    std::vector<std::pair<double,int>>,
//                    std::greater<std::pair<double,int>>> min_heap;

// std::vector<double>dist(n,INF);
// std::vector<int>parent(n,-1);
// dist[sp.source]=0;
// min_heap.push(std::make_pair(0,sp.source));
// while(!min_heap.empty()){
// auto [d,u] = min_heap.top();
// min_heap.pop();
// if(d > dist[u]) continue;
// for(auto &e: adj[u]){
//     if(! (sp.forbidden_road_types[e->road_type]) && !sp.forbidden_nodes[e->v])
//        {
//          if(e->v == sp.target) possible = true;
//            int v = e->v;
//            double length = e->length;
//             if(dist[v]> dist[u]+length){
//                dist[v] = dist[u] + length;
//                parent[v] = u;
//                 min_heap.push(std::make_pair(dist[v],v));
//             }
//            }
//        }
//    }
//    std::vector<int>path;
//    if(!possible) return path;
//    for(int i = sp.target; i!=-1; i = parent[i]){
//        path.push_back(i);
//    }
//        std::reverse(path.begin(),path.end());
//        return path;
// }
//////////////////////////////////////////////////////////////A*
