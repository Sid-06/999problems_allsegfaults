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
double Graph::get_haversine_dist(int u_id, int v_id) {
    
    const double R = 6371000.0; 
    if (u_id < 0 || u_id >= Nodes.size() || v_id < 0 || v_id >= Nodes.size()) {
        return 1e9; 
    }
    const Node& node_u = *(Nodes[u_id]);
    const Node& node_v = *(Nodes[v_id]);
    auto to_rad = [](double degree) { return degree * M_PI / 180.0; };
    
    double lat1 = to_rad(node_u.lat);
    double lon1 = to_rad(node_u.lon);
    double lat2 = to_rad(node_v.lat);
    double lon2 = to_rad(node_v.lon);
    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;
    double a = std::sin(dlat / 2.0) * std::sin(dlat / 2.0) +
    std::cos(lat1) * std::cos(lat2) *
    std::sin(dlon / 2.0) * std::sin(dlon / 2.0);
    double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(std::max(0.0, 1.0 - a)));
    return R * c;
}


std::vector<int> Graph::handleShortesPath(SRTP sp, bool &possible, double & mtbd){
   int n = static_cast<int>(Nodes.size());
    possible = false;
    if (sp.source < 0 || sp.source >= n || sp.target < 0 || sp.target >= n) {
        mtbd = -1;
        return {};
    }

    using P = std::pair<double,int>;
    std::priority_queue<P, std::vector<P>, std::greater<P>> min_heap;

    std::vector<double> dist(n, std::numeric_limits<double>::infinity());
    std::vector<int> parent(n, -1);

    dist[sp.source] = 0.0;
    min_heap.push({0.0, sp.source});

    while(!min_heap.empty()){
        auto [d,u] = min_heap.top(); min_heap.pop();
        if (d > dist[u]) continue;
        if (u == sp.target) break; // early exit when target popped

        for (auto *e : adj[u]) {
            if (!e) continue;
            auto it_rt = sp.forbidden_road_types.find(e->road_type);
            if (it_rt != sp.forbidden_road_types.end() && it_rt->second) continue;
            auto it_fn = sp.forbidden_nodes.find(e->v);
            if (it_fn != sp.forbidden_nodes.end() && it_fn->second) continue;

            int v = e->v;
            double length = e->length;
            if (dist[v] > dist[u] + length) {
                dist[v] = dist[u] + length;
                parent[v] = u;
                min_heap.push({dist[v], v});
            }
        }
    }

    std::vector<int> path;
    if (!std::isfinite(dist[sp.target])) {
        possible = false;
        mtbd = -1;
        return path;
    }

    possible = true;
    mtbd = dist[sp.target]; // mtbd is total cost (same units as edge->length)

    for (int cur = sp.target; cur != -1; cur = parent[cur]) path.push_back(cur);
    std::reverse(path.begin(), path.end());
    return path;

}
    // std::vector<int> Graph::handleShortesPath(SRTP sp,bool &possible,double & mtbd){
//     if(sp.source == sp.target) {
//         possible = true;
//         mtbd = 0.0;
//         return {sp.source};
//     }

//     int n = Nodes.size(); 
//     possible = false;
//     double min_path = INF; // Stores the best path length found so far
//     int meeting_point = -1;

//     std::priority_queue<std::pair<double,int>,
//                         std::vector<std::pair<double,int>>,
//                         std::greater<std::pair<double,int>>> h_f, h_r;
    
//     std::vector<double> d_f(n, INF), d_r(n, INF);
//     std::vector<int> parent_f(n, -1), parent_r(n, -1);
//     std::vector<bool> vis_f(n, false), vis_r(n, false);
    
//     // Initialize
//     d_f[sp.source] = 0;
//     h_f.push({0 + get_haversine_dist(sp.source, sp.target), sp.source});
    
//     d_r[sp.target] = 0;
//     h_r.push({0 + get_haversine_dist(sp.target, sp.source), sp.target});
    
//     while(!h_f.empty() && !h_r.empty()){
//         // FIX 1: Correct Termination for Bidirectional A*
//         // Stop if the most promising candidate in EITHER queue is already worse than our best found path
//         if (std::min(h_f.top().first, h_r.top().first) >= min_path) {
//             break;
//         }

//         // Expand the direction with smaller f-score (Balanced Search)
//         if(h_f.top().first <= h_r.top().first){
//             // --- Forward Expansion ---
//             auto [d, u] = h_f.top(); h_f.pop();
//             if(d > d_f[u] + get_haversine_dist(u, sp.target)) continue;
//             if(vis_f[u]) continue;
//             vis_f[u] = true;
            
//             for(auto &e : adj[u]){
//                 if (sp.forbidden_edges.count({u, e->v})) continue;
//                 if(!sp.forbidden_road_types[e->road_type] && !sp.forbidden_nodes[e->v]) {
//                     int v = e->v;
//                     double length = e->length;
                    
//                     if(d_f[v] > d_f[u] + length){
//                         d_f[v] = d_f[u] + length;
//                         parent_f[v] = u;
//                         h_f.push({d_f[v] + get_haversine_dist(v, sp.target), v});
//                     }
//                     // Meeting Check
//                     if(vis_r[v]){
//                         possible = true;
//                         if(d_f[u] + length + d_r[v] < min_path){
//                             min_path = d_f[u] + length + d_r[v];
//                             meeting_point = v;
//                         }
//                     }
//                 }
//             }
//         }
//         else{
//             // --- Reverse Expansion ---
//             auto [d, u] = h_r.top(); h_r.pop();
//             if(d > d_r[u] + get_haversine_dist(u, sp.source)) continue;
//             if(vis_r[u]) continue;
//             vis_r[u] = true;
            
//             for(auto &e : adj_r[u]){ // Ensure you have adj_r constructed!
//                 if (sp.forbidden_edges.count({e->v, u})) continue;
//                 if(!sp.forbidden_road_types[e->road_type] && !sp.forbidden_nodes[e->v]) {
//                     int v = e->v;
//                     double length = e->length;
                    
//                     if(d_r[v] > d_r[u] + length){
//                         d_r[v] = d_r[u] + length;
//                         parent_r[v] = u;
//                         h_r.push({d_r[v] + get_haversine_dist(v, sp.source), v});
//                     }
//                     // Meeting Check
//                     if(vis_f[v]){
//                         possible = true;
//                         if(d_r[u] + length + d_f[v] < min_path){
//                             min_path = d_r[u] + length + d_f[v];
//                             meeting_point = v;
//                         }
//                     }
//                 }
//             }
//         }
//     }

//     std::vector<int> path;
//     // FIX 2: Correct Return Value for "No Path"
//     if(!possible || meeting_point == -1) {
//         mtbd = INF; // Explicitly signal failure
//         return path; 
//     }

//     // Construct Path
//     std::vector<int> path_f;
//     for (int cur = meeting_point; cur != -1; cur = parent_f[cur]) path_f.push_back(cur);
//     std::reverse(path_f.begin(), path_f.end());
    
//     std::vector<int> path_r;
//     for (int cur = meeting_point; cur != -1; cur = parent_r[cur]) path_r.push_back(cur);
    
//     std::vector<int> full = path_f;
//     for (size_t i = 1; i < path_r.size(); ++i) full.push_back(path_r[i]);
    
//     mtbd = min_path;
//     return full;
// }

    // int n = Nodes.size(); 
    // possible = false;
    // double min_path = INF; // This is your 'Mu' (best path found so far)
    // int meeting_point = -1;
    
    // // Helper lambda for heuristics
    // // auto get_haversine_dist = [&](int u, int target_node) {
    // //     double dx = Nodes[u]->lat - Nodes[target_node]->lat;
    // //     double dy = Nodes[u]->lon - Nodes[target_node]->lon;
    // //     return std::sqrt(dx*dx + dy*dy);
    // // };
    
    // // Min-heaps store {f_score, node_index}
    // std::priority_queue<std::pair<double,int>,
    // std::vector<std::pair<double,int>>,
    // std::greater<std::pair<double,int>>> h_f, h_r;
    
    // // d_f and d_r store G-SCORES (actual distance from start/target)
    // std::vector<double> d_f(n, INF);
    // std::vector<double> d_r(n, INF);
    
    // std::vector<int> parent_f(n, -1);
    // std::vector<int> parent_r(n, -1);
    
    // std::vector<bool> vis_f(n, false);
    // std::vector<bool> vis_r(n, false);
    
    // d_f[sp.source] = 0;
    // h_f.push({0 + get_haversine_dist(sp.source, sp.target), sp.source});
    
    // d_r[sp.target] = 0;
    // h_r.push({0 + get_haversine_dist(sp.target, sp.source), sp.target});
    
    // while(!h_f.empty() && !h_r.empty()){
    //     double f_min_forward = h_f.top().first;
    //     double f_min_reverse = h_r.top().first;
        
    //     if (std::min(f_min_forward, f_min_reverse) >= min_path) {
    //         break;
    //     }

    //     if(h_f.top().first <= h_r.top().first){
    //         auto [f, u] = h_f.top(); h_f.pop();
    //         if(f > d_f[u] + get_haversine_dist(u, sp.target)) continue;
    //         if(vis_f[u]) continue;
    //         vis_f[u] = true;
            
    //         for(auto &e : adj[u]){
    //             if (sp.forbidden_edges.count({u, e->v})) continue;
                
    //             if(!sp.forbidden_road_types[e->road_type] && !sp.forbidden_nodes[e->v]) {
    //                 int v = e->v;
    //                 double length = e->length;
    //                 if(d_f[v] > d_f[u] + length){
    //                     d_f[v] = d_f[u] + length;
    //                     parent_f[v] = u;
    //                     h_f.push({d_f[v] + get_haversine_dist(v, sp.target), v});
    //                 }
    //                 if(vis_r[v]){
    //                      possible = true;
    //                      double current_total = d_f[u] + length + d_r[v];
    //                      if(current_total < min_path){
    //                          min_path = current_total;
    //                          meeting_point = v;
    //                      }
    //                 }
    //             }
    //         }
    //     }
    //     else {
    //         auto [f, u] = h_r.top(); h_r.pop();
            
    //         if(f > d_r[u] + get_haversine_dist(u, sp.source)) continue;
    //         if(vis_r[u]) continue;
    //         vis_r[u] = true;
            
    //         for(auto &e : adj_r[u]){ 
    //             if (sp.forbidden_edges.count({e->v, u})) continue; 
                
    //             if(!sp.forbidden_road_types[e->road_type] && !sp.forbidden_nodes[e->v]) {
    //                 int v = e->v;
    //                 double length = e->length;

    //                 if(d_r[v] > d_r[u] + length){
    //                     d_r[v] = d_r[u] + length;
    //                     parent_r[v] = u;
    //                     h_r.push({d_r[v] + get_haversine_dist(v, sp.source), v});
    //                 }
                    
    //                 if(vis_f[v]){
    //                     possible = true;
    //                     double current_total = d_r[u] + length + d_f[v];
    //                     if(current_total < min_path){
    //                         min_path = current_total;
    //                         meeting_point = v;
    //                     }
    //                 }
    //             }
    //         }
    //     }
    // }

    // std::vector<int> path;
    // if(!possible || meeting_point == -1) return path;
    
    
    // std::vector<int> path_f;
    // for (int cur = meeting_point; cur != -1; cur = parent_f[cur]) path_f.push_back(cur);
    // std::reverse(path_f.begin(), path_f.end());
    
    // std::vector<int> path_r;
    // for (int cur = meeting_point; cur != -1; cur = parent_r[cur]) path_r.push_back(cur);
    
    // std::vector<int> full = path_f;
    // for (size_t i = 1; i < path_r.size(); ++i) full.push_back(path_r[i]); // Skip duplicate meeting point
    
    // mtbd = min_path;
    // return full;


/////////////////////////////////////////////////A_star implementation
// std::vector<int> Graph::A_starimpl(SRTP sp, bool &possible, double & mtbd){
//     int n = Nodes.size();
//     possible = false; 
    
//     std::priority_queue<std::pair<double,int>,
//     std::vector<std::pair<double,int>>,
//     std::greater<std::pair<double,int>>> min_heap;
    
//     std::vector<double> dist(n, INF);
//     std::vector<int> parent(n, -1);
    
//     std::vector<double> h = euclidean_distance_to(sp.target);
    
//     dist[sp.source] = 0;
//     min_heap.push(std::make_pair(dist[sp.source] + h[sp.source], sp.source));
    
//     while(!min_heap.empty()){
//         auto [d, u] = min_heap.top();
//         min_heap.pop();
//         if (d > dist[u] + h[u]) continue;
//         if (u == sp.target) {
//             possible = true; 
//             break;
//         }
//         for(auto &e: adj[u]){
//             if(! (sp.forbidden_road_types[e->road_type]) && !sp.forbidden_nodes[e->v])
//             {
//                 int v = e->v;
//                 double length = e->length;
//                 if(dist[v] > dist[u] + length)
//                 {
//                     dist[v] = dist[u] + length;
//                     parent[v] = u;
//                     min_heap.push(std::make_pair(dist[v] + h[v], v));
//                 }
//              }
//         }
//     }
//     std::vector<int> path;
//     if(!possible) return path;
//     for(int i = sp.target; i != -1; i = parent[i])
//     {
//         path.push_back(i);
//     }
//     std::reverse(path.begin(), path.end());
//     return path;
// }

// std::vector<int> Graph::A_star_time(SRTP sp, bool &possible, double & mtbd){
//       int n = Nodes.size();
//     possible = false;
//     std::priority_queue<std::pair<double,int>,
//         std::vector<std::pair<double,int>>,
//         std::greater<std::pair<double,int>>> min_heap;

//     std::vector<double> arr(n, std::numeric_limits<double>::infinity());
//     std::vector<int> parent(n, -1);

//     if (sp.source < 0 || sp.source >= n || sp.target < 0 || sp.target >= n)
//         return {};

//     arr[sp.source] = 0.0;
//     min_heap.push(std::make_pair(0.0, sp.source));

//     while(!min_heap.empty()){
//         auto [t, u] = min_heap.top();
//         min_heap.pop();
//         if (t > arr[u]) continue;

//         if (u == sp.target) {
//             possible = true;
//             break;
//         }

//         for(auto &e: adj[u]){
//             if (sp.forbidden_road_types.count(e->road_type) && sp.forbidden_road_types[e->road_type]) continue;
//             if (sp.forbidden_nodes.count(e->v) && sp.forbidden_nodes[e->v]) continue;

//             // calc_time returns absolute arrival time given current time
//             double arrival = calc_time(arr[u], e->length, e->speed_profile);
//             if (!std::isfinite(arrival)) continue;

//             if (arrival < arr[e->v]) {
//                 arr[e->v] = arrival;
//                 parent[e->v] = u;
//                 min_heap.push(std::make_pair(arrival, e->v)); // push v not u
//             }
//         }
//     }

//     std::vector<int> path;
//     if (!possible) return path;

//     mtbd = arr[sp.target];
//     for (int cur = sp.target; cur != -1; cur = parent[cur]) path.push_back(cur);
//     std::reverse(path.begin(), path.end());
//     return path;
// }
    
    //////////////////////////////////////////////djikstra time based   
//    std::vector<int> Graph::Djikstra(SRTP sp, bool &possible, double &mtbd) {
//     int n = static_cast<int>(Nodes.size());
//     possible = false;
//     mtbd = -1;

//     if (sp.source < 0 || sp.source >= n || sp.target < 0 || sp.target >= n) return {};

//     using P = std::pair<double,int>;
//     std::priority_queue<P, std::vector<P>, std::greater<P>> pq;

//     std::vector<double> arr(n, std::numeric_limits<double>::infinity()); // arrival times (minutes or same unit as calc_time)
//     std::vector<int> parent(n, -1);

//     arr[sp.source] = 0.0;
//     pq.push({arr[sp.source], sp.source});

//     while (!pq.empty()) {
//         auto [t, u] = pq.top(); pq.pop();
//         if (t > arr[u]) continue; // stale
//         if (u == sp.target) break;

//         for (auto *e : adj[u]) {
//             if (!e) continue;
//             auto it_rt = sp.forbidden_road_types.find(e->road_type);
//             if (it_rt != sp.forbidden_road_types.end() && it_rt->second) continue;
//             auto it_fn = sp.forbidden_nodes.find(e->v);
//             if (it_fn != sp.forbidden_nodes.end() && it_fn->second) continue;

//             int v = e->v;
//             double arrival = calc_time(arr[u], e->length, e->speed_profile); // absolute arrival time
//             if (!std::isfinite(arrival)) continue;

//             if (arrival < arr[v]) {
//                 arr[v] = arrival;
//                 parent[v] = u;
//                 pq.push({arr[v], v});
//             }
//         }
//     }

//     std::vector<int> path;
//     if (!std::isfinite(arr[sp.target])) {
//         possible = false;
//         mtbd = -1;
//         return path;
//     }

//     possible = true;
//     mtbd = arr[sp.target];

//     for (int cur = sp.target; cur != -1; cur = parent[cur]) path.push_back(cur);
//     std::reverse(path.begin(), path.end());
//     return path;
// }
//////////////////////////////////////////////////