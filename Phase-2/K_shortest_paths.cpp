#include<iostream>
#include<vector>
#include<queue>
#include<map>
#include<memory>
#include<limits>
#include<algorithm>
#include<set>
#include "graph.hpp"
#define INF 1000000000

std::list<std::pair<double,std::vector<int>>> Graph::handle_K_ShortestPaths( SRTP sp, int K){
    std::list<std::pair<double,std::vector<int>>> A; // accepted shortest paths (cost, path)
    bool possible;
    double mtbd = 0.0;

    // 1) first shortest path   
    std::vector<int> first_path = handleShortesPath(sp, possible, mtbd);
    if(!possible) return A;
    A.emplace_back(mtbd, first_path);

    using PathPair = std::pair<double, std::vector<int>>;
    auto cmp = [](const PathPair &a, const PathPair &b){ return a.first > b.first; };
    std::priority_queue<PathPair, std::vector<PathPair>, decltype(cmp)> B(cmp); // candidate paths

    auto path_key = [](const std::vector<int>& p){
        std::string s;
        for(size_t i=0;i<p.size();++i){
            if(i) s.push_back(',');
            s += std::to_string(p[i]);
        }
        return s;
    };
    std::set<std::string> seen;
    seen.insert(path_key(first_path));

    // main loop to find up to K paths
    for(int k = 1; k < K; ++k){
        // use the (k-1)-th shortest path as the previous path
        const std::vector<int> prev_path = A.back().second;
        int path_len = static_cast<int>(prev_path.size());

        // generate spur nodes along the previous path (except the last node)
        for(int i = 0; i < path_len - 1; ++i){
            // root path is prev_path[0..i]
            std::vector<int> root_path(prev_path.begin(), prev_path.begin() + i + 1);

            // build SRTP for spur search
            SRTP spur_srtp = sp;
            spur_srtp.source = prev_path[i];
            spur_srtp.target = sp.target;

            // forbid all nodes in root_path except the spur node (to enforce simple paths)
            for(int j = 0; j < i; ++j){
                spur_srtp.forbidden_nodes[root_path[j]] = true;
            }

            // forbid edges that would create previously found paths with same root
            for(const auto &accepted : A){
                const std::vector<int> &p = accepted.second;
                // check if p shares the same root
                if((int)p.size() > i && std::equal(p.begin(), p.begin() + i + 1, root_path.begin())){
                    // forbid the next edge after the root in that accepted path
                    int u = p[i];
                    int v = p[i+1];
                    spur_srtp.forbidden_edges.insert({u,v});
                }
            }

            // compute spur path from spur node to target (with current forbids)
            bool spur_possible = false;
            double spur_cost = 0.0;
            std::vector<int> spur_path = handleShortesPath(spur_srtp, spur_possible, spur_cost);
            if(!spur_possible) continue;

            // combine root + spur (avoid duplicate spur node)
            std::vector<int> total_path = root_path;
            for(size_t t = 1; t < spur_path.size(); ++t) total_path.push_back(spur_path[t]);

            std::string key = path_key(total_path);
            if(seen.count(key)) continue; // skip duplicates
            seen.insert(key);

            // compute root cost: run a shortest-path query from original source to spur node
            // (this yields the arrival time at spur node starting from original source at t=0)
            SRTP root_cost_srtp = sp;
            root_cost_srtp.target = prev_path[i];
            bool root_possible = false;
            double root_cost = 0.0;
            std::vector<int> root_check = handleShortesPath(root_cost_srtp, root_possible, root_cost);
            if(!root_possible) continue; // unexpected; skip candidate

            double total_cost = root_cost + spur_cost;
            B.push(std::make_pair(total_cost, total_path));
        }

        if(B.empty()) break; // no more candidates

        // Pop best candidate from B and add to A
        PathPair best = B.top(); B.pop();
        A.emplace_back(best);
    }

    return A;
}