#include "graph.hpp"  
#include "srtp.hpp" 
#include "K_shortest_paths.h"
#include <map>
#include <vector>
#include <queue>
#include <algorithm>
#include <utility>
#include <cmath>
#include <limits>

// ---------------------------------------------------------
// HELPER: Node IDs -> Edge Pointers
// ---------------------------------------------------------
std::vector<Edge*> ConvertNodesToEdgePointers(const std::vector<int>& path_nodes, const std::map<std::pair<int,int>, Edge*>& edge_lookup) {
    std::vector<Edge*> path_edges;
    // Safety check: A path must have at least 2 nodes (u->v) to have an edge
    if (path_nodes.size() < 2) return path_edges;

    for (size_t i = 0; i < path_nodes.size() - 1; ++i) {
        int u = path_nodes[i];
        int v = path_nodes[i+1];
        
        // We use .at() because we trust the pathfinding algorithm. 
        // It throws if key is missing (helping debug logic errors).
        path_edges.push_back(edge_lookup.at({u, v}));
    }
    return path_edges;
}

// ---------------------------------------------------------
// HELPER FUNCTION: Calculate Overlap (O(N))
// ---------------------------------------------------------
double CalculateOverlap(const std::vector<Edge*>& pathA, const std::vector<Edge*>& pathB, int& current_token, std::vector<int>& edge_visited_token) {
    // Increment token
    current_token++; 
        
    // Overflow protection
    if (current_token >= 2147483600) { 
        std::fill(edge_visited_token.begin(), edge_visited_token.end(), 0);
        current_token = 1;
    }

    // Mark Path A
    for (Edge* e : pathA) {
        edge_visited_token[e->id] = current_token;
    }

    // Check Path B
    int intersection_count = 0;
    for (Edge* e : pathB) {
        if (edge_visited_token[e->id] == current_token) {
            intersection_count++;
        }
    }

    double union_size = (double)(pathA.size() + pathB.size() - intersection_count);
    return (union_size == 0.0) ? 0.0 : (double)intersection_count / union_size;
}

// ---------------------------------------------------------
// HELPER FUNCTION: Get Path Penalty
// ---------------------------------------------------------
double GetPathPenalty(const std::vector<Edge*>& target_edges, 
                             double target_weight, 
                             double best_weight, 
                             const std::vector<std::vector<Edge*>>& all_current_paths, 
                             double theta,
                             int current_token,
                             std::vector<int> edge_visited_token
                        ) {
    
    // 1. Distance Penalty: Percentage diff from shortest path
    double dist_penalty = ((target_weight - best_weight) / best_weight) + 0.1;

    // 2. Overlap Penalty: Count similar paths
    int overlap_count = 0;
    for(const auto& other_edges : all_current_paths) {
        // We calculate overlap against every path in the set
        if(CalculateOverlap(target_edges, other_edges, current_token, edge_visited_token) > theta) {
            overlap_count++;
        }
    }
    
    return overlap_count * dist_penalty;
}

// ---------------------------------------------------------
// MAIN ALGORITHM: ESX
// ---------------------------------------------------------
std::vector<std::pair<std::vector<int>, double>> Graph::k_shortest_paths_heustirics(KSP_H ksp) {
    
    int source = ksp.source;
    int dest = ksp.target;
    int k = ksp.k;
    double theta = ksp.overlap_threshold / 100.0;
    
    std::vector<std::pair<std::vector<int>, double>> KSPH; 
    std::map<Edge*, int> edge_frequencies;                 
    std::vector<Edge*> CONSTANT_EDGES;                                 
    std::vector<std::priority_queue<std::pair<int, Edge*>>> heaps;

    // ---------------------------------------------------
    // 1. INITIALIZATION (Find Shortest Path)
    // ---------------------------------------------------
    SRTP query; 
    query.id = 1; 
    query.source = source;
    query.target = dest;
    query.mode = "distance";
    query.forbidden_edges = {};
    
    double weight = 0.0;
    bool possible = false;
    std::vector<int> path = handleShortesPath(query, possible, weight);
    
    if(possible && !path.empty()) KSPH.push_back({path, weight});
    else return KSPH;
    
    double Least_length = weight;
    
    // Initial Heap Setup
    std::vector<Edge*> edges_first = ConvertNodesToEdgePointers(KSPH[0].first, this->edge_lookup);
    std::priority_queue<std::pair<int, Edge*>> heap_first;
    
    for(Edge* e : edges_first){
        edge_frequencies[e]++;
        heap_first.push({edge_frequencies[e], e});
    }
    heaps.push_back(heap_first);
    
    // ---------------------------------------------------
    // 2. MAIN LOOP
    // ---------------------------------------------------
    while (KSPH.size() < k){
        if(heaps.empty()) break;
        
        auto& current_heap = heaps.back();
        
        std::vector<int> best_path; // Fixed variable name
        double best_weight = 0.0;
        double min_system_penalty = std::numeric_limits<double>::max();
        bool candidate_found = false;
        
        // Safety Brake: Limit checks to top 15 edges
        int checks = 0; 
        int max_checks = std::min((int)current_heap.size(), 15); 

        while (!current_heap.empty() && checks < max_checks) {
            checks++;
            
            Edge* edge_removed = current_heap.top().second;
            current_heap.pop();
            
            // Check Vital
            bool is_essential = false;
            for(Edge* v : CONSTANT_EDGES) { if(v == edge_removed) { is_essential = true; break; } }
            if(is_essential) continue;
            
            // Block & Route
            SRTP temp_query = query; 
            temp_query.forbidden_edges.insert({edge_removed->u, edge_removed->v});
            
            bool temp_possible = false;
            double temp_weight = 0.0;
            std::vector<int> temp_nodes = handleShortesPath(temp_query, temp_possible, temp_weight);
            
            if(!temp_possible || temp_nodes.empty()) {
                CONSTANT_EDGES.push_back(edge_removed);
                continue;
            }
            
            // ---------------------------------------------
            // CALCULATE TOTAL SYSTEM PENALTY
            // ---------------------------------------------
            
            // 1. Prepare edge lists for "Hypothetical" system state
            std::vector<std::vector<Edge*>> hypothetical_edges;
            std::vector<double> hypothetical_weights;
            
            // Add existing paths
            for(auto& p : KSPH) {
                hypothetical_edges.push_back(ConvertNodesToEdgePointers(p.first, this->edge_lookup));
                hypothetical_weights.push_back(p.second);
            }
            
            // Add Candidate path
            hypothetical_edges.push_back(ConvertNodesToEdgePointers(temp_nodes, this->edge_lookup));
            hypothetical_weights.push_back(temp_weight);

            // 2. Calculate Sum of Penalties for this hypothetical state
            double current_system_total = 0.0;
            for(int i = 0; i < hypothetical_edges.size(); i++) {
                current_system_total += GetPathPenalty(
                    hypothetical_edges[i], 
                    hypothetical_weights[i], 
                    Least_length, 
                    hypothetical_edges, 
                    theta,
                    this->current_token,
                    this->edge_visited_token
                );
            }

            // 3. Compare
            if (current_system_total < min_system_penalty) {
                min_system_penalty = current_system_total;
                best_path = temp_nodes; // Fixed variable name
                best_weight = temp_weight;
                candidate_found = true;
            }
        }
        
        if(candidate_found) {
            KSPH.push_back({best_path, best_weight});
            
            std::vector<Edge*> best_edges = ConvertNodesToEdgePointers(best_path, this->edge_lookup);
            std::priority_queue<std::pair<int, Edge*>> new_heap;
            
            for(Edge* e : best_edges) {
                edge_frequencies[e]++;
                new_heap.push({edge_frequencies[e], e});
            }
            heaps.push_back(new_heap);
        } else {
            break;
        }
    }

    // ---------------------------------------------------
    // 3. FINAL SORT (Requirement: Output in order of distance)
    // ---------------------------------------------------
    std::sort(KSPH.begin(), KSPH.end(), [](const std::pair<std::vector<int>, double>& a, const std::pair<std::vector<int>, double>& b) {
        return a.second < b.second;
    });

    return KSPH;
}