#include "../common/graph.hpp"   
#include "K_shortest_paths.h"

std::vector<std::pair<std::vector<int>,double>> Graph::k_shortest_paths_heustirics(KSP_H ksp){
    int source = ksp.source;
    int dest = ksp.target;
    int threshold = ksp.overlap_threshold;
    int k = ksp.k;
    KSP_E ksp2;
    ksp2.id = ksp.id;
    ksp2.k = 50;
    ksp2.source = ksp.source;
    ksp2.target = ksp.target;
    std::vector<std::pair<std::vector<int>,double>> shortest_paths_heustirics;
    std::vector<std::pair<std::vector<int>,double>> shortest_paths_exact = k_shortest_paths_exact(ksp2);
    std::vector<double> distance_penality(shortest_paths_exact.size(),0);
    distance_penality[0] = 0;
    for(int i=1;i<shortest_paths_exact.size();i++){
        distance_penality[i] = ((shortest_paths_exact[i].second- shortest_paths_exact[0].second)/shortest_paths_exact[i].second) + 0.1; 
    }
    shortest_paths_heustirics.push_back(shortest_paths_exact[0]);
    double total_penality = 0;
    std::vector<double> overlap_penality;



}
