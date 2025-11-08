#include "../common/graph.hpp"   
#include "K_shortest_paths.h"
#include <map>
#include <vector>
#include <utility>


bool overlap_edges(std::vector<int> a, std::vector<int> b, double threshold){
    std::map<std::pair<int,int>,bool> k;
    for(int i = 0;i<a.size()-1;i++){
        std::pair<int,int> p = {a[i],a[i+1]};
        k[p] = true;
    }
    double common = 0;
    for(int i = 0;i< b.size()-1;i++){
        if(k[{b[i],b[i+1]}]) common++; 
    }
    return (common/(a.size()-1) > threshold);
}

std::vector<std::vector<int>> generateSubsets(int k, int a){
    std::vector<int> nums;
    for (int i = 1; i < k; ++i) nums.push_back(i); 
    std::vector<std::vector<int>> subsets;
    std::vector<bool> select(nums.size());
    fill(select.end() - (a-1), select.end(), true); 

    do {
        std::vector<int> subset;
        subset.push_back(0);
        for (int i = 0; i < nums.size(); ++i) if (select[i]) subset.push_back(nums[i]);
        subsets.push_back(subset);

    } while (next_permutation(select.begin(), select.end()));

    return subsets;
}



std::vector<std::pair<std::vector<int>,double>> Graph::k_shortest_paths_heustirics(KSP_H ksp){
    int source = ksp.source;
    int dest = ksp.target;
    int threshold = ksp.overlap_threshold;
    int k = ksp.k;
    KSP_E ksp2(ksp.id,ksp.source,ksp.target,50);
    std::vector<std::pair<std::vector<int>,double>> shortest_paths_heustirics;
    std::vector<std::pair<std::vector<int>,double>> shortest_paths_exact = k_shortest_paths_exact(ksp2);
    std::vector<double> distance_penality(shortest_paths_exact.size(),0);
    distance_penality[0] = 0;
    for(int i=1;i<shortest_paths_exact.size();i++){
        distance_penality[i] = ((shortest_paths_exact[i].second- shortest_paths_exact[0].second)/shortest_paths_exact[i].second) + 0.1; 
    }
    shortest_paths_heustirics.push_back(shortest_paths_exact[0]);
    std::vector<std::vector<double>> threshold_matrix(ksp2.k,std::vector<double>(ksp2.k,0.0));
    for(int i=0;i<ksp2.k;i++){
        for(int j=0;j<ksp2.k;j++){
            if(i == j) continue;
            threshold_matrix[i][j] = overlap_edges(shortest_paths_exact[i].first,shortest_paths_exact[j].first,threshold);
        }
    }

    std::vector<std::vector<int>> subsets = generateSubsets(ksp2.k,k);
    double min_penality = 1e9;
    std::vector<int> min;
    for(int i =0 ;i<subsets.size();i++){
        double penality = 0;
        for(int j = 1;j<k;j++){
            double n = 0;
            for(int l=0;l<k;l++){
                if(threshold_matrix[subsets[i][j]][subsets[i][l]]) n++;
            }
            double n =n / static_cast<double>(k);
            penality += distance_penality[subsets[i][j]] * n;
        }
        if(min_penality > penality){
            min_penality = penality;
            min = subsets[i];
        }
    }

    for(int i=0;i<k;i++){
        shortest_paths_heustirics.push_back(shortest_paths_exact[min[i]]);
    }
    return shortest_paths_heustirics;

}
