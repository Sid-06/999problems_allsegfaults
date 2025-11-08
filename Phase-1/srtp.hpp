#pragma once
#include<unordered_map>
#include<string>
#include<set>

struct SRTP{
    int id;
    int source;
    int target;
    std::string mode;
    std::unordered_map<int,bool> forbidden_nodes;
    std::set<std::pair<int,int>> forbidden_edges;
    std::unordered_map<std::string,bool> forbidden_road_types = {
    {"expressway",false},
    {"primary",false},
    {"secondary",false},
    {"tertiary",false},
    {"local",false}
    };
};