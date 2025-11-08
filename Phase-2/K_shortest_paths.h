#include <string>

struct KSP_E{
    int id;
    int source;
    int target;
    int k;
    std::string type;
    std::string mode;

    KSP_E(int id_, int source_ , int target_ , int k_, std::string type_, std::string mode_):id(id_) , source(source_), target(target_),k(k_),type(type_),mode(mode_){}

    KSP_E(int id_, int source_ , int target_ , int k_):id(id_) , source(source_), target(target_),k(k_){}
};



struct KSP_H{
    int id;
    int source;
    int target;
    int k;
    std::string type;
    int overlap_threshold;

    KSP_H(int id_, int source_ , int target_ , int k_, std::string type_, int overlap_threshold_) : id(id_) , source(source_), target(target_),k(k_),type(type_),overlap_threshold(overlap_threshold_){}

};