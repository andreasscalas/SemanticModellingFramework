#ifndef GRAPHENE_COMPUTE_SKINNING_COR_H
#define GRAPHENE_COMPUTE_SKINNING_COR_H

#include <graphene/surface_mesh/data_structure/Surface_mesh.h>
//#include <graphene/cl/CL_state.h>

namespace graphene
{
namespace character
{

using namespace surface_mesh;

double cor_similarity(const std::vector<double> &wp, const std::vector<double> &wv);

inline void sum_vectors_div3(const std::vector<double> &in_v1, const std::vector<double> &in_v2, const std::vector<double> &in_v3, std::vector<double> &out_sum);

void compute_skinning_corV(Surface_mesh* mesh, const std::string& prop_name, const int num_joints);

void compute_skinning_corT(Surface_mesh* mesh, const std::string& prop_name, const int num_joints);

} //namespace character
} //namespace graphene

#endif
