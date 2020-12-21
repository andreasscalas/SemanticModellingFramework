__kernel void blend(
    __global float* in_base_vertices,
    __global float* in_base_normals,
    __global float* in_blendshape_vertices,
    __global float* in_blendshape_normals,
    __global unsigned int* in_weight_indices,
    __global float* in_weights,
    unsigned int num_vertices,
    unsigned int num_indices,
    __global float* out_vertices,
    __global float* out_normals)
{
    size_t tid = get_global_id(0);
    
    if (tid >= num_vertices)
        return;
        
    size_t i;
    float3 vtx;
    float3 nrm;
    
    
    vtx = 0.0f;
    nrm = (float3)(in_base_normals[tid*3+0], in_base_normals[tid*3+1], in_base_normals[tid*3+2]);
    
    for (i=0; i < num_indices; ++i)
    {
        vtx.x += in_weights[i] * in_blendshape_vertices[in_weight_indices[i] * num_vertices*3 + tid*3 + 0];
        vtx.y += in_weights[i] * in_blendshape_vertices[in_weight_indices[i] * num_vertices*3 + tid*3 + 1];
        vtx.z += in_weights[i] * in_blendshape_vertices[in_weight_indices[i] * num_vertices*3 + tid*3 + 2];
        
        nrm.x += in_weights[i] * in_blendshape_normals[in_weight_indices[i] * num_vertices*3 + tid*3 + 0];
        nrm.y += in_weights[i] * in_blendshape_normals[in_weight_indices[i] * num_vertices*3 + tid*3 + 1];
        nrm.z += in_weights[i] * in_blendshape_normals[in_weight_indices[i] * num_vertices*3 + tid*3 + 2];
    }
    nrm = normalize(nrm);

    out_vertices[tid*3+0] = in_base_vertices[tid*3+0] + vtx.x;
    out_vertices[tid*3+1] = in_base_vertices[tid*3+1] + vtx.y;
    out_vertices[tid*3+2] = in_base_vertices[tid*3+2] + vtx.z;
    
    out_normals[tid*3+0] = nrm.x;
    out_normals[tid*3+1] = nrm.y;
    out_normals[tid*3+2] = nrm.z;
    
    
}

