__kernel void ps2mesh(
    __global float* in_mesh_vertices,
    //__global float* in_mesh_normals,
    //__global unsigned int* in_mesh_indices,
    __global float* in_ps_points,
    //__global float* in_ps_normals,
    unsigned int num_vertices,
    unsigned int num_points,
    __global unsigned int* out_closest_indices
    )
{
    size_t tid = get_global_id(0);
    
    if (tid >= num_points)
        return;

    float3 p,vtx;
    float d,dist = FLT_MAX;
    unsigned int closest_idx, i;

    p = (float3)(in_ps_points[tid*3+0], in_ps_points[tid*3+1], in_ps_points[tid*3+2]);

    for (i=0; i < num_vertices; ++i)
    {
        vtx = (float3)(in_mesh_vertices[i*3+0], in_mesh_vertices[i*3+1], in_mesh_vertices[i*3+2]);
        d = distance(vtx, p);

        if (d < dist)
        {
            dist = d;
            closest_idx = i;
        }
    }

    out_closest_indices[tid] = closest_idx;
}

__kernel void mesh2ps(
    __global float* in_mesh_vertices,
    //__global float* in_mesh_normals,
    //__global unsigned int* in_mesh_indices,
    __global float* in_ps_points,
    //__global float* in_ps_normals,
    unsigned int num_vertices,
    unsigned int num_points,
    __global unsigned int* out_closest_indices,
    __global unsigned int* out_closest_distance
    )
{
    size_t tid = get_global_id(0);

    if (tid >= num_vertices)
        return;

    float3 p,vtx;
    float d,dist = FLT_MAX;
    unsigned int closest_idx, i;

    //p = (float3)(in_ps_points[tid*3+0], in_ps_points[tid*3+1], in_ps_points[tid*3+2]);
    p = (float3)(in_mesh_vertices[tid*3+0], in_mesh_vertices[tid*3+1], in_mesh_vertices[tid*3+2]);

    for (i=0; i < num_points; ++i)
    {
        vtx = (float3)(in_ps_points[i*3+0], in_ps_points[i*3+1], in_ps_points[i*3+2]);
        d = distance(vtx, p);

        if (d < dist)
        {
            dist = d;
            closest_idx = i;
        }
    }

    out_closest_indices[tid]  = closest_idx;
    out_closest_distance[tid] = dist;
}


