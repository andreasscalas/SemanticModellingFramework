#ifndef FBX_FILE_H
#define FBX_FILE_H

#include <string>
#include <map>
#include <set>

#include <fbxsdk.h>
#include <graphene/character/data_structure/Character.h>

namespace graphene
{
namespace character
{

using namespace graphene::surface_mesh;

class FBX_file
{
private:
    std::string filename_;

    Character* character_;

    FbxArray<FbxNode *> fbxnode_array_;

    FbxScene* fbxscene_;

    std::map<std::string, FbxFileTexture*> fbxtextures_;

public:
    FBX_file();
    ~FBX_file();


    bool write(const Character* character, const char* filename);

    bool read(Character* character, const char* filename);

//helpers
private:
    //helpers for reading
    void traverse_nodes_rec(FbxNode* node, FbxNodeAttribute::EType type);

    void fbxskeleton_to_skeleton_rec(FbxNode* fbxnode, Skeleton& skeleton, Joint* parent);
    bool fbxmesh_to_surfacemesh(FbxNode* fbxnode, Surface_mesh_skin *skin);
    void fbxskin_to_surfacemesh(FbxSkin* fbxskin, Surface_mesh *mesh);
    bool fbxblendshape_to_blendshape(FbxBlendShape* fbxblendshape, Blendshapes* blendshapes);
    void get_material(FbxNode* fbxnode, Surface_mesh* mesh);

    //helpers for writing
    bool skeleton_to_fbx_skeleton_rec(const Joint* joint, FbxNode* fbxnode);
    bool surface_mesh_skin_to_fbxskin(const Surface_mesh_skin* skin, const Character *character);
    // Add the specified node to the node array. Also, add recursively
    // all the parent node of the specified node to the array.
    void add_node_rec(FbxNode* node);
    FbxSurfacePhong* set_phong_material(FbxNode* fbxmeshnode, const Surface_mesh_skin* skin, const std::string& meshid);
    void set_fbxtextures(FbxSurfacePhong* fbxmaterial, const Surface_mesh_skin* skin);


    //other helpers
    void fbxmat_to_mat(const FbxAMatrix& fbxmat, Mat4f& mat);
    void mat_to_fbxmat(const Mat4f& mat, FbxAMatrix& fbxmat);

    std::string extract_filepath(const std::string &filename);
    std::string extract_filename(const std::string &filename);
};


}
}



#endif
