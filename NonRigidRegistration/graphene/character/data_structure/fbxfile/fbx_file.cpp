#include "fbx_file.h"



namespace graphene
{
namespace character
{

FBX_file::FBX_file()
{

}

FBX_file::~FBX_file()
{

}

bool FBX_file::write(const Character *character, const char *filename)
{
    filename_ = filename;

    FbxManager* fbx_manager = FbxManager::Create();
    FbxIOSettings* iosettings = FbxIOSettings::Create(fbx_manager, IOSROOT);
    fbx_manager->SetIOSettings(iosettings);

    FbxExporter* exporter = FbxExporter::Create(fbx_manager,"");
    bool exportstatus = exporter->Initialize(filename, -1, fbx_manager->GetIOSettings());
    if (! exportstatus)
    {
        return false;
    }

    fbxscene_ = FbxScene::Create(fbx_manager, character->get_name().c_str());

    //set coordinate system to opengl (y-up, righthanded)
    fbxscene_->GetGlobalSettings().SetAxisSystem(FbxAxisSystem::eOpenGL);
    //set unit to meters
    fbxscene_->GetGlobalSettings().SetSystemUnit(FbxSystemUnit::m);

    FbxNode* fbxroot = fbxscene_->GetRootNode();

    if (! skeleton_to_fbx_skeleton_rec(character->skeleton().root_, fbxroot))
        return false;



    for (size_t i=0; i < character->skins().size(); ++i)
    {
        if (! surface_mesh_skin_to_fbxskin(character->skins()[i], character))
        {
            return false;
        }
    }

    FbxPose* bindpose = FbxPose::Create(fbxscene_,"bindpose");
    bindpose->SetIsBindPose(true);
    for (int i=0; i < fbxnode_array_.GetCount(); ++i)
    {
        FbxNode*  fbxnode   = fbxnode_array_.GetAt(i);
        FbxMatrix bindmatrix = fbxnode->EvaluateGlobalTransform();
        bindpose->Add(fbxnode, bindmatrix);
    }
    // Add the pose to the scene
    fbxscene_->AddPose(bindpose);


    exporter->Export(fbxscene_);

    exporter->Destroy();

    return true;
}

bool FBX_file::read(Character *character, const char *filename)
{
    filename_ = filename;
    character_ = character;

    FbxManager* fbx_manager = FbxManager::Create();

    FbxIOSettings* iosettings = FbxIOSettings::Create(fbx_manager, IOSROOT);
    fbx_manager->SetIOSettings(iosettings);

    FbxImporter* importer = FbxImporter::Create(fbx_manager,"");
    bool importstatus = importer->Initialize(filename, -1, fbx_manager->GetIOSettings());
    if (! importstatus)
    {
        return false;
    }

    FbxScene* scene = FbxScene::Create(fbx_manager, character->get_name().c_str());

    importer->Import(scene);

    //traverse for skeleton first, since we need it to properly build skinned mesh
    traverse_nodes_rec(scene->GetRootNode(), FbxNodeAttribute::eSkeleton);

    traverse_nodes_rec(scene->GetRootNode(), FbxNodeAttribute::eMesh);

    importer->Destroy();

    return true;
}

void FBX_file::traverse_nodes_rec(FbxNode *fbxnode, FbxNodeAttribute::EType type)
{

    if (fbxnode->GetNodeAttribute() != NULL)
    {
        if (type == FbxNodeAttribute::eMesh && fbxnode->GetNodeAttribute()->GetAttributeType() == FbxNodeAttribute::eMesh)
        {
            Surface_mesh_skin* skin = new Surface_mesh_skin();
            if (fbxmesh_to_surfacemesh(fbxnode, skin))
            {
                skin->init_properties();
                character_->skins().push_back(skin);
            }
            else
            {
                delete skin;
            }
        }
        else if (type == FbxNodeAttribute::eSkeleton && fbxnode->GetNodeAttribute()->GetAttributeType() == FbxNodeAttribute::eSkeleton)
        {
            fbxskeleton_to_skeleton_rec(fbxnode, character_->skeleton(), NULL);
            return; //do not go further down the hierarchy, since fbxskeleton_to_skeleton_rec() does this
        }
    }

    for (int i=0; i < fbxnode->GetChildCount(false); ++i)
    {
        traverse_nodes_rec(fbxnode->GetChild(i), type);
    }
}

void FBX_file::fbxskeleton_to_skeleton_rec(FbxNode* fbxnode, Skeleton& skeleton, Joint *parent)
{
    FbxSkeleton* fbxskeleton = fbxnode->GetSkeleton();
    if (fbxskeleton == NULL)
        return;

    Joint* current;
    FbxAMatrix fbxlcl = fbxnode->EvaluateLocalTransform();
    Mat4f local;

    fbxmat_to_mat(fbxlcl, local);

    current = new Joint(fbxnode->GetName(), local);
    skeleton.joints_.push_back(current);

    if (fbxskeleton->GetSkeletonType() == FbxSkeleton::eRoot || parent == NULL)
    {
        skeleton.root_ = current;
    }
    else if (fbxskeleton->GetSkeletonType() == FbxSkeleton::eLimbNode)
    {
        current->parent_ = parent;
        parent->children_.push_back(current);
    }


    for (int i=0; i < fbxnode->GetChildCount(false); ++i)
    {
        fbxskeleton_to_skeleton_rec(fbxnode->GetChild(i), skeleton, current);
    }
}

bool FBX_file::fbxmesh_to_surfacemesh(FbxNode *fbxnode, Surface_mesh_skin* skin)
{
    FbxMesh* fbxmesh = fbxnode->GetMesh();
    if (fbxmesh == NULL)
    {
        return false;
    }

    //mesh properties
    Surface_mesh::Vertex_property<Normal> sm_v_normals_;
    Surface_mesh::Vertex_property<Color>  sm_v_colors_;

    Surface_mesh::Halfedge_property<Normal>              sm_h_normals_;
    Surface_mesh::Halfedge_property<Texture_coordinate>  sm_h_texcoords_;

    Surface_mesh::Mesh_property<std::string> m_id;

    FbxLayer* fbxlayer = fbxmesh->GetLayer(0);

    //if we do not have at least layer 0 we can not read this mesh
    if (fbxlayer == NULL)
    {
        return false;
    }


    m_id = skin->mesh_property<std::string>("m:id");
    m_id[0] = fbxnode->GetName();

    skin->reserve(fbxmesh->GetControlPointsCount(), fbxmesh->GetControlPointsCount() * 3, fbxmesh->GetControlPointsCount() / 3);

    Mat4f transform;
    fbxmat_to_mat(fbxnode->EvaluateGlobalTransform(), transform);

    int idx,c;


    FbxVector4* cp = fbxmesh->GetControlPoints();

    Surface_mesh::Vertex sv;
    for (idx=0; idx < fbxmesh->GetControlPointsCount(); ++idx)
    {
        const FbxVector4 &p = cp[idx];
        const Point v = affine_transform(transform,Point(p[0],p[1],p[2]));
        sv = skin->add_vertex(v);
    }




    std::vector<Surface_mesh::Vertex> idcs;
    int t;

    for (idx=0; idx < fbxmesh->GetPolygonCount(); ++idx)
    {
        const int pc = fbxmesh->GetPolygonSize(idx);

        if (pc != 3)
        {
            std::cerr << "FBX_file::fbxmesh_to_surfacemesh: [ERROR] Mesh \"" << m_id[0] << "\" is not a trianlge mesh." << std::endl;
            return false;
        }

        idcs.clear();
        for (t=0; t < pc; ++t)
        {
            idcs.push_back(Surface_mesh::Vertex(fbxmesh->GetPolygonVertex(idx, t)));
        }

        skin->add_face(idcs);
    }

    FbxLayerElementNormal* fbxnormals = fbxlayer->GetNormals();
    if (fbxnormals != NULL)
    {
        if (fbxnormals->GetMappingMode() == FbxLayerElement::eByControlPoint)
        {
            sm_v_normals_ = skin->vertex_property<Normal>("v:normal");

            if (fbxnormals->GetReferenceMode() == FbxLayerElement::eDirect) //easy, just copy
            {
                Surface_mesh::Vertex_iterator vit;
                for (vit = skin->vertices_begin(); vit != skin->vertices_end(); ++vit)
                {
                    const FbxVector4 n = fbxnormals->GetDirectArray().GetAt((*vit).idx());
                    sm_v_normals_[*vit] = Normal(n[0],n[1],n[2]);
                }
            }
            else if (fbxnormals->GetReferenceMode() == FbxLayerElement::eIndexToDirect)// use index array
            {
                Surface_mesh::Vertex_iterator vit;
                for (vit = skin->vertices_begin(); vit != skin->vertices_end(); ++vit)
                {
                    const FbxVector4 n = fbxnormals->GetDirectArray().GetAt(fbxnormals->GetIndexArray().GetAt((*vit).idx()));
                    sm_v_normals_[*vit] = Normal(n[0],n[1],n[2]);
                }
            }
        }
        else if (fbxnormals->GetMappingMode() == FbxLayerElement::eByPolygonVertex)
        {
            sm_h_normals_ = skin->halfedge_property<Normal>("h:normal");
            if (fbxnormals->GetReferenceMode() == FbxLayerElement::eDirect)
            {
                Surface_mesh::Face_iterator fit;
                Surface_mesh::Halfedge_around_face_circulator hfc,hfc_end;
                c=0;
                for (fit = skin->faces_begin(); fit != skin->faces_end(); ++fit)
                {
                    hfc = hfc_end = skin->halfedges(*fit);
                    do
                    {
                        const FbxVector4 n = fbxnormals->GetDirectArray().GetAt(c);
                        sm_h_normals_[*hfc] = Normal(n[0], n[1], n[2]);
                        ++c;
                    }
                    while (++hfc != hfc_end);
                }
            }
            else if (fbxnormals->GetReferenceMode() == FbxLayerElement::eIndexToDirect)// this is untested
            {
                Surface_mesh::Face_iterator fit;
                Surface_mesh::Halfedge_around_face_circulator hfc,hfc_end;
                c=0;
                for (fit = skin->faces_begin(); fit != skin->faces_end(); ++fit)
                {
                    hfc = hfc_end = skin->halfedges(*fit);
                    do
                    {
                        const FbxVector4 n = fbxnormals->GetDirectArray().GetAt(fbxnormals->GetIndexArray().GetAt(c));
                        sm_h_normals_[*hfc] = Normal(n[0], n[1], n[2]);
                        ++c;
                    }
                    while (++hfc != hfc_end);
                }
            }

        }
        else
        {
            std::cout << "Normals of skin \"" << m_id[0] << "\" have an unsupported mapping!" << std::endl;
        }
    }

    FbxLayerElementUV* fbxuvs = fbxlayer->GetUVs();
    if (fbxuvs != NULL)
    {
        if (fbxuvs->GetMappingMode() == FbxLayerElement::eByPolygonVertex)
        {
            sm_h_texcoords_ = skin->halfedge_property<Texture_coordinate>("h:texcoord");
            if (fbxuvs->GetReferenceMode() == FbxLayerElement::eDirect)
            {
                Surface_mesh::Face_iterator fit;
                Surface_mesh::Halfedge_around_face_circulator hfc,hfc_end;
                c=0;
                for (fit = skin->faces_begin(); fit != skin->faces_end(); ++fit)
                {
                    hfc = hfc_end = skin->halfedges(*fit);
                    do
                    {
                        const FbxVector2 uv = fbxuvs->GetDirectArray().GetAt(c);
                        sm_h_texcoords_[*hfc] = Normal(uv[0], uv[1], 0.0f);
                        ++c;
                    }
                    while (++hfc != hfc_end);
                }
            }
            else if (fbxuvs->GetReferenceMode() == FbxLayerElement::eIndexToDirect)// this is untested
            {
                Surface_mesh::Face_iterator fit;
                Surface_mesh::Halfedge_around_face_circulator hfc,hfc_end;
                c=0;
                for (fit = skin->faces_begin(); fit != skin->faces_end(); ++fit)
                {
                    hfc = hfc_end = skin->halfedges(*fit);
                    do
                    {
                        const FbxVector2 uv = fbxuvs->GetDirectArray().GetAt(fbxuvs->GetIndexArray().GetAt(c));
                        sm_h_texcoords_[*hfc] = Normal(uv[0], uv[1], 0.0f);
                        ++c;
                    }
                    while (++hfc != hfc_end);
                }
            }

        }
        else
        {
            std::cout << "UVs of skin \"" << m_id[0] << "\" have an unsupported mapping!" << std::endl;
        }
    }


    FbxLayerElementVertexColor* fbxcolors = fbxlayer->GetVertexColors();
    if (fbxcolors != NULL)
    {
        if (fbxcolors->GetMappingMode() == FbxLayerElement::eByControlPoint)
        {
            sm_v_colors_ = skin->vertex_property<Color>("v:color");

            if (fbxcolors->GetReferenceMode() == FbxLayerElement::eDirect) //easy, just copy
            {
                Surface_mesh::Vertex_iterator vit;
                for (vit = skin->vertices_begin(); vit != skin->vertices_end(); ++vit)
                {
                    const FbxColor c = fbxcolors->GetDirectArray().GetAt((*vit).idx());
                    sm_v_colors_[*vit] = Color(c[0],c[1],c[2]);
                }
            }
            else if (fbxcolors->GetReferenceMode() == FbxLayerElement::eIndexToDirect)// use index array
            {
                Surface_mesh::Vertex_iterator vit;
                for (vit = skin->vertices_begin(); vit != skin->vertices_end(); ++vit)
                {
                    const FbxColor c = fbxcolors->GetDirectArray().GetAt(fbxcolors->GetIndexArray().GetAt((*vit).idx()));
                    sm_v_colors_[*vit] = Color(c[0],c[1],c[2]);
                }
            }
        }
        else
        {
            std::cout << "Vertexcolors of skin \"" << m_id[0] << "\" have an unsupported mapping!" << std::endl;
        }
    }

    for (idx = 0; idx < fbxmesh->GetDeformerCount(); ++idx)
    {
        FbxDeformer* deformer = fbxmesh->GetDeformer(idx);


        if (deformer->GetDeformerType() == FbxDeformer::eSkin)
        {
            FbxSkin* fbxskin = FbxCast<FbxSkin>(deformer);
            if (fbxskin != NULL)
            {
                fbxskin_to_surfacemesh(fbxskin, skin);
            }
        }

        else if (deformer->GetDeformerType() == FbxDeformer::eBlendShape)
        {
            FbxBlendShape* fbxblendshape = FbxCast<FbxBlendShape>(deformer);
            if (fbxblendshape != NULL)
            {
                Blendshapes* blendshapes = new Blendshapes();
                blendshapes->set_base(skin);
                if (fbxblendshape_to_blendshape(fbxblendshape, blendshapes))
                {
                    character_->blendshapes().push_back(blendshapes);
                }
                else
                {
                    delete blendshapes;
                }
            }
        }
    }

    get_material(fbxnode, skin);

    return true;
}

void FBX_file::fbxskin_to_surfacemesh(FbxSkin *fbxskin, Surface_mesh *mesh)
{

    int c,idx;
    FbxCluster* fbxcluster;

    std::vector< std::map<int, float> > dw(mesh->n_vertices());

    for (c=0; c < fbxskin->GetClusterCount(); ++c)
    {
        fbxcluster = fbxskin->GetCluster(c);

        const int d = character_->skeleton().get_joint_idx(fbxcluster->GetLink()->GetName());

        if (d == -1)
        {
            std::cout << "Could not map FbxCluster with link name \""
                      << fbxcluster->GetLink()->GetName()
                      << "\"."
                      << std::endl;
            continue;
        }

        int *indices = fbxcluster->GetControlPointIndices();
        double* weights = fbxcluster->GetControlPointWeights();


        for (idx=0; idx < fbxcluster->GetControlPointIndicesCount(); ++idx)
        {
            const int cp_idx = indices[idx];
            const double w   = weights[idx];
            dw[cp_idx][d] = (float) w;
        }
    }

    Surface_mesh::Vertex_property<Vec4f> depend1 = mesh->vertex_property<Vec4f>("v:skin_depend");
    Surface_mesh::Vertex_property<Vec4f> depend2 = mesh->vertex_property<Vec4f>("v:skin_depend2");
    Surface_mesh::Vertex_property<Vec4f> weight1 = mesh->vertex_property<Vec4f>("v:skin_weight");
    Surface_mesh::Vertex_property<Vec4f> weight2 = mesh->vertex_property<Vec4f>("v:skin_weight2");


    float w_sum;
    std::map<int, float>::const_iterator mit;
    for (size_t i=0; i < dw.size(); ++i)
    {
        const std::map<int, float> &m = dw[i];
        Vec4f &dv1 = depend1.vector()[i];
        Vec4f &wv1 = weight1.vector()[i];
        Vec4f &dv2 = depend2.vector()[i];
        Vec4f &wv2 = weight2.vector()[i];

        dv1 = dv2 = wv1 = wv2 = Vec4f(0.0f);

        w_sum = 0.0f;
        for (mit = m.begin(), c=0; mit != m.end() && c < 8; ++mit,++c)
        {
            const int d = mit->first;
            const float w = mit->second;

            w_sum += w;

            if (c < 4)
            {
                dv1[c] = (float) d;
                wv1[c] = w;
            }
            else
            {
                dv2[c-4] = (float) d;
                wv2[c-4] = w;
            }
        }

        const float w_sum_inv = 1.0f / w_sum;
        wv1 *= w_sum_inv;
        wv2 *= w_sum_inv;
    }
}

bool FBX_file::fbxblendshape_to_blendshape(FbxBlendShape *fbxblendshape, Blendshapes *blendshapes)
{
    int idx_channel,idx_shape,c;
    FbxBlendShapeChannel* fbxbschannel;
    FbxShape* fbxtargetshape;
    FbxLayer* layer;
    Blendshapes::Target_shape* target;
    Surface_mesh::Mesh_property<std::string> m_id;
    Surface_mesh::Vertex_property<Normal> sm_v_normal;
    Surface_mesh::Halfedge_property<Normal> sm_h_normal;


    for (idx_channel=0; idx_channel < fbxblendshape->GetBlendShapeChannelCount(); ++idx_channel)
    {
        fbxbschannel = fbxblendshape->GetBlendShapeChannel(idx_channel);

        for (idx_shape=0; idx_shape < fbxbschannel->GetTargetShapeCount(); ++idx_shape)
        {
            fbxtargetshape = fbxbschannel->GetTargetShape(idx_shape);

            target = new Blendshapes::Target_shape();
            m_id = target->mesh_property<std::string>("m:id");
            m_id[0] = fbxtargetshape->GetName();

            target->reserve(blendshapes->base()->n_vertices(), blendshapes->base()->n_edges(), blendshapes->base()->n_faces());

            FbxVector4* cp = fbxtargetshape->GetControlPoints();
            for (c=0; c < fbxtargetshape->GetControlPointsCount(); ++c)
            {
                const FbxVector4 &p = cp[c];
                target->add_vertex(Point(p[0],p[1],p[2]));
            }

            Surface_mesh::Face_iterator fit;
            Surface_mesh::Vertex_around_face_circulator vfc,vfc_end;
            std::vector<Surface_mesh::Vertex> vertices;
            for (fit = blendshapes->base()->faces_begin(); fit != blendshapes->base()->faces_end(); ++fit)
            {
                vfc = vfc_end = blendshapes->base()->vertices(*fit);

                vertices.clear();
                do
                {
                    vertices.push_back(*vfc);
                }
                while (++vfc != vfc_end);

                target->add_face(vertices);
            }

            layer = fbxtargetshape->GetLayer(0);
            if (layer != NULL)
            {
                FbxLayerElementNormal* fbxnormals = layer->GetNormals();
                if (fbxnormals != NULL)
                {


                    if (fbxnormals->GetMappingMode() == FbxLayerElement::eByControlPoint
                            && fbxnormals->GetDirectArray().GetCount() == target->n_vertices())
                    {
                        sm_v_normal = target->vertex_property<Normal>("v:normal");
                        for (c=0; c < fbxnormals->GetDirectArray().GetCount(); ++c)
                        {
                            FbxVector4 n = fbxnormals->GetDirectArray()[c];
                            sm_v_normal.vector()[c] = Normal(n[0],n[1],n[2]);
                        }
                    }

                    else if (fbxnormals->GetMappingMode() == FbxLayerElement::eByPolygonVertex
                             && fbxnormals->GetDirectArray().GetCount() == target->n_halfedges())
                    {
                        sm_h_normal = target->halfedge_property<Normal>("h:normal");
                        for (c=0; c < fbxnormals->GetDirectArray().GetCount(); ++c)
                        {
                            FbxVector4 n = fbxnormals->GetDirectArray()[c];
                            sm_h_normal.vector()[c] = Normal(n[0],n[1],n[2]);
                        }
                    }

                }
            }

            blendshapes->add_target(target);

        }
    }

    return ! blendshapes->targets().empty();
}

void FBX_file::get_material(FbxNode *fbxnode, Surface_mesh *mesh)
{
    FbxSurfaceMaterial* fbxmaterial = fbxnode->GetMaterial(0);
    if (fbxmaterial == NULL)// there is no material in this node
    {
        return;
    }

    Surface_mesh::Mesh_property<Vec3f> m_ambient_color  = mesh->mesh_property<Vec3f>("m:ambient_color");
    Surface_mesh::Mesh_property<Vec3f> m_diffuse_color  = mesh->mesh_property<Vec3f>("m:diffuse_color");
    Surface_mesh::Mesh_property<Vec4f> m_specular_color = mesh->mesh_property<Vec4f>("m:specular_color");
    Surface_mesh::Mesh_property<float> m_alpha          = mesh->mesh_property<float>("m:alpha");

    Vec3f &a = m_ambient_color[0];
    Vec3f &d = m_diffuse_color[0];
    Vec4f &s = m_specular_color[0];
    float &alpha = m_alpha[0];

    FbxSurfacePhong* fbxphong = FbxCast<FbxSurfacePhong>(fbxmaterial);
    if (fbxphong != NULL)
    {
        FbxDouble3 fbxambient = fbxphong->Ambient.Get();
        a = fbxphong->AmbientFactor.Get() * Vec3f(fbxambient[0],fbxambient[1], fbxambient[2]);
        FbxDouble3 fbxdiffuse = fbxphong->Diffuse.Get();
        d = fbxphong->DiffuseFactor.Get() * Vec3f(fbxdiffuse[0],fbxdiffuse[1], fbxdiffuse[2]);
        FbxDouble3 fbxspecular  = fbxphong->Specular.Get();
        FbxDouble  fbxshininess = fbxphong->Shininess.Get();
        s = fbxphong->SpecularFactor.Get() * Vec4f(fbxspecular[0],fbxspecular[1], fbxspecular[2], 0.0f);
        s[3] = fbxshininess;
        FbxDouble  fbxalpha     = fbxphong->TransparencyFactor.Get();
        alpha                   = (float) fbxalpha;



        if (fbxphong->Diffuse.GetSrcObjectCount() > 0)
        {
            FbxFileTexture* texture = NULL;
            for (int i=0; i < fbxphong->Diffuse.GetSrcObjectCount(); ++i)
            {
                texture = FbxCast<FbxFileTexture>(fbxphong->Diffuse.GetSrcObject(i));
                if (texture != NULL)
                    break;
            }

            if (texture != NULL)
            {
                Surface_mesh::Mesh_property<std::string> m_texname = mesh->mesh_property<std::string>("m:texturename");
                std::string &texfilename = m_texname[0];
                texfilename = extract_filepath(filename_);
                texfilename += extract_filename(texture->GetFileName());
            }
        }


        if (fbxphong->Specular.GetSrcObjectCount() > 0)
        {
            FbxFileTexture* texture = NULL;
            for (int i=0; i < fbxphong->Specular.GetSrcObjectCount(); ++i)
            {
                texture = FbxCast<FbxFileTexture>(fbxphong->Specular.GetSrcObject(i));
                if (texture != NULL)
                    break;
            }

            if (texture != NULL)
            {
                Surface_mesh::Mesh_property<std::string> m_texname = mesh->mesh_property<std::string>("m:texturename_spec");
                std::string &texfilename = m_texname[0];
                texfilename = extract_filepath(filename_);
                texfilename += extract_filename(texture->GetFileName());
            }
        }

        if (fbxphong->NormalMap.GetSrcObjectCount() > 0)
        {
            FbxFileTexture* texture = NULL;
            for (int i=0; i < fbxphong->NormalMap.GetSrcObjectCount(); ++i)
            {
                texture = FbxCast<FbxFileTexture>(fbxphong->NormalMap.GetSrcObject(i));
                if (texture != NULL)
                    break;
            }

            if (texture != NULL)
            {
                Surface_mesh::Mesh_property<std::string> m_texname = mesh->mesh_property<std::string>("m:texturename_nm");
                std::string &texfilename = m_texname[0];
                texfilename = extract_filepath(filename_);
                texfilename += extract_filename(texture->GetFileName());
            }
        }

        if (fbxphong->Bump.GetSrcObjectCount() > 0)
        {
            FbxFileTexture* texture = NULL;
            for (int i=0; i < fbxphong->Bump.GetSrcObjectCount(); ++i)
            {
                texture = FbxCast<FbxFileTexture>(fbxphong->Bump.GetSrcObject(i));
                if (texture != NULL)
                    break;
            }

            if (texture != NULL)
            {
                Surface_mesh::Mesh_property<std::string> m_texname = mesh->mesh_property<std::string>("m:texturename_nm");
                std::string &texfilename = m_texname[0];
                texfilename = extract_filepath(filename_);
                texfilename += extract_filename(texture->GetFileName());
            }

        }

        return;
    }

    FbxSurfaceLambert* fbxlambert = FbxCast<FbxSurfaceLambert>(fbxmaterial);
    if (fbxlambert != NULL)
    {
        FbxDouble3 fbxambient = fbxlambert->Ambient.Get();
        a = fbxlambert->AmbientFactor.Get() * Vec3f(fbxambient[0],fbxambient[1], fbxambient[2]);
        FbxDouble3 fbxdiffuse = fbxlambert->Diffuse.Get();
        d = fbxlambert->DiffuseFactor.Get() * Vec3f(fbxdiffuse[0],fbxdiffuse[1], fbxdiffuse[2]);
        s = Vec4f(0.0f);

        if (fbxlambert->Diffuse.GetSrcObjectCount() > 0)
        {
            FbxFileTexture* texture = NULL;
            for (int i=0; i < fbxlambert->Diffuse.GetSrcObjectCount(); ++i)
            {
                texture = FbxCast<FbxFileTexture>(fbxlambert->Diffuse.GetSrcObject(i));
                if (texture != NULL)
                    break;
            }

            if (texture != NULL)
            {
                Surface_mesh::Mesh_property<std::string> m_texname = mesh->mesh_property<std::string>("m:texturename");
                std::string &texfilename = m_texname[0];
                texfilename = extract_filepath(filename_);
                texfilename += extract_filename(texture->GetFileName());
            }
        }

        if (fbxlambert->NormalMap.GetSrcObjectCount() > 0)
        {
            FbxFileTexture* texture = NULL;
            for (int i=0; i < fbxlambert->NormalMap.GetSrcObjectCount(); ++i)
            {
                texture = FbxCast<FbxFileTexture>(fbxlambert->NormalMap.GetSrcObject(i));
                if (texture != NULL)
                    break;
            }

            if (texture != NULL)
            {
                Surface_mesh::Mesh_property<std::string> m_texname = mesh->mesh_property<std::string>("m:texturename_nm");
                std::string &texfilename = m_texname[0];
                texfilename = extract_filepath(filename_);
                texfilename += extract_filename(texture->GetFileName());
            }
        }

        if (fbxlambert->Bump.GetSrcObjectCount() > 0)
        {
            FbxFileTexture* texture = NULL;
            for (int i=0; i < fbxlambert->Bump.GetSrcObjectCount(); ++i)
            {
                texture = FbxCast<FbxFileTexture>(fbxlambert->Bump.GetSrcObject(i));
                if (texture != NULL)
                    break;
            }

            if (texture != NULL)
            {
                Surface_mesh::Mesh_property<std::string> m_texname = mesh->mesh_property<std::string>("m:texturename_nm");
                std::string &texfilename = m_texname[0];
                texfilename = extract_filepath(filename_);
                texfilename += extract_filename(texture->GetFileName());
            }

        }

        return;
    }


}

bool FBX_file::skeleton_to_fbx_skeleton_rec(const Joint *joint, FbxNode *fbxnode)
{

    FbxSkeleton* fbxjoint = FbxSkeleton::Create(fbxscene_, joint->get_name().c_str());

    if (joint->parent_ == NULL)
    {
        fbxjoint->SetSkeletonType(FbxSkeleton::eRoot);
    }
    /*
    else if (joint->children_.empty())
    {
        fbxjoint->SetSkeletonType(FbxSkeleton::eEffector);
    }
    */
    else
    {
        fbxjoint->SetSkeletonType(FbxSkeleton::eLimbNode);
    }

    fbxjoint->Size.Set(0.5);

    FbxNode* joint_node = FbxNode::Create(fbxscene_, joint->get_name().c_str());
    joint_node->SetNodeAttribute(fbxjoint);

    joint_node->SetRotationOrder(FbxNode::eDestinationPivot, FbxEuler::eOrderXYZ);

    FbxAMatrix fbxlocalmatrix;

    mat_to_fbxmat(joint->local_, fbxlocalmatrix);


    joint_node->LclRotation.Set(fbxlocalmatrix.GetR());
    joint_node->LclTranslation.Set(fbxlocalmatrix.GetT());


    fbxnode->AddChild(joint_node);


    for (size_t i=0; i < joint->children_.size(); ++i)
    {
        const Joint* child = joint->children_[i];
        skeleton_to_fbx_skeleton_rec(child, joint_node);
    }

    return true;
}

bool FBX_file::surface_mesh_skin_to_fbxskin(const Surface_mesh_skin *skin, const Character* character)
{
    Surface_mesh::Mesh_property<std::string> mid = skin->get_mesh_property<std::string>("m:id");
    std::string meshid = "no_name";
    if (mid)
    {
        meshid = mid[0];
    }

    std::string tmp_str;


    tmp_str = meshid;
    FbxNode* fbxmeshnode = FbxNode::Create(fbxscene_, tmp_str.c_str());
    fbxscene_->GetRootNode()->AddChild(fbxmeshnode);

    Surface_mesh::Vertex_iterator vit;
    Surface_mesh::Face_iterator fit;

    tmp_str = meshid + "_mesh";
    FbxMesh* fbxmesh = FbxMesh::Create(fbxscene_, tmp_str.c_str());

    FbxSurfacePhong *fbxmaterial = set_phong_material(fbxmeshnode, skin, meshid);

    set_fbxtextures(fbxmaterial, skin);



    fbxmeshnode->SetNodeAttribute(fbxmesh);

    fbxmesh->InitControlPoints(skin->n_vertices());

    FbxVector4* cp = fbxmesh->GetControlPoints();

    //if we have vertices, add them to fbxmesh
    if (skin->vertices_)
    {
        int c=0;
        for (vit = skin->vertices_begin(); vit != skin->vertices_end(); ++vit)
        {
            const Point p = skin->vertices_[*vit];
            cp[c] = FbxVector4(p[0],p[1],p[2]);
            ++c;
        }

        fbxmesh->ReservePolygonCount(skin->n_faces()*3);
        Surface_mesh::Face_iterator fit;
        Surface_mesh::Vertex_around_face_circulator vfc,vfc_end;
        for (fit = skin->faces_begin(); fit != skin->faces_end(); ++fit)
        {
            fbxmesh->BeginPolygon(-1,-1,-1,false);
            vfc_end = vfc = skin->vertices(*fit);
            do
            {
                const int idx = (*vfc).idx();
                fbxmesh->AddPolygon(idx);
            }
            while (++vfc != vfc_end);
            fbxmesh->EndPolygon();
        }
    }
    else //if we do not have vertices, we can stop here
    {
        return false;
    }



    //create at least one layer
    FbxLayer *layer0 = fbxmesh->GetLayer(0);
    if(layer0 == NULL) {
        fbxmesh->CreateLayer();
        layer0 = fbxmesh->GetLayer(0);
    }

    //if vnormals, add vertex normals to layer
    if (skin->v_normals_)
    {
        FbxLayerElementNormal* fbxnormals = FbxLayerElementNormal::Create(fbxmesh, "vnormals");
        fbxnormals->SetMappingMode(FbxLayerElement::eByControlPoint);
        fbxnormals->SetReferenceMode(FbxLayerElement::eDirect);

        for (vit = skin->vertices_begin(); vit != skin->vertices_end(); ++vit)
        {
            const Normal& n = skin->v_normals_[*vit];
            fbxnormals->GetDirectArray().Add(FbxVector4(n[0],n[1],n[2]));
        }
        layer0->SetNormals(fbxnormals);
    }

    //if hnormals, add halfedge based normals to layer
    if (skin->h_normals_)
    {
        FbxLayerElementNormal* fbxnormals = FbxLayerElementNormal::Create(fbxmesh, "hnormals");
        fbxnormals->SetMappingMode(FbxLayerElement::eByPolygonVertex);
        fbxnormals->SetReferenceMode(FbxLayerElement::eDirect);

        Surface_mesh::Halfedge_around_face_circulator hfc, hfc_end;

        for (fit = skin->faces_begin(); fit != skin->faces_end(); ++fit)
        {
            hfc = hfc_end = skin->halfedges(*fit);
            do
            {
                const Normal& n = skin->h_normals_[*hfc];
                fbxnormals->GetDirectArray().Add(FbxVector4(n[0],n[1],n[2]));
            }
            while (++hfc != hfc_end);
        }

        layer0->SetNormals(fbxnormals);
    }

    //if vcolors, add vertex colors to layer
    if (skin->v_colors_)
    {
        FbxLayerElementVertexColor* fbxcolors = FbxLayerElementVertexColor::Create(fbxmesh, "vcolors");
        fbxcolors->SetMappingMode(FbxLayerElement::eByControlPoint);
        fbxcolors->SetReferenceMode(FbxLayerElement::eDirect);

        for (vit = skin->vertices_begin(); vit != skin->vertices_end(); ++vit)
        {
            const Color& c = skin->v_colors_[*vit];
            fbxcolors->GetDirectArray().Add(FbxVector4(c[0],c[1],c[2]));
        }
        layer0->SetVertexColors(fbxcolors);
    }

    //if we texture coords, add them to layer
    if (skin->h_texcoords_)
    {
        FbxLayerElementUV* fbxuv = FbxLayerElementUV::Create(fbxmesh, "htexcoords");
        fbxuv->SetMappingMode(FbxLayerElement::eByPolygonVertex);
        fbxuv->SetReferenceMode(FbxLayerElement::eDirect);

        Surface_mesh::Halfedge_around_face_circulator hfc, hfc_end;

        for (fit = skin->faces_begin(); fit != skin->faces_end(); ++fit)
        {
            hfc = hfc_end = skin->halfedges(*fit);
            do
            {
                const Texture_coordinate& tc = skin->h_texcoords_[*hfc];
                fbxuv->GetDirectArray().Add(FbxVector2(tc[0],tc[1]));
            }
            while (++hfc != hfc_end);
        }

        layer0->SetUVs(fbxuv);
    }

    Blendshapes* blendshapes = NULL;
    for (size_t b=0; b < character->blendshapes().size(); ++b)
    {
        Blendshapes* bs = character->blendshapes()[b];
        if (bs->base() == skin)
        {
            blendshapes = bs;
            break;
        }
    }

    if (blendshapes != NULL)
    {
        tmp_str = meshid + "_blendshapes";
        FbxBlendShape* fbxblendshape = FbxBlendShape::Create(fbxscene_, tmp_str.c_str());
        FbxBlendShapeChannel* fbxbschannel;


        fbxmesh->AddDeformer(fbxblendshape);

        FbxShape* fbxtargetshape;
        FbxLayer* fbxlayer;

        Surface_mesh::Mesh_property<std::string> target_id;

        for (size_t b=0; b < blendshapes->targets().size(); ++b)
        {
            Blendshapes::Target_shape* target = blendshapes->targets()[b];
            target_id = target->get_mesh_property<std::string>("m:id");
            if (target_id && target->vertices_)
            {
                fbxtargetshape = FbxShape::Create(fbxscene_, target_id[0].c_str());

                fbxbschannel = FbxBlendShapeChannel::Create(fbxscene_, (target_id[0] + "_bs_channel").c_str());
                fbxblendshape->AddBlendShapeChannel(fbxbschannel);

                fbxtargetshape->InitControlPoints(target->vertices_.vector().size());
                FbxVector4* cp = fbxtargetshape->GetControlPoints();

                for (size_t i=0; i < target->vertices_.vector().size(); ++i)
                {
                    const Point &p = target->vertices_.vector()[i];
                    cp[i] = FbxVector4(p[0],p[1],p[2]);
                }

                fbxlayer = fbxtargetshape->GetLayer(0);
                if (fbxlayer == NULL)
                {
                    fbxtargetshape->CreateLayer();
                    fbxlayer = fbxtargetshape->GetLayer(0);
                }

                if (target->v_normals_)
                {
                    FbxLayerElementNormal* fbxnormals = FbxLayerElementNormal::Create(fbxmesh, "vnormals");
                    fbxnormals->SetMappingMode(FbxLayerElement::eByControlPoint);
                    fbxnormals->SetReferenceMode(FbxLayerElement::eDirect);

                    for (size_t i=0; i < target->v_normals_.vector().size(); ++i)
                    {
                        const Normal& n = target->v_normals_.vector()[i];
                        fbxnormals->GetDirectArray().Add(FbxVector4(n[0],n[1],n[2]));
                    }
                    fbxlayer->SetNormals(fbxnormals);
                }

                if (target->h_normals_)
                {
                    FbxLayerElementNormal* fbxnormals = FbxLayerElementNormal::Create(fbxmesh, "hnormals");
                    fbxnormals->SetMappingMode(FbxLayerElement::eByPolygonVertex);
                    fbxnormals->SetReferenceMode(FbxLayerElement::eDirect);

                    Surface_mesh::Halfedge_around_face_circulator hfc, hfc_end;

                    for (fit = skin->faces_begin(); fit != skin->faces_end(); ++fit)
                    {
                        hfc = hfc_end = skin->halfedges(*fit);
                        do
                        {
                            const Normal& n = target->h_normals_[*hfc];
                            fbxnormals->GetDirectArray().Add(FbxVector4(n[0],n[1],n[2]));
                        }
                        while (++hfc != hfc_end);
                    }

                    fbxlayer->SetNormals(fbxnormals);
                }

                fbxbschannel->AddTargetShape(fbxtargetshape);
            }
        }
    }


    if (skin->v_depends1_ && skin->v_depends2_ && skin->v_weights1_ && skin->v_weights2_)
    {
        const Skeleton& skeleton = character->skeleton();

        tmp_str = meshid + "_skindeformer";
        FbxSkin* fbxskin = FbxSkin::Create(fbxscene_, tmp_str.c_str());
        fbxskin->SetGeometry(fbxmesh);

        FbxCluster* cluster;

        FbxAMatrix global_mesh_transform = fbxmeshnode->EvaluateGlobalTransform();

        fbxskin->SetSkinningType(FbxSkin::eLinear);

        std::map<size_t, FbxCluster*> used_clusters;
        std::map<size_t, FbxCluster*>::iterator cmit;


        for (vit = skin->vertices_begin(); vit != skin->vertices_end(); ++vit)
        {
            const Vec4f& d1 = skin->v_depends1_[*vit];
            const Vec4f& d2 = skin->v_depends2_[*vit];
            const Vec4f& w1 = skin->v_weights1_[*vit];
            const Vec4f& w2 = skin->v_weights2_[*vit];

            for (int k=0; k < 4; ++k)
            {
                if (w1[k] > 0.000000001f)
                {
                    cmit = used_clusters.find((size_t) d1[k]);

                    if (cmit != used_clusters.end())
                    {
                        cluster = cmit->second;
                    }
                    else
                    {
                        const Joint* joint = skeleton.joints_[(size_t) d1[k]];
                        FbxNode* fbxjointnode = fbxscene_->GetRootNode()->FindChild(joint->get_name().c_str());
                        if (fbxjointnode != NULL)
                        {
                            cluster = FbxCluster::Create(fbxscene_, "");
                            cluster->SetLink(fbxjointnode);
                            cluster->SetLinkMode(FbxCluster::eNormalize);
                            cluster->SetTransformMatrix(global_mesh_transform);
                            cluster->SetTransformLinkMatrix(fbxjointnode->EvaluateGlobalTransform());
                            fbxskin->AddCluster(cluster);
                            used_clusters[(size_t) d1[k]] = cluster;
                            add_node_rec(fbxjointnode);
                        }
                        else
                        {
                            return false;
                        }
                    }
                    cluster->AddControlPointIndex((*vit).idx(), (double) w1[k]);
                }
                if (w2[k] > 0.000000001f)
                {
                    cmit = used_clusters.find((size_t) d2[k]);

                    if (cmit != used_clusters.end())
                    {
                        cluster = cmit->second;
                    }
                    else
                    {
                        const Joint* joint = skeleton.joints_[(size_t) d2[k]];
                        FbxNode* fbxjointnode = fbxscene_->GetRootNode()->FindChild(joint->get_name().c_str());
                        if (fbxjointnode != NULL)
                        {
                            cluster = FbxCluster::Create(fbxscene_, "");
                            cluster->SetLink(fbxjointnode);
                            cluster->SetLinkMode(FbxCluster::eNormalize);
                            cluster->SetTransformMatrix(global_mesh_transform);
                            cluster->SetTransformLinkMatrix(fbxjointnode->EvaluateGlobalTransform());
                            fbxskin->AddCluster(cluster);
                            used_clusters[(size_t) d2[k]] = cluster;
                            add_node_rec(fbxjointnode);
                        }
                        else
                        {
                            return false;
                        }
                    }
                    cluster->AddControlPointIndex((*vit).idx(), (double) w2[k]);
                }
            }
        }

        add_node_rec(fbxmeshnode);


        fbxmesh->AddDeformer(fbxskin);
    }

    return true;
}

void FBX_file::add_node_rec(FbxNode *node)
{
    if (node)
    {
        add_node_rec(node->GetParent());
        if (fbxnode_array_.Find(node) == -1)
        {
            // Node not in the list, add it
            fbxnode_array_.Add(node);
        }
    }
}

FbxSurfacePhong* FBX_file::set_phong_material(FbxNode *fbxmeshnode, const Surface_mesh_skin *skin, const std::string &meshid)
{
    Surface_mesh::Mesh_property<Vec3f> m_ambient_color  = skin->get_mesh_property<Vec3f>("m:ambient_color");
    Surface_mesh::Mesh_property<Vec3f> m_diffuse_color  = skin->get_mesh_property<Vec3f>("m:diffuse_color");
    Surface_mesh::Mesh_property<Vec4f> m_specular_color = skin->get_mesh_property<Vec4f>("m:specular_color");
    Surface_mesh::Mesh_property<float> m_alpha          = skin->get_mesh_property<float>("m:alpha");

    FbxSurfacePhong* fbxphong_material = FbxSurfacePhong::Create(fbxscene_, (meshid + "_phong_shading").c_str());
    if (m_ambient_color)
    {
        const Vec3f& a = m_ambient_color[0];
        fbxphong_material->Ambient.Set(FbxDouble3(a[0],a[1],a[2]));
    }
    else
    {
        fbxphong_material->Ambient.Set(FbxDouble3(0.0,0.0,0.0));
    }

    if (m_diffuse_color)
    {
        const Vec3f& a = m_diffuse_color[0];
        fbxphong_material->Diffuse.Set(FbxDouble3(a[0],a[1],a[2]));
    }
    else
    {
        fbxphong_material->Diffuse.Set(FbxDouble3(1.0,1.0,1.0));
        fbxphong_material->DiffuseFactor.Set(FbxDouble(0.5));
    }

    if (m_specular_color)
    {
        const Vec4f& a = m_specular_color[0];
        fbxphong_material->Specular.Set(FbxDouble3(a[0],a[1],a[2]));
        fbxphong_material->Shininess.Set(FbxDouble(a[3]));
    }
    else
    {
        fbxphong_material->Specular.Set(FbxDouble3(0.1,0.1,0.1));
        fbxphong_material->Shininess.Set(FbxDouble(32.0));
    }

    if (m_alpha)
    {
        const float &alpha = m_alpha[0];
        fbxphong_material->TransparencyFactor.Set((double) (1.0f-alpha));
    }


    fbxphong_material->Emissive.Set(FbxDouble3(0.0,0.0,0.0));

    fbxmeshnode->AddMaterial(fbxphong_material);

    return fbxphong_material;
}

void FBX_file::set_fbxtextures(FbxSurfacePhong *fbxmaterial, const Surface_mesh_skin *skin)
{
    Surface_mesh::Mesh_property<std::string> m_texture_name      = skin->get_mesh_property<std::string>("m:texturename");
    Surface_mesh::Mesh_property<std::string> m_texture_name_nm   = skin->get_mesh_property<std::string>("m:texturename_nm");
    Surface_mesh::Mesh_property<std::string> m_texture_name_spec = skin->get_mesh_property<std::string>("m:texturename_spec");

    FbxFileTexture* fbxtexture;
    std::map<std::string, FbxFileTexture*>::iterator mit;

    if (m_texture_name)
    {
        mit = fbxtextures_.find(m_texture_name[0]);

        if (mit != fbxtextures_.end())
        {
            fbxtexture = mit->second;
        }
        else
        {
            std::string fname = extract_filename(m_texture_name[0]);

            fbxtexture = FbxFileTexture::Create(fbxscene_, "diffuse_map");
            fbxtexture->SetFileName(fname.c_str());
            fbxtextures_[m_texture_name[0]] = fbxtexture;
        }

        fbxmaterial->Diffuse.ConnectSrcObject(fbxtexture);
    }


    if (m_texture_name_nm)
    {
        mit = fbxtextures_.find(m_texture_name_nm[0]);

        if (mit != fbxtextures_.end())
        {
            fbxtexture = mit->second;
        }
        else
        {
            std::string fname = extract_filename(m_texture_name_nm[0]);

            fbxtexture = FbxFileTexture::Create(fbxscene_, "normal_map");
            fbxtexture->SetFileName(fname.c_str());
            fbxtextures_[m_texture_name_nm[0]] = fbxtexture;
        }

        fbxmaterial->NormalMap.ConnectSrcObject(fbxtexture);
    }
    else if (m_texture_name_nm)
    {
        std::cerr << "FBX_file::set_textures(): Normalmaps are disabled for now, since they are not displayed correctly in Maya." << std::endl;
    }


    if (m_texture_name_spec)
    {
        mit = fbxtextures_.find(m_texture_name_spec[0]);

        if (mit != fbxtextures_.end())
        {
            fbxtexture = mit->second;
        }
        else
        {
            std::string fname = extract_filename(m_texture_name_spec[0]);

            fbxtexture = FbxFileTexture::Create(fbxscene_, "speclular_map");
            fbxtexture->SetFileName(fname.c_str());
            fbxtextures_[m_texture_name_spec[0]] = fbxtexture;
        }

        fbxmaterial->Specular.ConnectSrcObject(fbxtexture);
    }

}

void FBX_file::fbxmat_to_mat(const FbxAMatrix &fbxmat, Mat4f &mat)
{
    for (int i=0; i < 4; ++i)
    {
        for (int j=0; j < 4; ++j)
        {
            mat(i,j) = fbxmat.Buffer()[j][i];
        }
    }
}

void FBX_file::mat_to_fbxmat(const Mat4f &mat, FbxAMatrix &fbxmat)
{
    for (int i=0; i < 4; ++i)
    {
        for (int j=0; j < 4; ++j)
        {
            fbxmat.Buffer()[i][j] = mat(j,i);
        }
    }
}

std::string FBX_file::extract_filepath(const std::string &filename)
{
    size_t pos = filename.rfind('/');
    if (pos == std::string::npos)
        pos = filename.rfind('\\');

    if (pos != std::string::npos)
        return filename.substr(0,pos+1);
    else
        return "";

}

std::string FBX_file::extract_filename(const std::string &filename)
{
    size_t pos = filename.rfind('/');
    if (pos == std::string::npos)
        pos = filename.rfind('\\');

    if (pos != std::string::npos)
        return filename.substr(pos+1);
    else
        return filename;
}

}
}
