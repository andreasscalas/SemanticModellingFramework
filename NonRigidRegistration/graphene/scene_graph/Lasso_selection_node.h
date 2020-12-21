//=============================================================================
#ifndef GRAPHENE_LASSO_SELECTION_NODE_H
#define GRAPHENE_LASSO_SELECTION_NODE_H
//=============================================================================


#include <graphene/scene_graph/Base_node.h>
#include <graphene/gl/Shader.h>

#include <vector>


//=============================================================================


namespace graphene {
namespace scene_graph {


//=============================================================================


/// Draws a selection lasso on the screen
class Lasso_selection_node : public Base_node
{

public:

    Lasso_selection_node(const std::string& _name = "Lasso Selection");
    virtual ~Lasso_selection_node();

    void         draw(gl::GL_state* _gl);
    Bounding_box bbox() const;
    void         push_back(const Vec3f& p);

public:

    std::vector<Vec3f> points_;


private:

    void upload_to_gl();


private:

    GLuint vertexarrayobject_;
    GLuint vertexbuffer_;
};


//=============================================================================
} // scene_graph
} // graphene
//=============================================================================
#endif // GRAPHENE_LASSO_SELECTION_NDOE_H
//=============================================================================
