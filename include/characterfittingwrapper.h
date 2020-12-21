#ifndef CHARACTERFITTINGWRAPPER_H
#define CHARACTERFITTINGWRAPPER_H

#include <io/Loader.h>
#include <extendedtrimesh.h>
#include <vector>

class CharacterFittingWrapper
{
public:
    CharacterFittingWrapper();
    bool run();

    ExtendedTrimesh *getTemplate_mesh() const;
    void setTemplate_mesh(ExtendedTrimesh *value);
    ExtendedTrimesh *getFragment_mesh() const;
    void setFragment_mesh(ExtendedTrimesh *value);


    std::vector<std::pair<int, int> > getCorrespondences_indices() const;
    void setCorrespondences_indices(const std::vector<std::pair<int, int> > &value);

private:
    ExtendedTrimesh* template_mesh;
    ExtendedTrimesh* fragment_mesh;
    scene_graph::Surface_mesh_node* template_model_;
    scene_graph::Point_set_node*    pointset_fragment_;
    std::vector<std::pair<int, int> > correspondences_indices;

    bool setup();
    bool save_correspondences_indices();

};

#endif // CHARACTERFITTINGWRAPPER_H
