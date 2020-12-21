#ifndef ANNOTATIONSFITTING_H
#define ANNOTATIONSFITTING_H

#include <extendedtrimesh.h>
#include <annotation.h>

class NonRigidFitting
{
public:
    NonRigidFitting();
    NonRigidFitting(ExtendedTrimesh* template_mesh, ExtendedTrimesh* mesh_to_fit);

    void startFitting();

    ExtendedTrimesh *getTemplateMesh() const;
    void setTemplateMesh(ExtendedTrimesh *value);
    ExtendedTrimesh *getMeshToFit() const;
    void setMeshToFit(ExtendedTrimesh *value);

private:
    ExtendedTrimesh* template_mesh;
    ExtendedTrimesh* mesh_to_fit;

};

#endif // ANNOTATIONSFITTING_H
