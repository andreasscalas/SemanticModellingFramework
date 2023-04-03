#ifndef ANNOTATIONSFITTING_H
#define ANNOTATIONSFITTING_H

#include <extendedtrimesh.h>
#include <mainwindow.h>
#include <shapeopbarycentriccoordinates.h>

class NonRigidFitting
{
public:
    NonRigidFitting();
    NonRigidFitting(ExtendedTrimesh* template_mesh, ExtendedTrimesh* mesh_to_fit);

    void startFitting();
    void startFitting1();

    ExtendedTrimesh *getTemplateMesh() const;
    void setTemplateMesh(ExtendedTrimesh *value);
    ExtendedTrimesh *getMeshToFit() const;
    void setMeshToFit(ExtendedTrimesh *value);

    ExtendedTrimesh *getCageMesh() const;
    void setCageMesh(ExtendedTrimesh *newCage_mesh);

    ShapeOpBarycentricCoordinates *getCoordinates() const;
    void setCoordinates(ShapeOpBarycentricCoordinates *newCoordinates);

    ShapeOp::MatrixX3 *getSoTemplatePoints() const;
    void setSoTemplatePoints(ShapeOp::MatrixX3 *newSo_template_points);

    ShapeOp::MatrixX3 *getSoCagePoints() const;
    void setSoCagePoints(ShapeOp::MatrixX3 *newSo_cage_points);

    MainWindow *getContext() const;
    void setContext(MainWindow *newContext);

private:
    ExtendedTrimesh* template_mesh;
    ExtendedTrimesh* cage_mesh;
    ExtendedTrimesh* mesh_to_fit;
    ShapeOp::MatrixX3* so_template_points;
    ShapeOp::MatrixX3* so_cage_points;
    ShapeOpBarycentricCoordinates* coordinates;
    MainWindow* context;
};

#endif // ANNOTATIONSFITTING_H
