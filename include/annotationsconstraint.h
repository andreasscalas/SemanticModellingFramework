#ifndef ANNOTATIONSCONSTRAINT_H
#define ANNOTATIONSCONSTRAINT_H

#include <vector>
#include <annotation.h>
#include <pointannotation.h>
#include <lineannotation.h>
#include <surfaceannotation.h>
#include <Constraint.h>
#include <constraintsolver.h>
#include <semanticconstraint.h>
#include <annotationsrelationship.h>
#include <string>

//enum class SemanticConstraintType{
//    PointCloseness,
//    PointLaplacian,
//    PointLaplacianDisplacement,
//    LineLinearity,
//    LinePlanarity,
//    LineCircularity,
//    LineRectangularity,
//    LineAngle,
//    LineLength,
//    LineSafeguard,
//    SurfaceArea,
//    SurfacePlanarity,
//    SurfaceRigidity,
//    SurfaceSimilarity,
//    SurfaceCurvature,
//    SurfaceLength,
//    SurfaceWidth,
//    SurfaceOrientation,
//    SurfaceRegressionPlaneOrientation,
//    PointsEdgeStrain,
//    PointsTriangleStrain,
//    PointsTetrahedronStrain,
//    PointsArea,
//    PointsVolume,
//    PointsBending,
//    PointsLine,
//    PointsPlane,
//    PointsCircle,
//    PointsSphere,
//    PointsRigidity,
//    PointsSimilarity,
//    PointsRectangle,
//    PointsParallelogram,
//    PointsAngle,
//    LinesParallelism,
//    LinesPerpendicularity,
//    LinesSameLength,
//    LinesSymmetry,
//    LinesCoplanarity,
//    LinesSameLevel,
//    SurfacesCoplanarity,
//    SurfacesSameArea,
//    SurfacesRigidity,
//    SurfacesSameLength,
//    SurfacesSameWidth,
//    SurfacesSameOrientation,
//    SurfacesAngle,
//    SurfacesSameLevel,
//    SurfacesSymmetry
//};

class AnnotationsConstraint : virtual public AnnotationsRelationship//SemanticConstraint
{
public:
    AnnotationsConstraint();
    AnnotationsConstraint(AnnotationsRelationship*);

    virtual void constrain();

    std::vector<Annotation *> getAnnotations() const;
    void setAnnotations(const std::vector<Annotation *> &value);
    double getError();

    ShapeOp::MatrixX3 *getPoints() const;
    void setPoints(ShapeOp::MatrixX3 *value);

    std::vector<std::shared_ptr<ShapeOp::Constraint> > getConstraints() const;
    void setConstraints(const std::vector<std::shared_ptr<ShapeOp::Constraint> > &value);

    ConstraintSolver *getSolver() const;
    void setSolver(ConstraintSolver *value);

protected:
    ShapeOp::MatrixX3* points;
    std::vector<std::shared_ptr<ShapeOp::Constraint> > constraints;
    ConstraintSolver* solver;

    void constrainSelf();
    void constrainSet();
    std::vector<int> getInvolvedVerticesIDs(Annotation* annotation);
};

#endif // ANNOTATIONSCONSTRAINT_H
