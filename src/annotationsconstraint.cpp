#include "annotationsconstraint.h"
#include <utilities.h>
#include <pointannotation.h>
#include <lineannotation.h>
#include <surfaceannotation.h>

AnnotationsConstraint::AnnotationsConstraint()
{

}

AnnotationsConstraint::AnnotationsConstraint(AnnotationsRelationship *p) : AnnotationsRelationship(*p)
{

}

std::vector<Annotation *> AnnotationsConstraint::getAnnotations() const
{
    return annotations;
}

void AnnotationsConstraint::setAnnotations(const std::vector<Annotation *> &value)
{
    annotations = value;
}

void AnnotationsConstraint::constrain()
{
    if(annotations.size() > 0){
        if(annotations.size() == 1){
            constrainSelf();
        }else {
            constrainSet();
        }
    } else
        throw("You have to define at least one annotation");
}

void AnnotationsConstraint::checkConstraint()
{
}

void AnnotationsConstraint::print(std::ostream &os)
{
    AnnotationsRelationship::print(os);
    double error = 0;
    for(unsigned int i = 0; i < constraints.size(); i++){
        double cError = solver->getResidualError(constraints[i]) / constraints[i]->getWeight();
        error += cError;
    }

    os << "Average error: " << error / constraints.size() << std::endl;
    os << "Total error: " << error << std::endl << std::flush;
}

void AnnotationsConstraint::constrainSelf()
{
    std::shared_ptr<ShapeOp::Constraint> c;
    if(this->type.compare("Point closeness") == 0)
    {
        assert(annotations.size() == 1 && annotations[0]->getType() == AnnotationType::Point && annotations[0]->getInvolvedVertices().size() == 1);
        c = ShapeOp::Constraint::shapeConstraintFactory("Closeness", getInvolvedVerticesIDs(annotations[0]), weight, points->transpose());
        constraints.push_back(c);
    } else if(this->type.compare("Point laplacian") == 0)
    {
        assert(annotations.size() == 1 && annotations[0]->getType() == AnnotationType::Point && annotations[0]->getInvolvedVertices().size() == 1);
        PointAnnotation* annotation = dynamic_cast<PointAnnotation*>(annotations[0]);
        std::vector<int> involvedIds = getInvolvedVerticesIDs(annotation);
        std::vector<IMATI_STL::Vertex*> involved = annotation->getPoints();
        for(IMATI_STL::Node* n = involved[0]->VV()->head(); n != nullptr; n = n->next()){
            IMATI_STL::Vertex* v = static_cast<IMATI_STL::Vertex*>(n->data);
            involvedIds.push_back(static_cast<int>(annotation->getMesh()->getPointId(v)));
        }
        c = ShapeOp::Constraint::shapeConstraintFactory("Laplacian", involvedIds, weight, points->transpose());
        constraints.push_back(c);
    } else if(this->type.compare("Point laplacian displacement") == 0)
    {
        assert(annotations.size() == 1 && annotations[0]->getType() == AnnotationType::Point && annotations[0]->getInvolvedVertices().size() == 1);
        PointAnnotation* annotation = dynamic_cast<PointAnnotation*>(annotations[0]);
        std::vector<int> involvedIds = getInvolvedVerticesIDs(annotation);
        std::vector<IMATI_STL::Vertex*> involved = annotation->getPoints();
        for(IMATI_STL::Node* n = involved[0]->VV()->head(); n != nullptr; n = n->next()){
            IMATI_STL::Vertex* v = static_cast<IMATI_STL::Vertex*>(n->data);
            involvedIds.push_back(static_cast<int>(annotation->getMesh()->getPointId(v)));
        }
        c = ShapeOp::Constraint::shapeConstraintFactory("LaplacianDisplacement", involvedIds, weight, points->transpose());
        constraints.push_back(c);
    } else if(this->type.compare("Line linearity") == 0)
    {
        assert(annotations.size() == 1 && annotations[0]->getType() == AnnotationType::Line);
        LineAnnotation* annotation = dynamic_cast<LineAnnotation*>(annotations[0]);
        std::vector<int> involvedIds = getInvolvedVerticesIDs(annotation);
        c = ShapeOp::Constraint::shapeConstraintFactory("Line", involvedIds, weight, points->transpose());
        constraints.push_back(c);
    } else if(this->type.compare("Line planarity") == 0)
    {
        assert(annotations.size() == 1 && annotations[0]->getType() == AnnotationType::Line);
        LineAnnotation* annotation = dynamic_cast<LineAnnotation*>(annotations[0]);
        std::vector<int> involvedIds = getInvolvedVerticesIDs(annotation);
        c = ShapeOp::Constraint::shapeConstraintFactory("Plane", involvedIds, weight, points->transpose());
        constraints.push_back(c);
    } else if(this->type.compare("Line circularity") == 0)
    {
        assert(annotations.size() == 1 && annotations[0]->getType() == AnnotationType::Line);
        LineAnnotation* annotation = dynamic_cast<LineAnnotation*>(annotations[0]);
        std::vector<int> involvedIds = getInvolvedVerticesIDs(annotation);
        c = ShapeOp::Constraint::shapeConstraintFactory("Circle", involvedIds, weight, points->transpose());
        constraints.push_back(c);
    } else if(this->type.compare("Line rectangularity") == 0)
    {
        assert(annotations.size() == 1 && annotations[0]->getType() == AnnotationType::Line);
        LineAnnotation* annotation = dynamic_cast<LineAnnotation*>(annotations[0]);
        std::vector<int> involvedIds = getInvolvedVerticesIDs(annotation);
        c = ShapeOp::Constraint::shapeConstraintFactory("RectangleFit", involvedIds, weight, points->transpose());
        constraints.push_back(c);
    } else if(this->type.compare("Line angle") == 0)
    {
        assert(annotations.size() == 1 && annotations[0]->getType() == AnnotationType::Line &&
               minValue >= 0.0 && minValue < M_PI && maxValue >= 0.0 && maxValue < M_PI && minValue <= maxValue);
        LineAnnotation* annotation = dynamic_cast<LineAnnotation*>(annotations[0]);
        std::vector<int> involvedIds = getInvolvedVerticesIDs(annotation);
        std::vector<IMATI_STL::Vertex*> involvedVertices = annotation->getInvolvedVertices();
        assert(involvedVertices.size() >= 3);
        for(unsigned int i = 2; i < involvedIds.size(); i++){
            std::vector<int> ithInvolvedIds = {involvedIds[i - 2], involvedIds[i - 1], involvedIds[i]};
            c = std::make_shared<ShapeOp::AngleConstraint>(involvedIds, weight, points->transpose(), minValue, maxValue);
            constraints.push_back(c);
        }
    } else if(this->type.compare("Line length") == 0)
    {
        assert(annotations.size() == 1 && annotations[0]->getType() == AnnotationType::Line &&
               minValue >= 0.0 && minValue < 9999.9 && maxValue >= 0.0 && maxValue < 9999.9 && minValue <= maxValue);
        LineAnnotation* annotation = dynamic_cast<LineAnnotation*>(annotations[0]);
        std::vector<int> involvedIds = getInvolvedVerticesIDs(annotation);
        assert(involvedIds.size() >= 2);
        c = std::make_shared<ShapeOp::PolylineLengthConstraint>(involvedIds, weight, points->transpose(), minValue, maxValue);
        constraints.push_back(c);
    } else if(this->type.compare("Line safeguard") == 0)
    {
        assert(annotations.size() == 1 && annotations[0]->getType() == AnnotationType::Line);
        LineAnnotation* annotation = dynamic_cast<LineAnnotation*>(annotations[0]);
        std::vector<int> involvedIds = getInvolvedVerticesIDs(annotation);
        assert(involvedIds.size() >= 2);
        c = ShapeOp::Constraint::shapeConstraintFactory("Rigid", involvedIds, weight, points->transpose());
        constraints.push_back(c);
    } else if(this->type.compare("Line keeping") == 0)
    {
        assert(annotations.size() == 1 && annotations[0]->getType() == AnnotationType::Line);
        LineAnnotation* annotation = dynamic_cast<LineAnnotation*>(annotations[0]);
        std::vector<int> involvedIds = getInvolvedVerticesIDs(annotation);
        for(unsigned j = 0; j < involvedIds.size(); j++)
        {
            std::vector<int> point = {involvedIds[j]};
            c = ShapeOp::Constraint::shapeConstraintFactory("Closeness", point, weight, points->transpose());
            constraints.push_back(c);
        }
    } else if(this->type.compare("Surface area") == 0)
    {
        ;
    } else if(this->type.compare("Surfaces same measure") == 0)
    {
        ; //Based on min/max length
    } else if(this->type.compare("Surface planarity") == 0)
    {
        assert(annotations.size() == 1 && annotations[0]->getType() == AnnotationType::Surface);
        SurfaceAnnotation* annotation = dynamic_cast<SurfaceAnnotation*>(annotations[0]);
        std::vector<int> involvedIds = getInvolvedVerticesIDs(annotation);
        c = ShapeOp::Constraint::shapeConstraintFactory("Plane", involvedIds, weight, points->transpose());
        constraints.push_back(c);
    } else if(this->type.compare("Surface rigidity") == 0)
    {
        assert(annotations.size() == 1 && annotations[0]->getType() == AnnotationType::Surface);
        SurfaceAnnotation* annotation = dynamic_cast<SurfaceAnnotation*>(annotations[0]);
        std::vector<int> involvedIds = getInvolvedVerticesIDs(annotation);
        c = ShapeOp::Constraint::shapeConstraintFactory("Rigid", involvedIds, weight, points->transpose());
        constraints.push_back(c);
    } else if(this->type.compare("Surface similarity") == 0)
    {
        assert(annotations.size() == 1 && annotations[0]->getType() == AnnotationType::Surface);
        SurfaceAnnotation* annotation = dynamic_cast<SurfaceAnnotation*>(annotations[0]);
        std::vector<int> involvedIds = getInvolvedVerticesIDs(annotation);
        c = ShapeOp::Constraint::shapeConstraintFactory("Similarity", involvedIds, weight, points->transpose());
        constraints.push_back(c);
    } else if(this->type.compare("Surface curvature") == 0)
    {
        ;
    } else if(this->type.compare("Surface length") == 0)
    {
        ;
    } else if(this->type.compare("Surface width") == 0)
    {
        ;
    } else if(this->type.compare("Surface orientation") == 0)
    {
        assert(annotations.size() > 0 && annotations[0]->getType() == AnnotationType::Surface);
        SurfaceAnnotation* annotation = dynamic_cast<SurfaceAnnotation*>(annotations[0]);
        std::vector<int> involvedIds = getInvolvedVerticesIDs(annotation);
        c = ShapeOp::Constraint::shapeConstraintFactory("Orientation", involvedIds, weight, points->transpose());

        std::vector<int> annotationsInvolvedIDs = getInvolvedVerticesIDs(annotations[0]);
        Eigen::Matrix3Xd annotationPoints;
        annotationPoints.resize(3, static_cast<int>(annotationsInvolvedIDs.size()));
        for(unsigned j = 0; j < annotationsInvolvedIDs.size(); j++)
            annotationPoints.col(j) = points->row(annotationsInvolvedIDs[j]);

        std::pair<Eigen::Vector3d, Eigen::Vector3d> plane = Utilities::planarRegression(annotationPoints);
        static_cast<ShapeOp::OrientationConstraint*>(c.get())->setOrientation(plane.second);
        constraints.push_back(c);
    } else if(this->type.compare("Surface regression plane orientation") == 0)
    {
        assert(annotations.size() > 0 && annotations[0]->getType() == AnnotationType::Surface);
        SurfaceAnnotation* annotation = dynamic_cast<SurfaceAnnotation*>(annotations[0]);
        std::vector<int> involvedIds = getInvolvedVerticesIDs(annotation);
        c = ShapeOp::Constraint::shapeConstraintFactory("RegressionPlaneOrientation", involvedIds, weight, points->transpose());

        std::vector<int> annotationsInvolvedIDs = getInvolvedVerticesIDs(annotations[0]);
        Eigen::Matrix3Xd annotationPoints;

        annotationPoints.resize(3, static_cast<int>(annotationsInvolvedIDs.size()));
        for(unsigned j = 0; j < annotationsInvolvedIDs.size(); j++)
            annotationPoints.col(j) = points->row(annotationsInvolvedIDs[j]);

        std::pair<Eigen::Vector3d, Eigen::Vector3d> plane = Utilities::planarRegression(annotationPoints);
        static_cast<ShapeOp::RegressionPlaneOrientationConstraint*>(c.get())->setOrientation(plane.second);
        constraints.push_back(c);
    } else if(this->type.compare("Surface keeping") == 0)
    {
        assert(annotations.size() == 1 && annotations[0]->getType() == AnnotationType::Surface);
        SurfaceAnnotation* annotation = dynamic_cast<SurfaceAnnotation*>(annotations[0]);
        std::vector<int> involvedIds = getInvolvedVerticesIDs(annotation);

        for(unsigned j = 0; j < involvedIds.size(); j++)
        {
            std::vector<int> point = {involvedIds[j]};
            c = ShapeOp::Constraint::shapeConstraintFactory("Closeness", point, weight, points->transpose());
            constraints.push_back(c);
        }

    } else
        throw("This type of constraint can not be applied to singlets");

}

void AnnotationsConstraint::constrainSet()
{

    std::shared_ptr<ShapeOp::Constraint> c;
    if(this->type.compare("Points edge strain") == 0)
    {
        assert(annotations.size() == 2 &&
               annotations[0]->getType() == AnnotationType::Point &&
               annotations[1]->getType() == AnnotationType::Point );

        std::vector<int> involvedIDs;
        std::vector<int> a1InvolvedIds = getInvolvedVerticesIDs(annotations[0]);
        std::vector<int> a2InvolvedIds = getInvolvedVerticesIDs(annotations[1]);
        involvedIDs.insert(involvedIDs.end(), a1InvolvedIds.begin(), a1InvolvedIds.end());
        involvedIDs.insert(involvedIDs.end(), a2InvolvedIds.begin(), a2InvolvedIds.end());
        c = std::make_shared<ShapeOp::EdgeStrainConstraint>(involvedIDs, weight, points->transpose(), minValue, maxValue);
        constraints.push_back(c);
    } else if(this->type.compare("Points triangle strain") == 0)
    {
        assert(annotations.size() == 3 &&
             annotations[0]->getType() == AnnotationType::Point &&
             annotations[1]->getType() == AnnotationType::Point &&
             annotations[2]->getType() == AnnotationType::Point );

        std::vector<int> involvedIDs;
        std::vector<int> a1InvolvedIds = getInvolvedVerticesIDs(annotations[0]);
        std::vector<int> a2InvolvedIds = getInvolvedVerticesIDs(annotations[1]);
        std::vector<int> a3InvolvedIds = getInvolvedVerticesIDs(annotations[2]);
        involvedIDs.insert(involvedIDs.end(), a1InvolvedIds.begin(), a1InvolvedIds.end());
        involvedIDs.insert(involvedIDs.end(), a2InvolvedIds.begin(), a2InvolvedIds.end());
        involvedIDs.insert(involvedIDs.end(), a3InvolvedIds.begin(), a3InvolvedIds.end());
        c = std::make_shared<ShapeOp::TriangleStrainConstraint>(involvedIDs, weight, points->transpose(), minValue, maxValue);
        constraints.push_back(c);
    } else if(this->type.compare("Points tetrahedron strain") == 0)
    {
        assert(annotations.size() == 4 &&
               annotations[0]->getType() == AnnotationType::Point &&
               annotations[1]->getType() == AnnotationType::Point &&
               annotations[2]->getType() == AnnotationType::Point &&
               annotations[3]->getType() == AnnotationType::Point );

        std::vector<int> involvedIDs;
        std::vector<int> a1InvolvedIds = getInvolvedVerticesIDs(annotations[0]);
        std::vector<int> a2InvolvedIds = getInvolvedVerticesIDs(annotations[1]);
        std::vector<int> a3InvolvedIds = getInvolvedVerticesIDs(annotations[2]);
        std::vector<int> a4InvolvedIds = getInvolvedVerticesIDs(annotations[3]);
        involvedIDs.insert(involvedIDs.end(), a1InvolvedIds.begin(), a1InvolvedIds.end());
        involvedIDs.insert(involvedIDs.end(), a2InvolvedIds.begin(), a2InvolvedIds.end());
        involvedIDs.insert(involvedIDs.end(), a3InvolvedIds.begin(), a3InvolvedIds.end());
        involvedIDs.insert(involvedIDs.end(), a4InvolvedIds.begin(), a4InvolvedIds.end());
        c = std::make_shared<ShapeOp::TriangleStrainConstraint>(involvedIDs, weight, points->transpose(), minValue, maxValue);
        constraints.push_back(c);
    } else if(this->type.compare("Points area") == 0)
    {
        assert(annotations.size() == 3 &&
               annotations[0]->getType() == AnnotationType::Point &&
               annotations[1]->getType() == AnnotationType::Point &&
               annotations[2]->getType() == AnnotationType::Point );

        std::vector<int> involvedIDs;
        std::vector<int> a1InvolvedIds = getInvolvedVerticesIDs(annotations[0]);
        std::vector<int> a2InvolvedIds = getInvolvedVerticesIDs(annotations[1]);
        std::vector<int> a3InvolvedIds = getInvolvedVerticesIDs(annotations[2]);
        involvedIDs.insert(involvedIDs.end(), a1InvolvedIds.begin(), a1InvolvedIds.end());
        involvedIDs.insert(involvedIDs.end(), a2InvolvedIds.begin(), a2InvolvedIds.end());
        involvedIDs.insert(involvedIDs.end(), a3InvolvedIds.begin(), a3InvolvedIds.end());
        c = std::make_shared<ShapeOp::AreaConstraint>(involvedIDs, weight, points->transpose(),minValue, maxValue);
        constraints.push_back(c);
    } else if(this->type.compare("Points volume") == 0)
    {
        assert(annotations.size() == 4 &&
               annotations[0]->getType() == AnnotationType::Point &&
               annotations[1]->getType() == AnnotationType::Point &&
               annotations[2]->getType() == AnnotationType::Point &&
               annotations[3]->getType() == AnnotationType::Point );

        std::vector<int> involvedIDs;
        std::vector<int> a1InvolvedIds = getInvolvedVerticesIDs(annotations[0]);
        std::vector<int> a2InvolvedIds = getInvolvedVerticesIDs(annotations[1]);
        std::vector<int> a3InvolvedIds = getInvolvedVerticesIDs(annotations[2]);
        std::vector<int> a4InvolvedIds = getInvolvedVerticesIDs(annotations[3]);
        involvedIDs.insert(involvedIDs.end(), a1InvolvedIds.begin(), a1InvolvedIds.end());
        involvedIDs.insert(involvedIDs.end(), a2InvolvedIds.begin(), a2InvolvedIds.end());
        involvedIDs.insert(involvedIDs.end(), a3InvolvedIds.begin(), a3InvolvedIds.end());
        involvedIDs.insert(involvedIDs.end(), a4InvolvedIds.begin(), a4InvolvedIds.end());
        c = std::make_shared<ShapeOp::VolumeConstraint>(involvedIDs, weight, points->transpose(),minValue, maxValue);
        constraints.push_back(c);
    } else if(this->type.compare("Points line") == 0)
    {
        assert(annotations.size() >= 2);
        std::vector<int> involvedIDs;
        for (unsigned int i = 0; i < annotations.size(); i++) {
            assert(annotations[i]->getType() == AnnotationType::Point);
            std::vector<int> annotationInvolvedIds = getInvolvedVerticesIDs(annotations[i]);
            involvedIDs.insert(involvedIDs.end(), annotationInvolvedIds.begin(), annotationInvolvedIds.end());
        }
        c = ShapeOp::Constraint::shapeConstraintFactory("Line", involvedIDs, weight, points->transpose());
    } else if(this->type.compare("Points plane") == 0)
    {
        assert(annotations.size() >= 3);
        std::vector<int> involvedIDs;
        for (unsigned int i = 0; i < annotations.size(); i++) {
            assert(annotations[i]->getType() == AnnotationType::Point);
            std::vector<int> annotationInvolvedIds = getInvolvedVerticesIDs(annotations[i]);
            involvedIDs.insert(involvedIDs.end(), annotationInvolvedIds.begin(), annotationInvolvedIds.end());
        }
        c = ShapeOp::Constraint::shapeConstraintFactory("Plane", involvedIDs, weight, points->transpose());
    } else if(this->type.compare("Points circle") == 0)
    {
        assert(annotations.size() >= 3);
        std::vector<int> involvedIDs;
        for (unsigned int i = 0; i < annotations.size(); i++) {
            assert(annotations[i]->getType() == AnnotationType::Point);
            std::vector<int> annotationInvolvedIds = getInvolvedVerticesIDs(annotations[i]);
            involvedIDs.insert(involvedIDs.end(), annotationInvolvedIds.begin(), annotationInvolvedIds.end());
        }
        c = ShapeOp::Constraint::shapeConstraintFactory("Circle", involvedIDs, weight, points->transpose());
    } else if(this->type.compare("Points sphere") == 0)
    {
        assert(annotations.size() >= 4);
        std::vector<int> involvedIDs;
        for (unsigned int i = 0; i < annotations.size(); i++) {
            assert(annotations[i]->getType() == AnnotationType::Point);
            std::vector<int> annotationInvolvedIds = getInvolvedVerticesIDs(annotations[i]);
            involvedIDs.insert(involvedIDs.end(), annotationInvolvedIds.begin(), annotationInvolvedIds.end());
        }
        c = ShapeOp::Constraint::shapeConstraintFactory("Sphere", involvedIDs, weight, points->transpose());
    } else if(this->type.compare("Points rigidity") == 0)
    {
        assert(annotations.size() >= 1);
        std::vector<int> involvedIDs;
        for (unsigned int i = 0; i < annotations.size(); i++) {
            assert(annotations[i]->getType() == AnnotationType::Point);
            std::vector<int> annotationInvolvedIds = getInvolvedVerticesIDs(annotations[i]);
            involvedIDs.insert(involvedIDs.end(), annotationInvolvedIds.begin(), annotationInvolvedIds.end());
        }
        c = ShapeOp::Constraint::shapeConstraintFactory("Rigid", involvedIDs, weight, points->transpose());
    } else if(this->type.compare("Points similarity") == 0)
    {
        assert(annotations.size() >= 1);
        std::vector<int> involvedIDs;
        for (unsigned int i = 0; i < annotations.size(); i++) {
            assert(annotations[i]->getType() == AnnotationType::Point);
            std::vector<int> annotationInvolvedIds = getInvolvedVerticesIDs(annotations[i]);
            involvedIDs.insert(involvedIDs.end(), annotationInvolvedIds.begin(), annotationInvolvedIds.end());
        }
        c = ShapeOp::Constraint::shapeConstraintFactory("Similarity", involvedIDs, weight, points->transpose());
    } else if(this->type.compare("Points rectangle") == 0)
    {
        assert(annotations.size() == 4 &&
               annotations[0]->getType() == AnnotationType::Point &&
               annotations[1]->getType() == AnnotationType::Point &&
               annotations[2]->getType() == AnnotationType::Point &&
               annotations[3]->getType() == AnnotationType::Point );

        std::vector<int> involvedIDs;
        std::vector<int> a1InvolvedIds = getInvolvedVerticesIDs(annotations[0]);
        std::vector<int> a2InvolvedIds = getInvolvedVerticesIDs(annotations[1]);
        std::vector<int> a3InvolvedIds = getInvolvedVerticesIDs(annotations[2]);
        std::vector<int> a4InvolvedIds = getInvolvedVerticesIDs(annotations[3]);
        involvedIDs.insert(involvedIDs.end(), a1InvolvedIds.begin(), a1InvolvedIds.end());
        involvedIDs.insert(involvedIDs.end(), a2InvolvedIds.begin(), a2InvolvedIds.end());
        involvedIDs.insert(involvedIDs.end(), a3InvolvedIds.begin(), a3InvolvedIds.end());
        involvedIDs.insert(involvedIDs.end(), a4InvolvedIds.begin(), a4InvolvedIds.end());
        c = ShapeOp::Constraint::shapeConstraintFactory("Rectangle", involvedIDs, weight, points->transpose());
    } else if(this->type.compare("Points parallelogram") == 0)
    {
        assert(annotations.size() == 4 &&
               annotations[0]->getType() == AnnotationType::Point &&
               annotations[1]->getType() == AnnotationType::Point &&
               annotations[2]->getType() == AnnotationType::Point &&
               annotations[3]->getType() == AnnotationType::Point );

        std::vector<int> involvedIDs;
        std::vector<int> a1InvolvedIds = getInvolvedVerticesIDs(annotations[0]);
        std::vector<int> a2InvolvedIds = getInvolvedVerticesIDs(annotations[1]);
        std::vector<int> a3InvolvedIds = getInvolvedVerticesIDs(annotations[2]);
        std::vector<int> a4InvolvedIds = getInvolvedVerticesIDs(annotations[3]);
        involvedIDs.insert(involvedIDs.end(), a1InvolvedIds.begin(), a1InvolvedIds.end());
        involvedIDs.insert(involvedIDs.end(), a2InvolvedIds.begin(), a2InvolvedIds.end());
        involvedIDs.insert(involvedIDs.end(), a3InvolvedIds.begin(), a3InvolvedIds.end());
        involvedIDs.insert(involvedIDs.end(), a4InvolvedIds.begin(), a4InvolvedIds.end());
        c = ShapeOp::Constraint::shapeConstraintFactory("Parallelogram", involvedIDs, weight, points->transpose());
    } else if(this->type.compare("Points angle") == 0)
    {
        assert(annotations.size() == 3 &&
               annotations[0]->getType() == AnnotationType::Point &&
               annotations[1]->getType() == AnnotationType::Point &&
               annotations[2]->getType() == AnnotationType::Point );

        std::vector<int> involvedIDs;
        std::vector<int> a1InvolvedIds = getInvolvedVerticesIDs(annotations[0]);
        std::vector<int> a2InvolvedIds = getInvolvedVerticesIDs(annotations[1]);
        std::vector<int> a3InvolvedIds = getInvolvedVerticesIDs(annotations[2]);
        involvedIDs.insert(involvedIDs.end(), a1InvolvedIds.begin(), a1InvolvedIds.end());
        involvedIDs.insert(involvedIDs.end(), a2InvolvedIds.begin(), a2InvolvedIds.end());
        involvedIDs.insert(involvedIDs.end(), a3InvolvedIds.begin(), a3InvolvedIds.end());
        c = std::make_shared<ShapeOp::AngleConstraint>(involvedIDs, weight, points->transpose(), minValue, maxValue);
    } else if(this->type.compare("Lines parallelism") == 0)
    {
        ; //Angle == 0??
    } else if(this->type.compare("Lines perpendicularity") == 0)
    {
        ; //Angle == 90°??
    } else if(this->type.compare("Lines same length") == 0)
    {
        assert(annotations.size() == 2 &&
               annotations[0]->getType() == AnnotationType::Line &&
               annotations[1]->getType() == AnnotationType::Line );

        std::vector<int> involvedIDs;
        std::vector<int> a1InvolvedIds = getInvolvedVerticesIDs(annotations[0]);
        std::vector<int> a2InvolvedIds = getInvolvedVerticesIDs(annotations[1]);
        involvedIDs.insert(involvedIDs.end(), a1InvolvedIds.begin(), a1InvolvedIds.end());
        involvedIDs.insert(involvedIDs.end(), a2InvolvedIds.begin(), a2InvolvedIds.end());
        c = std::make_shared<ShapeOp::SimilarLengthConstraint>(involvedIDs, weight, points->transpose(), minValue, maxValue);
        dynamic_cast<ShapeOp::SimilarLengthConstraint*>(c.get())->setLinePointsCardinality1(a1InvolvedIds.size());
        dynamic_cast<ShapeOp::SimilarLengthConstraint*>(c.get())->setLinePointsCardinality2(a2InvolvedIds.size());
        constraints.push_back(c);
    } else if(this->type.compare("Lines simmetry") == 0)
    {
        ; //No idea
    }else if(this->type.compare("Lines coplanarity") == 0)
    {
        ; //Does this imply that the lines should be planar?
    } else if(this->type.compare("Surfaces coplanarity") == 0)
    {
        ; //Does this imply that the surfaces should be planar?
    } else if(this->type.compare("Surfaces structural continuity") == 0)
    {
        assert(annotations.size() == 2);
        std::vector<int> representativesIDs;
        std::vector<int> involvedIDs;
        SurfaceAnnotation* a1 = dynamic_cast<SurfaceAnnotation*>(annotations[0]);
        SurfaceAnnotation* a2 = dynamic_cast<SurfaceAnnotation*>(annotations[1]);
        std::vector<int> a2VerticesIds = getInvolvedVerticesIDs(a2);
        std::vector<std::vector<unsigned int> > a1OutlinesIDs;
        std::vector<std::vector<unsigned int> > a2OutlinesIDs;

        for(unsigned int i = 0; i < a1->getOutlines().size(); i++){
            std::vector<unsigned int> outline;
            for(unsigned int j = 0; j < a1->getOutlines()[i].size(); j++)
                outline.push_back(a1->getMesh()->getPointId(a1->getOutlines()[i][j]));
            a1OutlinesIDs.push_back(outline);
        }
        for(unsigned int i = 0; i < a2->getOutlines().size(); i++){
            std::vector<unsigned int> outline;
            for(unsigned int j = 0; j < a2->getOutlines()[i].size(); j++)
                outline.push_back(a2->getMesh()->getPointId(a2->getOutlines()[i][j]));
            a2OutlinesIDs.push_back(outline);
        }


        std::vector<int> tmp = getInvolvedVerticesIDs(a1);

        std::vector<unsigned int> firstSetTriangles = Utilities::pointsToTriangles(getInvolvedVerticesIDs(a1), a1->getMesh());

        for(unsigned int i = 0; i < firstSetTriangles.size(); i++)
        {
            IMATI_STL::Triangle* t = a1->getMesh()->getTriangle(firstSetTriangles[i]);
            IMATI_STL::Vertex* v = t->v1();
            for(unsigned int j = 0; j < 3; j++)
            {
                std::cout << v->x << " " << v->y << " " << v->z <<  " ";
                v=t->nextVertex(v);
            }
            std::cout << std::endl;
        }
        std::vector<unsigned int> secondSetTriangles = Utilities::pointsToTriangles(a2VerticesIds, a2->getMesh());
        GraphTemplate::Graph<Annotation*>* graph = a1->getMesh()->getGraph();

        std::vector<GraphTemplate::Arc<Annotation*>*> path = graph->shortestPathSearch(graph->getNodeFromData(a1), graph->getNodeFromData(a2), "adjacency");

        SurfaceAnnotation* a1adjacent = dynamic_cast<SurfaceAnnotation*>(path[0]->getOppositeNode(graph->getNodeFromData(a1))->getData());
        SurfaceAnnotation* a2adjacent = dynamic_cast<SurfaceAnnotation*>(path.back()->getOppositeNode(graph->getNodeFromData(a2))->getData());
        std::pair<unsigned int, unsigned int> adjacentBoundaries1IDs = a1->getAdjacentBoundary(a1adjacent);
        std::pair<unsigned int, unsigned int> adjacentBoundaries2IDs = a2->getAdjacentBoundary(a2adjacent);
        std::vector<std::pair<IMATI_STL::Point*, std::vector<IMATI_STL::Vertex*> > > firstSetSkeletonWithLoops =
                Utilities::extractSkeletonWithLoops(a1->getMesh(), firstSetTriangles, a1OutlinesIDs, adjacentBoundaries1IDs.first);
        std::vector<std::pair<IMATI_STL::Point*, std::vector<IMATI_STL::Vertex*> > > secondSetSkeletonWithLoops =
                Utilities::extractSkeletonWithLoops(a2->getMesh(), secondSetTriangles, a2OutlinesIDs, adjacentBoundaries2IDs.first);

        std::pair<Eigen::Vector3d, Eigen::Vector3d> axis, axis1, axis2;
        Eigen::MatrixXd secondSetSkeleton;
        secondSetSkeleton.resize(3, secondSetSkeletonWithLoops.size());
        for(unsigned int i = 0; i < secondSetSkeletonWithLoops.size(); i++)
        {
            IMATI_STL::Point* center = secondSetSkeletonWithLoops[i].first;
            Eigen::Vector3d p = {center->x, center->y, center->z};
            secondSetSkeleton.col(i) = p;
        }
        secondSetSkeleton.colwise() -= secondSetSkeleton.rowwise().mean();
        axis2.first = secondSetSkeleton.rowwise().mean();
        Eigen::Matrix3d U = secondSetSkeleton.bdcSvd(Eigen::ComputeFullU | Eigen::ComputeThinV).matrixU();
        axis2.second = U.col(0);
        unsigned int maxReachedPos = 0;
        for(unsigned int i = 1; i < firstSetSkeletonWithLoops.size(); i++)
        {
            IMATI_STL::Point d = (*firstSetSkeletonWithLoops[i].first) - (*firstSetSkeletonWithLoops[i - 1].first);
            d.normalize();
            Eigen::Vector3d direction = {d.x, d.y, d.z};
            double angle = direction.dot(axis2.second);
            if(angle < 0)
                angle = direction.dot(-axis2.second);
            angle = acos(angle);

            if(angle > 0.1) //il cono deve essere massimo di 10°
                break;
            maxReachedPos++;
        }

        std::vector<unsigned int> reachedBoundary;
        for(unsigned int i = 0; i < firstSetSkeletonWithLoops[maxReachedPos].second.size(); i++)
            reachedBoundary.push_back(a1->getMesh()->getPointId(firstSetSkeletonWithLoops[maxReachedPos].second[i]));
        std::vector<std::vector<unsigned int> > newOutlines;
        newOutlines.push_back(a1OutlinesIDs[adjacentBoundaries1IDs.first]);
        newOutlines.push_back(reachedBoundary);
        std::vector<unsigned int> newTriangles = Utilities::regionGrowing(a1->getMesh(), newOutlines);

        std::vector<int> a1VerticesIds = Utilities::trianglesToPoints(newTriangles, a1->getMesh());

        std::vector<int> verticesIds = a1VerticesIds;
        verticesIds.insert(verticesIds.end(), a2VerticesIds.begin(), a2VerticesIds.end());

        std::shared_ptr<ShapeOp::Constraint> c;
        c = ShapeOp::Constraint::shapeConstraintFactory("Coaxiality", verticesIds, weight, points->transpose());
        dynamic_cast<ShapeOp::CoaxialityConstraint*>(c.get())->setFirstSetOutlines(newOutlines);
        dynamic_cast<ShapeOp::CoaxialityConstraint*>(c.get())->setSecondSetOutlines(a2OutlinesIDs);
        dynamic_cast<ShapeOp::CoaxialityConstraint*>(c.get())->setFirstSetSeedLoop(0);
        dynamic_cast<ShapeOp::CoaxialityConstraint*>(c.get())->setSecondSetSeedLoop(adjacentBoundaries2IDs.first);
        dynamic_cast<ShapeOp::CoaxialityConstraint*>(c.get())->setFirstSetSize(a1VerticesIds.size());
        dynamic_cast<ShapeOp::CoaxialityConstraint*>(c.get())->setSecondSetSize(a2VerticesIds.size());
        dynamic_cast<ShapeOp::CoaxialityConstraint*>(c.get())->setMesh(a1->getMesh());

        constraints.push_back(c);
        std::vector<int> a1WholeInvolvedVertices = getInvolvedVerticesIDs(a1);
        for(unsigned int i = 0; i < a1WholeInvolvedVertices.size(); i++)
        {
            std::vector<int> neighbourhood;
            IMATI_STL::Vertex* v = a1->getMesh()->getPoint(a1WholeInvolvedVertices[i]);
            neighbourhood.push_back(a1->getMesh()->getPointId(v));
            for(IMATI_STL::Node* n = v->VV()->head(); n != nullptr; n=n->next())
                neighbourhood.push_back(a1->getMesh()->getPointId(static_cast<IMATI_STL::Vertex*>(n->data)));
            c = ShapeOp::Constraint::shapeConstraintFactory("Laplacian", neighbourhood, weight, points->transpose());
            constraints.push_back(c);
        }
    } else if(this->type.compare("Surfaces same area") == 0)
    {
        ; //Based on min/max area
    } else if(this->type.compare("Surfaces rigidity") == 0)
    {
        assert(annotations.size() >= 3);
        std::vector<int> representativesIDs;
        std::vector<int> involvedIDs;
        for(unsigned int i = 0; i < annotations.size(); i++){
            std::vector<int> annotationsInvolvedIDs = getInvolvedVerticesIDs(annotations[i]);
            Eigen::Matrix3Xd annotationPoints;
            annotationPoints.resize(3, static_cast<int>(annotationsInvolvedIDs.size()));
            for(unsigned j = 0; j < annotationsInvolvedIDs.size(); j++)
                annotationPoints.col(j) = points->row(annotationsInvolvedIDs[j]);
            std::pair<Eigen::Vector3d, Eigen::Vector3d> plane = Utilities::planarRegression(annotationPoints);
            Eigen::Vector3d planeCentroid = Eigen::Vector3d::Zero(3);
            for(unsigned j = 0; j < annotationsInvolvedIDs.size(); j++){
                Eigen::Vector3d p = annotationPoints.col(j);
                Eigen::Vector3d v = p - plane.first;
                double dist = abs(v.dot(plane.second));
                Eigen::Vector3d projected = p - plane.second * dist;
                planeCentroid += projected;
            }
            planeCentroid /= annotationsInvolvedIDs.size();
            double bestDist = DBL_MAX;
            unsigned int bestId = INT_MAX;
            for(unsigned int j = 0; j < annotationsInvolvedIDs.size(); j++){
                involvedIDs.push_back(annotationsInvolvedIDs[j]);
                double dist = (planeCentroid - annotationPoints.col(j)).norm();
                if(dist < bestDist){
                    bestId = j;
                    bestDist = dist;
                }
            }

            representativesIDs.push_back(static_cast<int>(annotationsInvolvedIDs[bestId]));
        }
        c = ShapeOp::Constraint::shapeConstraintFactory("Rigid", representativesIDs, weight, points->transpose());
        constraints.push_back(c);
        c = ShapeOp::Constraint::shapeConstraintFactory("Rigid", involvedIDs, weight, points->transpose());
        constraints.push_back(c);
    } else if(this->type.compare("Surfaces same measure") == 0)
    {
        ; //Based on min/max length
    } else if(this->type.compare("Surfaces same width") == 0)
    {
        ;
    } else if(this->type.compare("Surfaces same orientation") == 0)
    {
        ; //Based on orientation
    } else if(this->type.compare("Surfaces angle") == 0)
    {
        ; //Based on orientation
    } else if(this->type.compare("Surfaces same level") == 0)
    {
        assert(annotations.size() >= 3);
        std::vector<int> representativesIDs;
        std::vector<int> involvedIDs;
        std::vector<Annotation*> fathers;
        for(unsigned int i = 0; i < annotations.size(); i++){
            SurfaceAnnotation* a = dynamic_cast<SurfaceAnnotation*>(annotations[i]);
            GraphTemplate::Node<Annotation*>* aNode = a->getMesh()->getGraph()->getNodeFromData(a);
            std::vector<int> annotationsInvolvedIDs = getInvolvedVerticesIDs(a);
            GraphTemplate::Arc<Annotation*>* parentality = a->getMesh()->getGraph()->getArcsFromTip(aNode, "containment")[0];
            //Since we do not allow partial overlap, the father can be only one
            fathers.push_back(parentality->getOppositeNode(aNode)->getData());
            std::vector<std::vector<IMATI_STL::Vertex*> > outlines = a->getOutlines();
            #pragma omp parallel for
            for(unsigned int j = 0; j < outlines.size(); j++)
                #pragma omp parallel for
                for(unsigned int k = 0; k < outlines[j].size(); k++)
                    annotationsInvolvedIDs.push_back(static_cast<int>(a->getMesh()->getPointId(outlines[j][k])));

            c = ShapeOp::Constraint::shapeConstraintFactory("Rigid", annotationsInvolvedIDs, weight, points->transpose());
            constraints.push_back(c);

            Eigen::Matrix3Xd annotationPoints;
            annotationPoints.resize(3, static_cast<int>(annotationsInvolvedIDs.size()));
            #pragma omp parallel for
            for(unsigned j = 0; j < annotationsInvolvedIDs.size(); j++)
                annotationPoints.col(j) = points->row(annotationsInvolvedIDs[j]);


            std::pair<Eigen::Vector3d, Eigen::Vector3d> plane = Utilities::planarRegression(annotationPoints);

            Eigen::Vector3d planeCentroid = Eigen::Vector3d::Zero(3);
            #pragma omp parallel for
            for(unsigned j = 0; j < annotationsInvolvedIDs.size(); j++){
                Eigen::Vector3d p = annotationPoints.col(j);
                Eigen::Vector3d v = p - plane.first;
                double dist = abs(v.dot(plane.second));
                Eigen::Vector3d projected = p - plane.second * dist;
                planeCentroid += projected;
            }
            planeCentroid /= annotationsInvolvedIDs.size();
            double bestDist = DBL_MAX;
            unsigned int bestPos = INT_MAX;

            for(unsigned int j = 0; j < annotationsInvolvedIDs.size(); j++){
                involvedIDs.push_back(annotationsInvolvedIDs[j]);
                double dist = (planeCentroid - annotationPoints.col(j)).norm();
                if(dist < bestDist){
                    bestPos = j;
                    bestDist = dist;
                }
            }
            representativesIDs.push_back(static_cast<int>(annotationsInvolvedIDs[bestPos]));
        }
        c = ShapeOp::Constraint::shapeConstraintFactory("Orientation", representativesIDs, weight, points->transpose());
        ShapeOp::Vector3 orientation = {0,0,1};
        orientation.normalize();
        dynamic_cast<ShapeOp::OrientationConstraint*>(c.get())->setOrientation(orientation);
        constraints.push_back(c);
        std::vector<unsigned int> usedFathers;
        for(unsigned int i = 0; i < fathers.size(); i++)
        {
            if(find(usedFathers.begin(), usedFathers.end(), fathers[i]->getId()) != usedFathers.end())
                continue;
            usedFathers.push_back(fathers[i]->getId());
            std::vector<int> involvedPointsIds = getInvolvedVerticesIDs(fathers[i]);
            for(unsigned int j = 0; j < involvedPointsIds.size(); j++){
                std::vector<int> pointNeighbourhood = {involvedPointsIds[j]};
                IMATI_STL::Node* n = fathers[i]->getMesh()->getPoint(involvedPointsIds[j])->VV()->head();
                for(; n != nullptr; n = n->next())
                {
                    IMATI_STL::Vertex* v = (static_cast<IMATI_STL::Vertex*>(n->data));
                    pointNeighbourhood.push_back(fathers[i]->getMesh()->getPointId(v));
                }
                c = ShapeOp::Constraint::shapeConstraintFactory("LaplacianDisplacement", pointNeighbourhood, weight, points->transpose());
                constraints.push_back(c);
            }
        }
    } else if(this->type.compare("Surfaces simmetry") == 0)
    {
        ; //No idea
    } else {
        throw("This type of constraint can not be applied to sets");
    }

}

double AnnotationsConstraint::getError()
{
    double error = 0;
    for(unsigned int i = 0; i < constraints.size(); i++){
        double cError = solver->getResidualError(constraints[i]) / constraints[i]->getWeight();
        error += cError;

        std::cout << cError << " ";
    }

    std::cout << std::endl << std::flush;
    error /= constraints.size();

    return error;
}

ShapeOp::MatrixX3 *AnnotationsConstraint::getPoints() const
{
    return points;
}

void AnnotationsConstraint::setPoints(ShapeOp::MatrixX3 *value)
{
    points = value;
}

std::vector<std::shared_ptr<ShapeOp::Constraint> > AnnotationsConstraint::getConstraints() const
{
    return constraints;
}

void AnnotationsConstraint::setConstraints(const std::vector<std::shared_ptr<ShapeOp::Constraint> > &value)
{
    constraints = value;
}

ConstraintSolver *AnnotationsConstraint::getSolver() const
{
    return solver;
}

void AnnotationsConstraint::setSolver(ConstraintSolver *value)
{
    solver = value;
}

std::vector<int> AnnotationsConstraint::getInvolvedVerticesIDs(Annotation *annotation)
{
    std::vector<IMATI_STL::Vertex*> involvedVertices = annotation->getInvolvedVertices();
    std::vector<int> involvedIDs;
    for (unsigned int i = 0; i < involvedVertices.size(); i++)
        involvedIDs.push_back( static_cast<int>(annotation->getMesh()->getPointId(involvedVertices[i])));

    return involvedIDs;
}
