#include "annotationmeasuresconstraint.h"

#include <drawableboundingmeasure.h>
#include <utilities.h>
#include <surfaceannotation.h>
#include <drawableheightmeasure.h>
AnnotationMeasuresConstraint::AnnotationMeasuresConstraint()
{

}

void AnnotationMeasuresConstraint::constrain()
{
    std::shared_ptr<ShapeOp::Constraint> c;
    assert(annotations.size() <= 2);
    std::vector<int> representativesIDs;
    std::vector<int> involvedIDs;
    SurfaceAnnotation* a = dynamic_cast<SurfaceAnnotation*>(annotations[0]);
    std::vector<IMATI_STL::Vertex*> aVertices = a->getInvolvedVertices();
    std::vector<int> aVerticesIndices = Utilities::verticesToIntIDvector(a->getMesh(), aVertices);
    std::vector<unsigned int> measure1Points = attribute1->getMeasurePointsID();
    std::vector<unsigned int> measure2Points = attribute2->getMeasurePointsID();

    assert(measure1Points.size() == 2 && measure2Points.size() == 2);
    c = ShapeOp::Constraint::shapeConstraintFactory("SameMeasure", aVerticesIndices, weight, points->transpose());
    dynamic_cast<ShapeOp::NonUniformScalingConstraint*>(c.get())->setMinProportion(minValue);
    dynamic_cast<ShapeOp::NonUniformScalingConstraint*>(c.get())->setMaxProportion(maxValue);
    dynamic_cast<ShapeOp::NonUniformScalingConstraint*>(c.get())->setL1StartIndex(measure1Points[0]);
    dynamic_cast<ShapeOp::NonUniformScalingConstraint*>(c.get())->setL1EndIndex(measure1Points[1]);
    if(dynamic_cast<DrawableBoundingMeasure*>(attribute1) != nullptr)
    {
        IMATI_STL::Point* d = dynamic_cast<DrawableBoundingMeasure*>(attribute1)->getDirection();
        ShapeOp::Vector3 direction = {d->x, d->y, d->z};
        dynamic_cast<ShapeOp::NonUniformScalingConstraint*>(c.get())->setL1directed(true);
        dynamic_cast<ShapeOp::NonUniformScalingConstraint*>(c.get())->setL1direction(direction);
    } else if(dynamic_cast<DrawableHeightMeasure*>(attribute1) != nullptr)
    {
        IMATI_STL::Point* d = dynamic_cast<DrawableHeightMeasure*>(attribute1)->getDirection();
        ShapeOp::Vector3 direction = {d->x, d->y, d->z};
        dynamic_cast<ShapeOp::NonUniformScalingConstraint*>(c.get())->setL1directed(true);
        dynamic_cast<ShapeOp::NonUniformScalingConstraint*>(c.get())->setL1direction(direction);
    }
    dynamic_cast<ShapeOp::NonUniformScalingConstraint*>(c.get())->setL2StartIndex(measure2Points[0]);
    dynamic_cast<ShapeOp::NonUniformScalingConstraint*>(c.get())->setL2EndIndex(measure2Points[1]);
    if(dynamic_cast<DrawableBoundingMeasure*>(attribute2) != nullptr)
    {
        IMATI_STL::Point* d = dynamic_cast<DrawableBoundingMeasure*>(attribute2)->getDirection();
        ShapeOp::Vector3 direction = {d->x, d->y, d->z};
        dynamic_cast<ShapeOp::NonUniformScalingConstraint*>(c.get())->setL2directed(true);
        dynamic_cast<ShapeOp::NonUniformScalingConstraint*>(c.get())->setL2direction(direction);
    }

    constraints.push_back(c);

}

void AnnotationMeasuresConstraint::print(std::ostream &os)
{
    AnnotationsRelationship::print(os);
    os << "measure 1 id: " << this->attribute1->getId() << std::endl;
    os << "measure 2 id: " << this->attribute2->getId() << std::endl;
    double error = 0;
    for(unsigned int i = 0; i < constraints.size(); i++){
        double cError = solver->getResidualError(constraints[i]) / constraints[i]->getWeight();
        error += cError;
    }

    os << "Average error: " << error / constraints.size() << std::endl;
    os << "Total error: " << error << std::endl << std::flush;
}

void AnnotationMeasuresConstraint::checkConstraint()
{
    dynamic_cast<DrawableAttribute*>(attribute1)->update();
    dynamic_cast<DrawableAttribute*>(attribute2)->update();
    double measure1 = *static_cast<double*>(attribute1->getValue());
    double measure2 = *static_cast<double*>(attribute2->getValue());
    double proportion = measure1 / measure2;
    if(proportion < minValue || proportion > maxValue)
        dynamic_cast<DrawableAttribute*>(attribute1)->setError(true);
    else
        dynamic_cast<DrawableAttribute*>(attribute1)->setError(false);

}
