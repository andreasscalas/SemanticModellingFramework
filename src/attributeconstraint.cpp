#include "attributeconstraint.h"
//#include <semanticattribute.h>
#include <geometricattribute.h>

AttributeConstraint::AttributeConstraint()
{

}

void AttributeConstraint::constrain()
{
    switch (type) {
        case SemanticRelationshipType::MeasureRatio:
            assert(attributes.size() == 2);
            std::vector<int> involvedIds;

            std::vector<unsigned int> a1ids = dynamic_cast<GeometricAttribute*>(attributes[0])->getMeasurePointsID();
            std::vector<unsigned int> a2ids = dynamic_cast<GeometricAttribute*>(attributes[1])->getMeasurePointsID();
            for(unsigned int i = 0; i < a1ids.size(); i++)
                involvedIds.push_back(static_cast<int>(a1ids[i]));
            for(unsigned int i = 0; i < a2ids.size(); i++)
                involvedIds.push_back(static_cast<int>(a2ids[i]));

            auto c = std::make_shared<ShapeOp::SimilarLengthConstraint>(involvedIds, weight, points->transpose(), minValue, maxValue);
            constraints.push_back(c);
            break;
            /*
        default:
            std::cerr << "This kind of constraint cannot be applied to attributes.";*/
    }
}

double AttributeConstraint::getError()
{
    return 0;
}

std::vector<Attribute *> AttributeConstraint::getAttributes() const
{
    return attributes;
}

void AttributeConstraint::setAttributes(const std::vector<Attribute *> &value)
{
    attributes = value;
}

SemanticRelationshipType AttributeConstraint::getType() const
{
    return type;
}

void AttributeConstraint::setType(const SemanticRelationshipType &value)
{
    type = value;
}

