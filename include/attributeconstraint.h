#ifndef ATTRIBUTECONSTRAINT_H
#define ATTRIBUTECONSTRAINT_H

#include <semanticconstraint.h>
#include <attribute.h>

enum class SemanticRelationshipType{
    MeasureRatio
};
class AttributeConstraint : public SemanticConstraint
{
public:
    AttributeConstraint();
    virtual void constrain() override;
    virtual double getError() override;
    std::vector<Attribute *> getAttributes() const;
    void setAttributes(const std::vector<Attribute *> &value);
    SemanticRelationshipType getType() const;
    void setType(const SemanticRelationshipType &value);



protected:
    std::vector<Attribute*> attributes;
    SemanticRelationshipType type;
};

#endif // ATTRIBUTECONSTRAINT_H
