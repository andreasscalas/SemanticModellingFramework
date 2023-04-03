#include "annotationmeasurerelationship.h"

AnnotationMeasuresRelationship::AnnotationMeasuresRelationship()
{

}

AnnotationMeasuresRelationship::~AnnotationMeasuresRelationship()
{
    attribute1 = nullptr;
    attribute2 = nullptr;
}

GeometricAttribute *AnnotationMeasuresRelationship::getAttribute1() const
{
    return attribute1;
}

void AnnotationMeasuresRelationship::setAttribute1(GeometricAttribute *value)
{
    attribute1 = value;
}

GeometricAttribute *AnnotationMeasuresRelationship::getAttribute2() const
{
    return attribute2;
}

void AnnotationMeasuresRelationship::setAttribute2(GeometricAttribute *value)
{
    attribute2 = value;
}

void AnnotationMeasuresRelationship::print(std::ostream &os)
{
    AnnotationsRelationship::print(os);
    os << "measure 1 id: " << this->attribute1->getId() << std::endl;
    os << "measure 2 id: " << this->attribute2->getId() << std::endl;
}

void AnnotationMeasuresRelationship::printJSON(rapidjson::PrettyWriter<rapidjson::StringBuffer> &writer)
{
    AnnotationsRelationship::printJSON(writer);
    writer.Key("measure 1 id");
    writer.Uint(this->attribute1->getId());
    writer.Key("measure 2 id");
    writer.Uint(this->attribute2->getId());
}
