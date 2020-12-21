#include "relationship.h"

Relationship::Relationship()
{
    constrained = false;
}

Annotation *Relationship::getActor1() const
{
    return actor1;
}

void Relationship::setActor1(Annotation *value)
{
    actor1 = value;
}

Annotation *Relationship::getActor2() const
{
    return actor2;
}

void Relationship::setActor2(Annotation *value)
{
    actor2 = value;
}

SameMeasureRelationship::SameMeasureRelationship()
{

}

std::string SameMeasureRelationship::getMeasureName1() const
{
    return measureName1;
}

void SameMeasureRelationship::setMeasureName1(const std::string &value)
{
    measureName1 = value;
}

std::string SameMeasureRelationship::getMeasureName2() const
{
    return measureName2;
}

void SameMeasureRelationship::setMeasureName2(const std::string &value)
{
    measureName2 = value;
}

double SameMeasureRelationship::getRangeMin() const
{
    return rangeMin;
}

void SameMeasureRelationship::setRangeMin(double value)
{
    rangeMin = value;
}

double SameMeasureRelationship::getRangeMax() const
{
    return rangeMax;
}

void SameMeasureRelationship::setRangeMax(double value)
{
    rangeMax = value;
}

SymmetryRelationship::SymmetryRelationship()
{

}
