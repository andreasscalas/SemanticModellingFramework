#ifndef RELATIONSHIP_H
#define RELATIONSHIP_H

#include <annotation.h>


class Relationship
{
public:
    Relationship();
    Annotation *getActor1() const;
    void setActor1(Annotation *value);
    Annotation *getActor2() const;
    void setActor2(Annotation *value);

protected:
    Annotation *actor1, *actor2;
    bool constrained;

};

class SameMeasureRelationship : Relationship
{
public:
    SameMeasureRelationship();

    std::string getMeasureName1() const;
    void setMeasureName1(const std::string &value);
    std::string getMeasureName2() const;
    void setMeasureName2(const std::string &value);
    double getRangeMin() const;
    void setRangeMin(double value);
    double getRangeMax() const;
    void setRangeMax(double value);

private:
    std::string measureName1, measureName2;
    double rangeMin, rangeMax;
};

class SymmetryRelationship : Relationship
{
public:
    SymmetryRelationship();

private:
};

#endif // RELATIONSHIP_H
