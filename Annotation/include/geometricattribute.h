#ifndef GEOMETRICATTRIBUTE_H
#define GEOMETRICATTRIBUTE_H

#include <attribute.h>
#include <vector>

class GeometricAttribute : virtual public Attribute
{
public:
    GeometricAttribute();
    ~GeometricAttribute();

    virtual void print(std::ostream&) override;
    virtual void printJson(rapidjson::PrettyWriter<rapidjson::StringBuffer>&) override;

    std::vector<unsigned int> getMeasurePointsID() const;
    void setMeasurePointsID(const std::vector<unsigned int> &value);
    void addMeasurePointID(const unsigned int &value);
    void removeMeasurePointID(const unsigned int value);
    void clearMeasurePointsID();

    bool isDirected() const;
    void setDirected(bool value);

    double *getDirection() const;
    void setDirection(double x, double y, double z);


protected:
    std::vector<unsigned int> measurePointsID;


};

#endif // GEOMETRICATTRIBUTE_H
