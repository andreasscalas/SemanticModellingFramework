#ifndef GEOMETRICATTRIBUTE_H
#define GEOMETRICATTRIBUTE_H

#include <attribute.h>
#include <vector>

class GeometricAttribute : virtual public Attribute
{
public:
    GeometricAttribute();
    virtual ~GeometricAttribute() override;

    virtual void print(std::ostream&) override;
    virtual void printJson(rapidjson::PrettyWriter<rapidjson::StringBuffer>&) override;

    std::vector<unsigned int> getMeasurePointsID() const;
    void setMeasurePointsID(const std::vector<unsigned int> &value);
    void addMeasurePointID(const unsigned int &value);
    void removeMeasurePointID(const unsigned int value);
    void clearMeasurePointsID();

protected:
    std::vector<unsigned int> measurePointsID;


};

#endif // GEOMETRICATTRIBUTE_H
