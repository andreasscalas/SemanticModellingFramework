#include "geometricattribute.h"
#include <algorithm>
#include <iostream>

GeometricAttribute::GeometricAttribute()
{
}

GeometricAttribute::~GeometricAttribute()
{
}

void GeometricAttribute::print(std::ostream &writer)
{

    Attribute::print(writer);
    writer << "measure : " << *static_cast<double*>(value) << std::endl;

    writer << "measure points: [" << std::endl;
    for(unsigned int i = 0; i < measurePointsID.size(); i++)
        writer << measurePointsID[i] << " ";

    writer << std::endl << "]" << std::endl;
}

void GeometricAttribute::printJson(rapidjson::PrettyWriter<rapidjson::StringBuffer> &writer)
{
    Attribute::printJson(writer);
    writer.Key("type");
    writer.String("Geometric");
    writer.Key("value");
    double measure = *static_cast<double*>(value);
    writer.Double(measure);
    writer.Key("points");
    writer.StartArray();
    for(unsigned int i = 0; i < measurePointsID.size(); i++)
        writer.Uint(measurePointsID[i]);
    writer.EndArray();
}

std::vector<unsigned int> GeometricAttribute::getMeasurePointsID() const
{
    return measurePointsID;
}

void GeometricAttribute::setMeasurePointsID(const std::vector<unsigned int> &value)
{
    measurePointsID = value;
}

void GeometricAttribute::addMeasurePointID(const unsigned int &value)
{
    measurePointsID.push_back(value);
}

void GeometricAttribute::removeMeasurePointID(const unsigned int value)
{
    measurePointsID.erase(std::find(measurePointsID.begin(), measurePointsID.end(), value));
}

void GeometricAttribute::clearMeasurePointsID()
{
    measurePointsID.clear();
}
