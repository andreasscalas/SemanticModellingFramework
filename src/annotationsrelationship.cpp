#include "annotationsrelationship.h"

AnnotationsRelationship::AnnotationsRelationship()
{
    type = "";
    weight = 0.0;
    minValue = -DBL_MAX;
    maxValue = DBL_MAX;
    id = 0;
}

AnnotationsRelationship::~AnnotationsRelationship()
{
    annotations.clear();
}

std::string AnnotationsRelationship::getType() const
{
    return type;
}

void AnnotationsRelationship::setType(const std::string &value)
{
    type = value;
}

std::vector<Annotation *> AnnotationsRelationship::getAnnotations() const
{
    return annotations;
}

void AnnotationsRelationship::setAnnotations(const std::vector<Annotation *> &value)
{
    annotations = value;
}

double AnnotationsRelationship::getWeight() const
{
    return weight;
}

void AnnotationsRelationship::setWeight(double value)
{
    weight = value;
}

double AnnotationsRelationship::getMinValue() const
{
    return minValue;
}

void AnnotationsRelationship::setMinValue(double value)
{
    minValue = value;
}

double AnnotationsRelationship::getMaxValue() const
{
    return maxValue;
}

void AnnotationsRelationship::setMaxValue(double value)
{
    maxValue = value;
}

void AnnotationsRelationship::print(std::ostream &os)
{
    os << "id: " << id << std::endl;
    os << "type: " << type << std::endl;
    os << "weight: " << weight << std::endl;
    if(type.compare("Surfaces same measure") == 0)
    {
        os << "minimum value:" << minValue << std::endl;
        os << "maximum value: " << maxValue << std::endl;
    }
    os << "annotations ids: [";
    for(unsigned int i = 0; i < annotations.size(); i++){
        os << annotations[i]->getId();
        if(i < annotations.size() - 1)
            os << ",";
    }
    os << "]" << std::endl;

}


void AnnotationsRelationship::printJSON(rapidjson::PrettyWriter<rapidjson::StringBuffer> &writer)
{
    writer.Key("type");
    writer.String(type.c_str());
    writer.Key("weight");
    writer.Double(weight);
    writer.Key("minimum value");
    writer.Double(minValue);
    writer.Key("maximum value");
    writer.Double(maxValue);
    writer.Key("annotations");
    writer.StartArray();
    for(unsigned int i = 0; i < annotations.size(); i++)
        writer.Uint(annotations[i]->getId());

}

unsigned int AnnotationsRelationship::getId() const
{
    return id;
}

void AnnotationsRelationship::setId(unsigned int value)
{
    id = value;
}
