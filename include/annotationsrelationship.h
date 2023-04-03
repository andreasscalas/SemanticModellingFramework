#ifndef ANNOTATIONSRELATIONSHIP_H
#define ANNOTATIONSRELATIONSHIP_H

#include <vector>
#include <string>

#include <annotation.h>

class AnnotationsRelationship
{
public:
    AnnotationsRelationship();

    virtual ~AnnotationsRelationship();
    unsigned int getId() const;
    void setId(unsigned int value);
    std::string getType() const;
    void setType(const std::string &value);
    std::vector<Annotation *> getAnnotations() const;
    void setAnnotations(const std::vector<Annotation *> &value);
    double getWeight() const;
    void setWeight(double value);
    double getMinValue() const;
    void setMinValue(double value);
    double getMaxValue() const;
    void setMaxValue(double value);

    virtual void print(std::ostream &os);
    virtual void printJSON(rapidjson::PrettyWriter<rapidjson::StringBuffer> &writer);


protected:
    unsigned int id;
    std::vector<Annotation *> annotations;
    std::string type;
    double weight;
    double minValue, maxValue;
};

#endif // ANNOTATIONSRELATIONSHIP_H
