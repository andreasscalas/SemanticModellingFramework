#ifndef SEMANTICATTRIBUTE_H
#define SEMANTICATTRIBUTE_H

#include <attribute.h>

class SemanticAttribute : virtual public Attribute
{
public:
    SemanticAttribute();


    virtual void print(std::ostream&);
    virtual void printJson(rapidjson::PrettyWriter<rapidjson::StringBuffer>&);

};

#endif // SEMANTICATTRIBUTE_H
