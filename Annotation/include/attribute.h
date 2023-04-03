#ifndef ATTRIBUTE_H
#define ATTRIBUTE_H

#include <string>
#include <rapidjson/prettywriter.h>

class Attribute
{
public:
    virtual ~Attribute() {  }
    virtual void print(std::ostream& os);
    virtual void printJson(rapidjson::PrettyWriter<rapidjson::StringBuffer> &writer);

    std::string getKey() const;
    void setKey(const std::string &value);
    void *getValue() const;
    void setValue(void *value);
    void setValue(int);
    void setValue(double);
    void setValue(std::string);
    unsigned int getId() const;
    void setId(unsigned int value);

protected:
    unsigned int id;
    std::string key;
    void* value;
};

#endif // ATTRIBUTE_H
