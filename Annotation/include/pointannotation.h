#ifndef POINTANNOTATION_H
#define POINTANNOTATION_H

#include <annotation.h>


class PointAnnotation : virtual public Annotation
{
public:
    PointAnnotation();
   virtual ~PointAnnotation() override;

    virtual Annotation* transfer(ExtendedTrimesh* otherMesh, short metric = 2) override;

    virtual Annotation* parallelTransfer(ExtendedTrimesh* otherMesh, short metric = 2) override;

    virtual void print(std::ostream&) override;

    virtual void printJson(rapidjson::PrettyWriter<rapidjson::StringBuffer>&) override;

    virtual std::vector<IMATI_STL::Vertex*> getInvolvedVertices() override;

    virtual bool isPointInAnnotation(IMATI_STL::Vertex* annotatedPoint) override;

    std::vector<IMATI_STL::Vertex *> getPoints() const;
    void setPoints(const std::vector<IMATI_STL::Vertex *> &value);
    void addPoint(IMATI_STL::Vertex* value);

    virtual IMATI_STL::Point* getCenter() override;
    virtual IMATI_STL::Point* getOrientation() override;

protected:
    std::vector<IMATI_STL::Vertex*> points;

};

#endif // POINTANNOTATION_H
