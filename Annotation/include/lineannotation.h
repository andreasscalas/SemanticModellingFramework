#ifndef LINEANNOTATION_H
#define LINEANNOTATION_H

#include <vector>
#include <annotation.h>
#include <vertex.h>

class LineAnnotation : virtual public Annotation
{
public:
    LineAnnotation();

    ~LineAnnotation();

    virtual Annotation* transfer(ExtendedTrimesh* otherMesh, short metric = 2);

    /**
     * @brief parallelTransfer This method takes the annotations of an object defined on a model with
     * a certain resolution and transfers it to a model with another resolution using multiple threads
     * @param otherMesh The model with lower resolution
     * @param metric the metric to be used for the shortest path
     * @return The annotation defined on the other model.
     */
    virtual Annotation* parallelTransfer(ExtendedTrimesh* otherMesh, short metric = 2);

    virtual void print (std::ostream&);
    virtual void printJson(rapidjson::PrettyWriter<rapidjson::StringBuffer>&);

    virtual std::vector<IMATI_STL::Vertex*> getInvolvedVertices();
    virtual bool isPointInAnnotation(IMATI_STL::Vertex* p);

    void addPolyLine(std::vector<IMATI_STL::Vertex *> &value);
    std::vector<std::vector<IMATI_STL::Vertex *> > getPolyLines() const;
    void setPolyLines(const std::vector<std::vector<IMATI_STL::Vertex *> > &value);
    void clearPolylines();


    virtual IMATI_STL::Point* getCenter() override;
    virtual IMATI_STL::Point* getOrientation() override;

protected:
    std::vector<std::vector<IMATI_STL::Vertex*> > polyLines;
    const short BBOX_SPHERE_RATIO = 1000;                   //Divisive coefficient between the BBox longest diagonal and neighborhood sphere radius
    const bool POST_PROCESSING = true;                     //Spikes can be removed in postprocessing or avoided by inserting infinite weights in the shortest path computation.
};

#endif // LINEANNOTATION_H
