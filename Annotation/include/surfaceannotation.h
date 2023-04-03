#ifndef AREAANNOTATION_H
#define AREAANNOTATION_H

#include <annotation.h>

class SurfaceAnnotation : virtual public Annotation
{
public:
    SurfaceAnnotation();

    virtual ~SurfaceAnnotation() override;

    virtual Annotation* transfer(ExtendedTrimesh* otherMesh, short metric = 2) override;

    /**
     * @brief parallelTransfer This method takes the annotations of an object defined on a model with
     * a certain resolution and transfers it to a model with another resolution using multiple threads
     * @param otherMesh The model with lower resolution
     * @param metric the metric to be used for the shortest path
     * @return The annotation defined on the other model.
     */
    virtual Annotation* parallelTransfer(ExtendedTrimesh* otherMesh, short metric = 2) override;

    virtual void print(std::ostream&) override;

    virtual void printJson(rapidjson::PrettyWriter<rapidjson::StringBuffer>&) override;

    virtual std::vector<IMATI_STL::Vertex*> getInvolvedVertices() override;
    virtual std::vector<IMATI_STL::Vertex*> getInnerVertices();

    virtual bool isPointInAnnotation(IMATI_STL::Vertex* p) override;
    virtual bool isPointInsideAnnotation(IMATI_STL::Vertex* p);
    bool isPointOnBorder(IMATI_STL::Vertex* p);
    bool isPointOnBorder(IMATI_STL::Vertex* p, unsigned int &boundaryIndex);

    virtual bool isTriangleInAnnotation(IMATI_STL::Triangle* p);

    std::pair<unsigned int, unsigned int> getAdjacentBoundary(SurfaceAnnotation* a);
    bool checkAdjacency(SurfaceAnnotation* a);

    std::vector<int> getIsBoundaryStatus();

    //Getter and setter methods
    std::vector<std::vector<IMATI_STL::Vertex *> > getOutlines() const;
    void setOutlines(const std::vector<std::vector<IMATI_STL::Vertex *> > value);
    void addOutline(const std::vector<IMATI_STL::Vertex*> value);
    std::vector<IMATI_STL::Triangle*> getTriangles();


    virtual IMATI_STL::Point* getCenter() override;
    virtual IMATI_STL::Point* getOrientation() override;

protected:
    std::vector<std::vector<IMATI_STL::Vertex*> > outlines; //The outline of the annotated region
    const short BBOX_SPHERE_RATIO = 100;                    //Divisive coefficient between the BBox longest diagonal and neighborhood sphere radius
    const bool ORDER = true;                                //Order of the outline: if TRUE then it is counterclockwise, otherwise is clockwise
    const bool POST_PROCESSING = false;                     //Spikes can be removed in postprocessing or avoided by inserting infinite weights in the shortest path computation.
};

#endif // AREAANNOTATION_H
