#ifndef EXTENDEDTRIMESH_H
#define EXTENDEDTRIMESH_H

#include <imatistl.h>
#include <annotation.h>
#include <string>
#include <map>
#include <vector>

#include <KDTreeVectorOfVectorsAdaptor.h>
#include <../../DataStructures/include/graph.h>

class Annotation;
class ExtendedTrimesh : public IMATI_STL::TriMesh{

    public:
        const unsigned int BUFFER_SIZE = 1024;
        const double EPSILON = 1e-5;
        enum WeightType{UNIFORM, COTANGENT};


        typedef std::vector<std::vector<double> > my_vector_of_vectors_t;
        typedef KDTreeVectorOfVectorsAdaptor< my_vector_of_vectors_t, double > my_kd_tree_t;
        typedef double num_t;
    public:
        ExtendedTrimesh();
        ExtendedTrimesh(ExtendedTrimesh* m);
        ~ExtendedTrimesh();

        int load(std::string filename);
        IMATI_STL::Point computeLaplacian(unsigned int vertexId, WeightType type = WeightType::UNIFORM);
        void smoothMesh(unsigned int iterations_number, WeightType type = WeightType::UNIFORM);
        double getMinEdgeLength();
        IMATI_STL::Vertex* getPoint(unsigned long vertexID) const;
        void setPoint(unsigned long vertexID, IMATI_STL::Point* p);
        void setPoint(unsigned long vertexID, double x, double y, double z);
        IMATI_STL::Vertex* getClosestPoint(IMATI_STL::Point queryPt) const;
        std::vector<IMATI_STL::Vertex*> getClosestPoints(IMATI_STL::Point queryPt, unsigned int neighboursNumber) const;
        std::vector<IMATI_STL::Vertex*> getNeighboursInSphere(IMATI_STL::Point queryPt, double radius) const;
        IMATI_STL::Triangle* getTriangle(unsigned long triangleID) const;
        unsigned long getPointId(IMATI_STL::Vertex* v) const;
        unsigned long getTriangleId(IMATI_STL::Triangle* t) const;
        std::vector<unsigned int> findAnnotations(std::string tag);
        /**
         * @brief addAnnotation Adds an annotation to the mesh
         * @param annotation the annotation to add
         */
        void addAnnotation(Annotation* annotation);

        unsigned int getAnnotationId(Annotation*) const;

        /**
         * @brief removeAnnotation Removes an annotation from the mesh
         * @param annotation The annotation to remove
         */
        void removeAnnotation(Annotation* annotation);

        std::vector<Annotation *> getAnnotations() const;

        void setAnnotation(unsigned int pos,  Annotation* value);
        void setAnnotations(const std::vector<Annotation *> &value);

        void clearAnnotations();

        std::vector<IMATI_STL::Vertex*> cutMesh(std::vector<IMATI_STL::Point*> cut);

        std::string getFilename() const;

        int saveAnnotationsAsXYZ(const char *, bool ascii = 1);

        int saveXYZ(const char *, bool ascii = 1);

        bool getIsTemplate() const;
        void setIsTemplate(bool value);

        bool getIsCage() const;
        void setIsCage(bool value);

        GraphTemplate::Graph<Annotation *> *getGraph() const;
        void setGraph(GraphTemplate::Graph<Annotation *> *value);

        bool addAnnotationsRelationship(Annotation* a1, Annotation* a2, std::string relationshipType, double weight, bool directed = false, void *info = nullptr);

        std::map<unsigned long, IMATI_STL::Vertex *> getIdVertices() const;
        std::map<unsigned long, IMATI_STL::Triangle *> getIdTriangles() const;

        IMATI_STL::Vertex* getLowestVertex();
        IMATI_STL::Vertex* getLowestVertex(IMATI_STL::Point* direction);
        
		std::map<IMATI_STL::Vertex *, unsigned long> getVerticesId() const;

protected:

        std::map<IMATI_STL::Vertex*, unsigned long> verticesId;
        std::map<IMATI_STL::Triangle*, unsigned long> trianglesId;
        std::map<unsigned long, IMATI_STL::Vertex*> idVertices;
        std::map<unsigned long, IMATI_STL::Triangle*> idTriangles;
        std::vector<Annotation*> annotations;                       //List of annotations defined on the mesh
        GraphTemplate::Graph<Annotation *> *graph;

        my_vector_of_vectors_t points_vector;
        my_kd_tree_t* mat_index;


        std::string filename;                                       //Name of the file in which the mesh is stored
        double minEdgeLength;

        bool isTemplate;
        bool isCage;

        void buildInnerStructure();
        void init(ExtendedTrimesh *m);
};

#endif // EXTENDEDTRIMESH_H
