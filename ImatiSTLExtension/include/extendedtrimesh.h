#ifndef EXTENDEDTRIMESH_H
#define EXTENDEDTRIMESH_H

#include <imatistl.h>
#include <annotation.h>
#include <string>
#include <map>
#include <vector>
#include <../../DataStructures/include/graph.h>

class Annotation;
class ExtendedTrimesh : public IMATI_STL::TriMesh{

    public:
        const unsigned int BUFFER_SIZE = 1024;
    public:
        ExtendedTrimesh();
        ExtendedTrimesh(ExtendedTrimesh* m);
        ~ExtendedTrimesh();

        int load(std::string filename);
        double getMinEdgeLength();
        IMATI_STL::Vertex* getPoint(unsigned long triangleID) const;
        IMATI_STL::Triangle* getTriangle(unsigned long triangleID) const;
        unsigned long getPointId(IMATI_STL::Vertex* v) const;
        unsigned long getTriangleId(IMATI_STL::Triangle* t) const;
        std::vector<unsigned int> findAnnotations(std::string tag);
        /**
         * @brief addAnnotation Adds an annotation to the mesh
         * @param annotation the annotation to add
         */
        void addAnnotation(Annotation* annotation);

        /**
         * @brief removeAnnotation Removes an annotation from the mesh
         * @param annotation The annotation to remove
         */
        void removeAnnotation(Annotation* annotation);

        std::vector<Annotation *> getAnnotations() const;

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

protected:

        std::map<IMATI_STL::Vertex*, unsigned long> verticesId;
        std::map<IMATI_STL::Triangle*, unsigned long> trianglesId;
        std::map<unsigned long, IMATI_STL::Vertex*> idVertices;
        std::map<unsigned long, IMATI_STL::Triangle*> idTriangles;
        std::vector<Annotation*> annotations;                       //List of annotations defined on the mesh
        GraphTemplate::Graph<Annotation *> *graph;
        std::string filename;                                       //Name of the file in which the mesh is stored
        double minEdgeLength;

        bool isTemplate;
        bool isCage;

        void buildInnerStructure();
        void init(ExtendedTrimesh *m);
};

#endif // EXTENDEDTRIMESH_H
