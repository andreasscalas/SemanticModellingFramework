#ifndef ANNOTATION_H
#define ANNOTATION_H

#include <vector>
#include <string>
#include <ostream>
#include <rapidjson/prettywriter.h>
#include <rapidjson/document.h>
#include <extendedtrimesh.h>
#include <attribute.h>

class ExtendedTrimesh;

enum class AnnotationType{
    Volume,
    Surface,
    Line,
    Point
};


class Annotation {

    public:

        Annotation(){ color = static_cast<unsigned char*>(std::malloc(3 * sizeof(unsigned char)));}
        /**
         * @brief transfer This method takes the annotations of an object defined on
         * a model with a certain resolution and transfers it to a model with another resolution
         * @param otherMesh The model with lower resolution
         * @param metric the metric to be used for the shortest path
         * @return The annotation defined on the other model.
         */
        virtual Annotation* transfer(ExtendedTrimesh* otherMesh, short metric = 2) = 0;

        /**
         * @brief parallelTransfer This method takes the annotations of an object defined on a model with
         * a certain resolution and transfers it to a model with another resolution using multiple threads
         * @param otherMesh The model with lower resolution
         * @param metric the metric to be used for the shortest path
         * @return The annotation defined on the other model.
         */
        virtual Annotation* parallelTransfer(ExtendedTrimesh* otherMesh, short metric = 2) = 0;

        virtual void print(std::ostream& os);

        virtual void printJson(rapidjson::PrettyWriter<rapidjson::StringBuffer> &writer);

        virtual std::vector<IMATI_STL::Vertex*> getInvolvedVertices() = 0;

        virtual bool isPointInAnnotation(IMATI_STL::Vertex* p) = 0;

        //Getter and setter methods
        ExtendedTrimesh *getMesh() const{ return mesh; }
        void setMesh(ExtendedTrimesh *value){ this->mesh = value; }
        AnnotationType getType() const { return type; }
        std::string getTag() const { return tag; }
        void setTag(const std::string &value) { tag = value; }
        unsigned char* getColor(){ return this->color; }
        void setColor(unsigned char value[3]) { this->color[0] = value[0]; this->color[1] = value[1]; this->color[2] = value[2]; }
        std::vector<Attribute *> getAttributes() const { return attributes; }
        void setAttributes(const std::vector<Attribute *> &value) { attributes = value; }
        void addAttribute(Attribute* value){ attributes.push_back(value); }
        unsigned int getHierarchyLevel() const { return hierarchyLevel; }
        void setHierarchyLevel(unsigned int value) { hierarchyLevel = value; }
        unsigned int getId() const { return id; }
        void setId(unsigned int value){ id = value; }
        virtual IMATI_STL::Point* getCenter() = 0;
        virtual IMATI_STL::Point* getOrientation() = 0;

protected:
        ExtendedTrimesh* mesh;
        AnnotationType type;
        unsigned int id;
        std::string tag;                                        //The tag of the annotation
        unsigned int hierarchyLevel;
        std::vector<Attribute*> attributes;
        unsigned char* color;                                   //The color associated to the annotation
        const unsigned short NUM_OF_THREADS = 8;                //Number of threads used for the transfer procedure
        double sphereRay;                                       //The radius of the sphere for the neighborhood search

};

#endif // ANNOTATION_H

