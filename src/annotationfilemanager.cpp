#include "annotationfilemanager.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <facet.h>
#include <geometricattribute.h>
#include <semanticattribute.h>
#include <surfaceannotation.h>
#include <lineannotation.h>
#include <pointannotation.h>
#include <drawableeuclideanmeasure.h>
#include <drawablegeodesicmeasure.h>
#include <drawableboundingmeasure.h>
#include <annotationutilities.h>
#include <drawableheightmeasure.h>
#include <rapidjson/writer.h>
#include <rapidjson/filereadstream.h>
#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>

using namespace std;
using namespace IMATI_STL;

AnnotationFileManager::AnnotationFileManager()
{
    mesh = nullptr;
}

bool AnnotationFileManager::writeAnnotations(std::string fileName)
{
    if(mesh != nullptr){
        string extension = fileName.substr(fileName.find_last_of(".") + 1);
        if(extension.compare("ant") == 0){
            vector<Annotation*> annotations = mesh->getAnnotations();
            ofstream annotationsFile;
            rapidjson::StringBuffer s;
            rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(s);
            writer.StartObject();
            writer.Key("annotations");
            writer.StartArray();
            for(unsigned int i = 0; i < annotations.size(); i++)
                annotations[i]->printJson(writer);
            writer.EndArray();
            writer.EndObject();
            annotationsFile.open(fileName);
            if(annotationsFile.is_open())
            {
                annotationsFile << s.GetString();
                annotationsFile.close();
            }
        } else if(extension.compare("triant") == 0){
            vector<Annotation*> annotations = mesh->getAnnotations();
            ofstream annotationsFile;
            rapidjson::StringBuffer s;
            rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(s);
            writer.StartObject();
            writer.Key("annotations");
            writer.StartArray();
            vector<Vertex*> lst;
            GraphTemplate::Graph<Annotation*>* graph = mesh->getGraph();
            for(unsigned int i = 0; i < annotations.size(); i++){
                writer.StartObject();
                SurfaceAnnotation* a = dynamic_cast<SurfaceAnnotation*>(annotations[i]);
                writer.Key("id");
                writer.Uint(a->getId());
                if(graph != nullptr){
                    writer.Key("father");
                    GraphTemplate::Node<Annotation*>* node = graph->getNodeFromData(a);
                    std::vector<GraphTemplate::Arc<Annotation*>*> enteringArcs = graph->getArcsFromTip(node);
                    bool isRoot = true;
                    for(unsigned int i = 0; i < enteringArcs.size(); i++){
                        std::size_t found = enteringArcs[i]->getLabel().find("Containment");
                        if (found != std::string::npos){
                            isRoot = false;
                            writer.Int(enteringArcs[i]->getN1()->getData()->getId());
                        }
                    }
                    if(isRoot)
                        writer.Int(-1);

                    writer.Key("level");
                    writer.Uint(a->getHierarchyLevel());
                }
                writer.Key("tag");
                writer.String(a->getTag().c_str());
//                writer.Key("level");
//                writer.Uint(a->getHierarchyLevel());
                writer.Key("color");
                writer.StartArray();
                writer.Int(a->getColor()[0]);
                writer.Int(a->getColor()[1]);
                writer.Int(a->getColor()[2]);
                writer.EndArray();
                writer.Key("triangles");

                std::vector<IMATI_STL::Triangle*> triangles = a->getTriangles();
                writer.StartArray();
                for(unsigned int i = 0; i < triangles.size(); i++){
                    writer.Int(mesh->T.numels() - mesh->getTriangleId(triangles[i]) - 1);
                }
                writer.EndArray();
                writer.EndObject();
            }
            writer.EndArray();
            writer.EndObject();
            annotationsFile.open(fileName);
            annotationsFile << s.GetString();
            annotationsFile.close();
        } else if (extension.compare("fct") == 0){

            FacetWriter fw;
            vector<Facet> facets;
            fw.setFilename(this->mesh->getFilename());
            string facetFilename(fileName);
            char id = 'A';
            vector<Annotation*> annotations = mesh->getAnnotations();
            for(vector<Annotation*>::iterator it = annotations.begin(); it != annotations.end(); it++){
                Facet f;
                Annotation* a = static_cast<Annotation*>(*it);
                vector<vector<Vertex*> > outlines = dynamic_cast<SurfaceAnnotation*>(a)->getOutlines();
                if (a->getTag().compare("Internal") == 0)
                    f.type = Facet::FacetType::INTERNAL;
                else if(a->getTag().compare("External") == 0)
                    f.type = Facet::FacetType::EXTERNAL;
                else if(a->getTag().compare("Fracture") == 0)
                    f.type = Facet::FacetType::FRACTURE;
                else
                    f.type = Facet::FacetType::UNKNOWN;

                for(vector<vector<Vertex*> >::iterator oit = outlines.begin(); oit != outlines.end(); oit++){
                    vector<Vertex*> outline = static_cast<vector<Vertex*> >(*oit);
                    vector<int> boundaryVertices;
                    for(vector<Vertex*>::iterator bit = outline.begin(); bit != outline.end(); bit++)
                        boundaryVertices.push_back(static_cast<int>(mesh->getPointId(static_cast<Vertex*>(*bit))));
                    f.boundaries.push_back(boundaryVertices);
                }

                vector<Triangle*> innerTriangles = Utilities::regionGrowing(outlines);
                vector<Vertex*> innerVertices;
                Utilities::findPoints(innerVertices, innerTriangles, outlines);
                for(vector<Vertex*>::iterator vit = innerVertices.begin(); vit != innerVertices.end(); vit++)
                    f.innerVertices.push_back(static_cast<int>(mesh->getPointId(static_cast<Vertex*>(*vit))));
                f.label.push_back(id++);
                facets.push_back(f);
            }
            fw.setFacets(facets);
            fw.writeFct(facetFilename);
        }else if(extension.compare("m") == 0){
            ofstream matrixFile;
            matrixFile.open(fileName);
            for(unsigned int i = 0; i < mesh->getAnnotations().size(); i++)
            {
                Annotation* a = mesh->getAnnotations()[i];
                std::vector<Vertex*> involvedVertices;
                std::vector<int> isBoundary;
                switch (a->getType()) {
                    case AnnotationType::Point:
                    {
                        PointAnnotation* p = dynamic_cast<PointAnnotation*>(a);
                        involvedVertices = p->getInvolvedVertices();
                        isBoundary.resize(involvedVertices.size());
                        fill(isBoundary.begin(), isBoundary.end(), 1);
                        break;
                    }
                    case AnnotationType::Line:
                    {
                        LineAnnotation* l = dynamic_cast<LineAnnotation*>(a);
                        involvedVertices = l->getInvolvedVertices();
                        isBoundary.resize(involvedVertices.size());
                        fill(isBoundary.begin(), isBoundary.end(), 1);
                        break;
                    }
                    case AnnotationType::Surface:
                    {
                        SurfaceAnnotation* s = dynamic_cast<SurfaceAnnotation*>(a);
                        involvedVertices = s->getInvolvedVertices();
                        isBoundary = s->getIsBoundaryStatus();
                        break;
                    }
                    default:
                        exit(11);

                }
                matrixFile << "Mesh: [";
                for(IMATI_STL::Node* n = mesh->V.head(); n != nullptr; n = n->next())
                {
                    Vertex* v = static_cast<Vertex*>(n->data);
                    matrixFile << "\t" << v->x << " " << v->y << " " << v->z << std::endl;
                }
                matrixFile << "];" << std::endl << std::endl;
                matrixFile << "A" << i << ":" << std::endl << "[" << std::endl << "\t";
                for(unsigned int j = 0; j < involvedVertices.size(); j++)
                {
                    matrixFile << "\t" << mesh->getPointId(involvedVertices[j]) << std::endl;
                }
                matrixFile << "]" << std::endl << "B" << i << ":" << std::endl << "[" << std::endl;
                for(unsigned int j = 0; j < isBoundary.size(); j++)
                {
                    matrixFile <<  "\t" << isBoundary[j] << std::endl;
                }
                matrixFile << "]" << std::endl << std::endl;
            }
        } else
            return false;
        return true;
    }else
        return false;
}

bool AnnotationFileManager::readAnnotations(string fileName)
{

    if(mesh != nullptr && mesh->V.numels()){
        string extension = fileName.substr(fileName.find_last_of(".") + 1);
        if(extension.compare("fct") == 0){
            FacetReader fr;
            if(fr.readFct(fileName)){

                std::vector<Facet> facets = fr.getFacets();

                for(std::vector<Facet>::iterator it = facets.begin(); it != facets.end(); it++){
                    vector<vector<int> > boundaryVertices = it->boundaries;
                    vector<int> innerVerticesIndices = it->innerVertices;

                    Annotation* a = new SurfaceAnnotation();
                    for(vector<vector<int> >::iterator oit = boundaryVertices.begin(); oit != boundaryVertices.end(); oit++){
                        vector<int> facetOutline = static_cast<vector<int> >(*oit);
                        vector<Vertex*> outline;
                        for(unsigned int i = 0; static_cast<unsigned int>(i < facetOutline.size()); i++){
                            Vertex* v = mesh->getPoint(static_cast<unsigned long>(facetOutline[i]));
                            outline.push_back(v);
                        }
                        Utilities::fixOutlineOrderToCounterclockwise(innerVerticesIndices, outline, mesh);
                        dynamic_cast<SurfaceAnnotation*>(a)->addOutline(outline);
                    }
                    unsigned char color[3];
                    string tag;
                    switch (it->type) {
                    case Facet::FacetType::INTERNAL:
                        color[0] = 255;
                        color[1] = 0;
                        color[2] = 0;
                        tag = "Internal";
                        break;
                    case Facet::FacetType::EXTERNAL:
                        color[0] = 0;
                        color[1] = 255;
                        color[2] = 0;
                        tag = "External";
                        break;
                    case Facet::FacetType::FRACTURE:
                        color[0] = 0;
                        color[1] = 0;
                        color[2] = 255;
                        tag = "Fracture";
                        break;
                    default:
                        color[0] = 0;
                        color[1] = 0;
                        color[2] = 0;
                        tag = "Unknown";
                        break;
                    }

                    a->setColor(color);
                    a->setTag(tag);
                    a->setMesh(mesh);
                    mesh->addAnnotation(a);
                }
            }
        }
        else if(extension.compare("ant") == 0){
            FILE* fp = fopen(fileName.c_str(),"r");
            char buffer[BUFFER_SIZE];
            rapidjson::FileReadStream frs(fp, buffer, sizeof (buffer));

            rapidjson::Document document;
            if(!(document.ParseStream(frs).HasParseError())){
                if(document.HasMember("annotations") && document["annotations"].IsArray()){
                    rapidjson::Value& annotationsList = document["annotations"];
                    for (rapidjson::SizeType i = 0; i < annotationsList.Size(); i++) // rapidjson uses SizeType instead of size_t.
                    {
                        rapidjson::Value& jsonAnnotation = annotationsList[i];
                        if(jsonAnnotation.IsObject()){
                            assert(jsonAnnotation.HasMember("type"));
                            assert(jsonAnnotation["type"].IsString());
                            string type = jsonAnnotation["type"].GetString();

                            Annotation* annotation;

                            if(type.compare("Area") == 0)
                                annotation = new SurfaceAnnotation();
                            else if(type.compare("Line") == 0)
                                annotation = new LineAnnotation();
                            else if(type.compare("Point") == 0)
                                annotation = new PointAnnotation();
                            else
                                return false;

                            annotation->setMesh(mesh);
                            assert(jsonAnnotation.HasMember("id"));
                            assert(jsonAnnotation["id"].IsUint());
                            annotation->setId(jsonAnnotation["id"].GetUint());

                            assert(jsonAnnotation.HasMember("tag"));
                            assert(jsonAnnotation["tag"].IsString());
                            annotation->setTag(jsonAnnotation["tag"].GetString());

//                            assert(jsonAnnotation.HasMember("level"));
//                            assert(jsonAnnotation["level"].IsUint());
//                            annotation->setHierarchyLevel(jsonAnnotation["level"].GetUint());

                            unsigned char color[3];
                            if(jsonAnnotation.HasMember("color")){
                                rapidjson::Value& jsonColor = jsonAnnotation["color"];
                                assert(jsonColor.IsArray());
                                assert(jsonColor[0].IsInt() && jsonColor[1].IsInt() && jsonColor[2].IsInt());
                                color[0] = static_cast<unsigned char>(jsonColor[0].GetInt());
                                color[1] = static_cast<unsigned char>(jsonColor[1].GetInt());
                                color[2] = static_cast<unsigned char>(jsonColor[2].GetInt());
                            }else {
                                color[0] = 0;
                                color[1] = 0;
                                color[2] = 0;
                            }

                            annotation->setColor(color);

                            if(type.compare("Area") == 0){
                                assert(jsonAnnotation.HasMember("boundaries"));
                                assert(jsonAnnotation["boundaries"].IsArray());
                                rapidjson::Value& boundaries = jsonAnnotation["boundaries"];
                                for(rapidjson::SizeType j = 0; j < boundaries.Size(); j++){
                                    rapidjson::Value& boundary = boundaries[j];
                                    vector<Vertex*> outline;
                                    assert(boundary.IsArray());
                                    for(rapidjson::SizeType k = 0; k < boundary.Size(); k++){
                                        rapidjson::Value& pointID = boundary[k];
                                        assert(pointID.IsInt());
                                        Vertex* v = mesh->getPoint(static_cast<unsigned long>(pointID.GetInt()));
                                        outline.push_back(v);
                                    }
                                    dynamic_cast<SurfaceAnnotation*>(annotation)->addOutline(outline);
                                }

                            } else if(type.compare("Line") == 0){
                                assert(jsonAnnotation.HasMember("polylines"));
                                assert(jsonAnnotation["polylines"].IsArray());
                                rapidjson::Value& polyLines = jsonAnnotation["polylines"];
                                for(rapidjson::SizeType j = 0; j < polyLines.Size(); j++){
                                    rapidjson::Value& jsonPolyLine = polyLines[j];
                                    vector<Vertex*> polyLine;
                                    assert(jsonPolyLine.IsArray());
                                    for(rapidjson::SizeType k = 0; k < jsonPolyLine.Size(); k++){
                                        rapidjson::Value& pointID = jsonPolyLine[k];
                                        assert(pointID.IsInt());
                                        Vertex* v = mesh->getPoint(static_cast<unsigned long>(pointID.GetInt()));
                                        polyLine.push_back(v);
                                    }
                                    dynamic_cast<LineAnnotation*>(annotation)->addPolyLine(polyLine);
                                }
                            } else if(type.compare("Point") == 0){
                                assert(jsonAnnotation.HasMember("points"));
                                assert(jsonAnnotation["points"].IsArray());
                                rapidjson::Value& points = jsonAnnotation["points"];
                                for(rapidjson::SizeType j = 0; j < points.Size(); j++){
                                    rapidjson::Value& pointID = points[j];
                                    assert(pointID.IsInt());
                                    Vertex* v = mesh->getPoint(static_cast<unsigned long>(pointID.GetInt()));
                                    dynamic_cast<PointAnnotation*>(annotation)->addPoint(v);
                                }

                            }

                            assert(jsonAnnotation.HasMember("attributes"));
                            assert(jsonAnnotation["attributes"].IsArray());
                            rapidjson::Value& attributes = jsonAnnotation["attributes"];

                            for(rapidjson::SizeType j = 0; j < attributes.Size(); j++){
                                rapidjson::Value& jsonAttribute = attributes[j];
                                assert(jsonAttribute.IsObject());
                                assert(jsonAttribute.HasMember("id"));
                                assert(jsonAttribute.HasMember("type"));
                                assert(jsonAttribute.HasMember("name"));
                                assert(jsonAttribute.HasMember("value"));
                                assert(jsonAttribute["type"].IsString());
                                assert(jsonAttribute["name"].IsString());
                                unsigned int attributeID = jsonAttribute["id"].GetUint();
                                string attributeType = jsonAttribute["type"].GetString();
                                string attributeName = jsonAttribute["name"].GetString();
                                Attribute* attribute;

                                if(attributeType.compare("Geometric") == 0){
                                    assert(jsonAttribute.HasMember("tool"));
                                    std::string tool = jsonAttribute["tool"].GetString();
                                    if(tool.compare("ruler") == 0)
                                    {
                                        attribute = new DrawableEuclideanMeasure();
                                    } else if(tool.compare("tape") == 0)
                                    {
                                        attribute = new DrawableGeodesicMeasure();
                                    } else if(tool.compare("bounding") == 0)
                                    {
                                        attribute = new DrawableBoundingMeasure();
                                        assert(jsonAttribute.HasMember("direction"));
                                        rapidjson::Value& directionVector = jsonAttribute["direction"];
                                        IMATI_STL::Point* d = new IMATI_STL::Point(directionVector[0].GetDouble(),
                                                                                   directionVector[1].GetDouble(),
                                                                                   directionVector[2].GetDouble());
                                        IMATI_STL::Point* o = new IMATI_STL::Point(0,0,0);
                                        std::vector<IMATI_STL::Vertex*> involved = annotation->getInvolvedVertices();
                                        for(unsigned int k = 0; k < involved.size(); k++)
                                            (*o) += *(involved[k]);
                                        (*o) /= involved.size();

                                        dynamic_cast<DrawableBoundingMeasure*>(attribute)->setOrigin(o);
                                        dynamic_cast<DrawableBoundingMeasure*>(attribute)->setDirection(d);
                                    } else if(tool.compare("height") == 0)
                                    {
                                        attribute = new DrawableHeightMeasure();
                                        assert(jsonAttribute.HasMember("direction"));
                                        rapidjson::Value& directionVector = jsonAttribute["direction"];
                                        IMATI_STL::Point* d = new IMATI_STL::Point(directionVector[0].GetDouble(),
                                                                                   directionVector[1].GetDouble(),
                                                                                   directionVector[2].GetDouble());
                                        IMATI_STL::Point* o = new IMATI_STL::Point(0,0,0);
                                        std::vector<IMATI_STL::Vertex*> involved = annotation->getInvolvedVertices();
                                        for(unsigned int k = 0; k < involved.size(); k++)
                                            (*o) += *(involved[k]);
                                        (*o) /= involved.size();

                                        dynamic_cast<DrawableHeightMeasure*>(attribute)->setDirection(d);
                                    } else
                                        exit(11);
                                    double value = jsonAttribute["value"].GetDouble();
                                    attribute->setValue(value);
                                    assert(jsonAttribute.HasMember("points"));
                                    assert(jsonAttribute["points"].IsArray());
                                    rapidjson::Value& points = jsonAttribute["points"];
                                    for(unsigned int k = 0; k < points.Size(); k++)
                                        dynamic_cast<GeometricAttribute*>(attribute)->addMeasurePointID(points[k].GetUint());
                                    dynamic_cast<DrawableAttribute*>(attribute)->setMesh(static_cast<DrawableMesh*>(mesh));
                                    dynamic_cast<DrawableAttribute*>(attribute)->update();
                                } else if(attributeType.compare("Semantic") == 0){
                                    attribute = new SemanticAttribute();
                                    string value = jsonAttribute["value"].GetString();
                                    attribute->setValue(value);

                                } else {
                                    assert(false);
                                }

                                attribute->setId(attributeID);
                                attribute->setKey(attributeName);
                                annotation->addAttribute(attribute);

                            }
                            annotation->setMesh(mesh);
                            mesh->addAnnotation(annotation);
                        }else
                            return false;
                    }
                }else
                    return false;
            }else
                return false;
        } else if (extension.compare("triant") == 0){

            FILE* fp = fopen(fileName.c_str(),"r");
            char buffer[BUFFER_SIZE];
            rapidjson::FileReadStream frs(fp, buffer, sizeof (buffer));

            rapidjson::Document document;
            if(!(document.ParseStream(frs).HasParseError())){
                if(document.HasMember("annotations") && document["annotations"].IsArray()){
                    rapidjson::Value& annotationsList = document["annotations"];
                    for (rapidjson::SizeType i = 0; i < annotationsList.Size(); i++) // rapidjson uses SizeType instead of size_t.
                    {
                        rapidjson::Value& jsonAnnotation = annotationsList[i];
                        if(jsonAnnotation.IsObject()){

                            SurfaceAnnotation* annotation = new SurfaceAnnotation();
                            assert(jsonAnnotation.HasMember("tag"));
                            assert(jsonAnnotation["tag"].IsString());
                            annotation->setTag(jsonAnnotation["tag"].GetString());

//                            assert(jsonAnnotation.HasMember("level"));
//                            assert(jsonAnnotation["level"].IsUint());
//                            annotation->setHierarchyLevel(jsonAnnotation["level"].GetUint());

                            unsigned char color[3];
                            if(jsonAnnotation.HasMember("color")){
                                rapidjson::Value& jsonColor = jsonAnnotation["color"];
                                assert(jsonColor.IsArray());
                                assert(jsonColor[0].IsInt() && jsonColor[1].IsInt() && jsonColor[2].IsInt());
                                color[0] = static_cast<unsigned char>(jsonColor[0].GetInt());
                                color[1] = static_cast<unsigned char>(jsonColor[1].GetInt());
                                color[2] = static_cast<unsigned char>(jsonColor[2].GetInt());
                            }else {
                                color[0] = 0;
                                color[1] = 0;
                                color[2] = 0;
                            }

                            annotation->setColor(color);

                            assert(jsonAnnotation.HasMember("triangles"));
                            assert(jsonAnnotation["triangles"].IsArray());
                            rapidjson::Value& jsonTriangles = jsonAnnotation["triangles"];
                            std::vector<IMATI_STL::Triangle*> triangles;
                            for(rapidjson::SizeType j = 0; j < jsonTriangles.Size(); j++){
                                IMATI_STL::Vertex* v = mesh->getTriangle(mesh->T.numels() - jsonTriangles[j].GetInt() - 1)->v1();
                                for(unsigned int i = 0; i < 3; i++)
                                    v = mesh->getTriangle(mesh->T.numels() - jsonTriangles[j].GetInt() - 1)->nextVertex(v);
                                triangles.push_back(mesh->getTriangle(mesh->T.numels() - jsonTriangles[j].GetInt() - 1));
                            }

                            annotation->setOutlines(Utilities::getOutlines(triangles));

                            annotation->setMesh(mesh);
                            mesh->addAnnotation(annotation);
                        }else
                            return false;
                    }
                }else
                    return false;
            }else
                return false;
        }
    }else
        return false;

    return true;
}

ExtendedTrimesh *AnnotationFileManager::getMesh() const
{
    return mesh;
}

void AnnotationFileManager::setMesh(ExtendedTrimesh *value)
{
    mesh = value;
}

