#include "lineannotation.h"
#include <utilities.h>
#include <annotationutilities.h>

using namespace std;
using namespace IMATI_STL;

LineAnnotation::LineAnnotation()
{
    type = AnnotationType::Line;
}

LineAnnotation::~LineAnnotation()
{
    polyLines.clear();
}

Annotation *LineAnnotation::transfer(ExtendedTrimesh *otherMesh, short metric)
{
    /************************************INIZIO WORK IN PROGRESS************************/
    LineAnnotation* otherAnnotation = new LineAnnotation(); //The transferred annotation
    Vertex* v, *initialVertex;                      //Some support variable

    sphereRay = otherMesh->bboxLongestDiagonal() / BBOX_SPHERE_RATIO;
    for(Node* n = otherMesh->V.head(); n != nullptr; n = n->next())
        static_cast<Vertex*>(n->data)->info = nullptr;
    for(vector<vector<Vertex*> >::iterator lit = polyLines.begin(); lit != polyLines.end(); lit++){

        std::vector<Vertex*> otherPolyLine;
        vector<Vertex*> polyLine = static_cast<vector<Vertex*> >(*lit);
        std::vector<Vertex*>::iterator vit = polyLine.begin();
        Vertex* v1, *v2;

        do{
            v = static_cast<Vertex*>(*vit);
            vector<Vertex*> neighbors = otherMesh->getNeighboursInSphere(*v, sphereRay);
            vector<Triangle*> toCheckTriangles;
            Utilities::findFaces(toCheckTriangles, neighbors);
            v1 = Utilities::findCorrespondingVertex(v, toCheckTriangles);
            if(v1 == nullptr)
                sphereRay *= 2;
        }while(v1 == nullptr);
        initialVertex = v1;

        for(; vit != polyLine.end(); vit++){
            do{
                v = static_cast<Vertex*>(*vit);
                vector<Vertex*> neighbors = otherMesh->getNeighboursInSphere(*v, sphereRay);
                vector<Triangle*> toCheckTriangles;
                Utilities::findFaces(toCheckTriangles, neighbors);
                v2 = Utilities::findCorrespondingVertex(v, toCheckTriangles);
                if(v2 == nullptr)
                    sphereRay *= 2;
            }while(v2 == nullptr);
            vector<Vertex*> path = Utilities::dijkstra(v1, v2, metric, false);
            otherPolyLine.insert(otherPolyLine.end(), path.begin(), path.end());
            v1 = v2;
        }

        for(unsigned int i = 0; i < otherPolyLine.size(); i++)
            otherPolyLine[i]->info = nullptr;

        otherAnnotation->addPolyLine(otherPolyLine);


        /************************FINE WORK IN PROGRESS*****************************/
    }

    //The Outline and inner vertex have been found, the tag and color are the same, so the process ends.
    otherAnnotation->setId(this->id);
    otherAnnotation->setMesh(otherMesh);
    otherAnnotation->setTag(this->tag);
    otherAnnotation->setColor(this->color);
    otherAnnotation->setHierarchyLevel(this->hierarchyLevel);

    return otherAnnotation;


}

Annotation *LineAnnotation::parallelTransfer(ExtendedTrimesh *otherMesh, short metric)
{
    return nullptr;
}

void LineAnnotation::print(ostream &os)
{
    Annotation::print(os);
    os << "type: Line" << std::endl << "PolyLines: [" << std::endl;
    for(unsigned int i = 0; i < polyLines.size(); i++){
        os << "[ " ;
        for(unsigned j = 0; j < polyLines[i].size(); j++){
            os << mesh->getPointId(polyLines[i][j]);
            if(j < polyLines[i].size() - 1)
                os << ", ";
        }
        os << " ]," << std::endl;
    }
    os << "]" << std::endl;
}

void LineAnnotation::printJson(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
    writer.StartObject();
    Annotation::printJson(writer);
    writer.Key("type");
    writer.String("Line");
    writer.Key("polylines");
    writer.StartArray();
    for(unsigned int i = 0; i < polyLines.size(); i++){
        writer.StartArray();
        for(unsigned j = 0; j < polyLines[i].size(); j++)
            writer.Int(static_cast<int>(mesh->getPointId(polyLines[i][j])));
        writer.EndArray();
    }
    writer.EndArray();
    writer.EndObject();
}

void LineAnnotation::addPolyLine(std::vector<Vertex *> &value)
{
    this->polyLines.push_back(value);
}

std::vector<std::vector<IMATI_STL::Vertex *> > LineAnnotation::getPolyLines() const
{
    return polyLines;
}

void LineAnnotation::setPolyLines(const std::vector<std::vector<IMATI_STL::Vertex *> > &value)
{
    polyLines = value;
}

void LineAnnotation::clearPolylines()
{
    polyLines.clear();
}

std::vector<Vertex *> LineAnnotation::getInvolvedVertices()
{
    vector<Vertex*> vertices;
    for (unsigned int i = 0; i < polyLines.size(); i++)
        vertices.insert(vertices.end(), polyLines[i].begin(), polyLines[i].end());

    return vertices;
}

bool LineAnnotation::isPointInAnnotation(Vertex *p)
{
    vector<Vertex*> involved = getInvolvedVertices();
    for(unsigned int i = 0; i < involved.size(); i++){
        if(involved[i] == p)
            return true;
    }

    return false;
}


IMATI_STL::Point *LineAnnotation::getCenter()
{
    std::vector<IMATI_STL::Vertex*> involvedVertices = getInvolvedVertices();
    IMATI_STL::Point* center = new IMATI_STL::Point(0,0,0);
    for(unsigned int i = 0; i < involvedVertices.size(); i++ )
    {
        *center += *(involvedVertices[i]);
    }

    *center /= involvedVertices.size();

    return center;
}

IMATI_STL::Point *LineAnnotation::getOrientation()
{
    std::vector<IMATI_STL::Vertex*> involvedVertices = getInvolvedVertices();
    Eigen::MatrixXd eigenPoints;
    eigenPoints.resize(3, involvedVertices.size());

    for(unsigned int i = 0; i < involvedVertices.size(); i++ )
    {
        Eigen::Vector3d p = {involvedVertices[i]->x, involvedVertices[i]->y, involvedVertices[i]->z};
        eigenPoints.col(i) = p;
    }

    Eigen::Vector3d mean_vector = eigenPoints.rowwise().mean();
    eigenPoints.colwise() -= mean_vector;
    Eigen::Matrix3d U = eigenPoints.bdcSvd(Eigen::ComputeFullU | Eigen::ComputeThinV).matrixU();
    IMATI_STL::Point* orientation = new IMATI_STL::Point();
    orientation->setValue(U(0,2), U(1,2), U(2,2));
    return orientation;
}
