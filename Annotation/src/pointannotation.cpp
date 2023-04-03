#include "pointannotation.h"
#include <Eigen/Dense>

PointAnnotation::PointAnnotation()
{
    type = AnnotationType::Point;
}

PointAnnotation::~PointAnnotation()
{
    this->points.clear();
}

Annotation *PointAnnotation::transfer(ExtendedTrimesh *otherMesh, short metric)
{
    PointAnnotation* otherAnnotation = new PointAnnotation();
    std::vector<IMATI_STL::Vertex*> otherPoints;
    unsigned int ALREADY_USED = 18;
    for(unsigned int i = 0; i < points.size(); i++){
        double bestDistance = DBL_MAX;
        IMATI_STL::Vertex* best;
        for (IMATI_STL::Node* n = otherMesh->V.head(); n != nullptr; n = n->next()){
            IMATI_STL::Vertex* tmp = static_cast<IMATI_STL::Vertex*>(n->data);
            double distance = ((*tmp) - (*points[i])).length();
            if(distance < bestDistance){
                best = tmp;
                bestDistance = distance;
            }
        }
        if(best->info == nullptr || *static_cast<unsigned int*>(best->info) != ALREADY_USED)
        {
            otherPoints.push_back(best);
            best->info = &ALREADY_USED;
        }
    }

    otherAnnotation->setId(this->id);
    otherAnnotation->setPoints(otherPoints);
    otherAnnotation->setMesh(otherMesh);
    otherAnnotation->setTag(this->tag);
    otherAnnotation->setColor(this->color);
    otherAnnotation->setHierarchyLevel(this->hierarchyLevel);
    return otherAnnotation;

}

Annotation *PointAnnotation::parallelTransfer(ExtendedTrimesh *otherMesh, short metric)
{
    return transfer(otherMesh, metric);
}

void PointAnnotation::print(std::ostream &os)
{
    Annotation::print(os);
    os << "Annotation on points: [" << std::endl;
    for(unsigned int i = 0; i < points.size(); i++){
        os << "(" << points[i]->x << "," << points[i]->y << "," << points[i]->z << ")";
        if(i != points.size() - 1)
            os <<",";
        os << std::endl;
    }
}

void PointAnnotation::printJson(rapidjson::PrettyWriter<rapidjson::StringBuffer> &writer)
{
    writer.StartObject();
    Annotation::printJson(writer);
    writer.Key("type");
    writer.String("Point");
    writer.Key("points");
    writer.StartArray();
    for(unsigned int i = 0; i < points.size(); i++)
        writer.Int(static_cast<int>(mesh->getPointId(points[i])));
    writer.EndArray();
    writer.EndObject();
}

std::vector<T_MESH::Vertex *> PointAnnotation::getInvolvedVertices()
{
    return points;
}

bool PointAnnotation::isPointInAnnotation(T_MESH::Vertex *v)
{
    for(unsigned int i = 0; i < points.size(); i++)
        if(points[i] == v)
            return true;
    return false;
}

std::vector<IMATI_STL::Vertex *> PointAnnotation::getPoints() const
{
    return points;
}

void PointAnnotation::setPoints(const std::vector<IMATI_STL::Vertex *> &value)
{
    points = value;
}

void PointAnnotation::addPoint(T_MESH::Vertex *value)
{
    this->points.push_back(value);
}

IMATI_STL::Point *PointAnnotation::getCenter()
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

IMATI_STL::Point *PointAnnotation::getOrientation()
{
    std::vector<IMATI_STL::Vertex*> involvedVertices = getInvolvedVertices();
    Eigen::MatrixXd eigenPoints;
    eigenPoints.resize(3, static_cast<Eigen::Index>(involvedVertices.size()));

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
