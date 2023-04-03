#include "maarcpath.h"
#include <imatistl.h>
//#include <utilities.h>

using namespace AndreasStructures;

MAArcPath::MAArcPath()
{
    isTerminalPath = false;
}

void MAArcPath::addPoint(IMATI_STL::Point* point)
{
    this->path.push_back(point);

}

std::vector<IMATI_STL::Point*> MAArcPath::getPath() const
{
    return path;
}

void MAArcPath::setPath(const std::vector<IMATI_STL::Point*> &value)
{
    path = value;
}

AndreasStructures::MANode *MAArcPath::getN1() const
{
    return n1;
}

void MAArcPath::setN1(AndreasStructures::MANode *value)
{
    n1 = value;
}

AndreasStructures::MANode *MAArcPath::getN2() const
{
    return n2;
}

void MAArcPath::setN2(AndreasStructures::MANode *value)
{
    n2 = value;
}

void MAArcPath::addTriangle(T_MESH::Triangle* triangle)
{
    this->traversedTriangles.push_back(triangle);
}

std::vector<IMATI_STL::Triangle*> MAArcPath::getTraversedTriangles() const
{
    return traversedTriangles;
}

void MAArcPath::setTraversedTriangles(const std::vector<IMATI_STL::Triangle*> &value)
{
    traversedTriangles = value;
}

bool MAArcPath::getIsTerminalPath() const
{
    return isTerminalPath;
}

void MAArcPath::setIsTerminalPath(bool value)
{
    isTerminalPath = value;
}

void MAArcPath::computeImportance()
{
    double area = 0;
    double length = 0;
    double curvature = 0;
    IMATI_STL::Point t(5.75903, 0 , -4.04732);
    std::vector<IMATI_STL::Vertex*> localBoundary;
    IMATI_STL::Vertex* v = nullptr;
    for(unsigned int j = 0; j < traversedTriangles.size(); j++)
        area += traversedTriangles[j]->area();
    if(n1->getIsTerminalNode()){
        if((*n1->getPoint() - t).length() < 0.05){
            int a = 0;
            int b = 0;
        }
        v = static_cast<IMATI_STL::Vertex*>(n1->getPoint());
        area += n1->getTriangle()->area();
        length += ((*n1->getPoint()) - (*path.front())).length();
    }
    if(n1 != n2 && n2->getIsTerminalNode()){
        if((*n2->getPoint() - t).length() < 0.05){
            int a = 0;
            int b = 0;
        }
        v = static_cast<IMATI_STL::Vertex*>(n2->getPoint());
        area += n2->getTriangle()->area();
        length += ((*n2->getPoint()) - (*path.back())).length();
    }
    if(path.size() > 0){
        for(unsigned int j = 1; j < path.size(); j++)
            length += (*path[j] - *(path[j - 1])).length();
    }else if(n1 != n2 && (n1->getIsTerminalNode() || n2->getIsTerminalNode()))
        length += ((*n1->getPoint()) - (*n2->getPoint())).length();

    /*if(v != nullptr){

        IMATI_STL::Vertex* tmp = v;
        double minEdgeLength = DBL_MAX;
        do{
            tmp = tmp->nextOnBoundary();
            if(tmp->getEdge(tmp->prevOnBoundary())->length() < minEdgeLength )
                minEdgeLength = tmp->getEdge(tmp->prevOnBoundary())->length();
        }while(tmp != v);

        curvature = Utilities::computeLocalCurvature2D(v, minEdgeLength, LOCAL_BOUNDARY_DIM);
        importance = length * area * curvature;
    }else
        */importance = length * area;

}

double MAArcPath::getImportance() const
{
    return importance;
}

void MAArcPath::print(std::ostream &stream)
{
    stream<<"N1: "<<(long) n1<<std::endl;
    stream<<"N2: "<<(long) n2<<std::endl;
    stream<<"Inner points:"<<std::endl;
    for(unsigned j = 0; j < path.size(); j++)
        stream<<"\t("<<path[j]->x<<","<<path[j]->y<<","<<path[j]->z<<")"<<std::endl;
    stream<<"Traversed triangles:"<<std::endl;

    for(unsigned j = 0; j < traversedTriangles.size(); j++){
        stream<<"\tT"<<j + 1<<": [("<<traversedTriangles[j]->v1()->x<<","<<traversedTriangles[j]->v1()->y<<","<<traversedTriangles[j]->v1()->z<<"), ("
                              <<traversedTriangles[j]->v2()->x<<","<<traversedTriangles[j]->v2()->y<<","<<traversedTriangles[j]->v2()->z<<"), ("
                              <<traversedTriangles[j]->v3()->x<<","<<traversedTriangles[j]->v3()->y<<","<<traversedTriangles[j]->v3()->z<<")]"<<std::endl;
    }
    stream<<"Importance: "<<importance<<std::endl<<std::flush;
}
