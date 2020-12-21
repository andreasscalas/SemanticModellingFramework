#include "extendedtrimesh.h"
#include <utilities.h>
#include <iostream>
#include <sstream>
#include <typeinfo>
#include <nanoflannhelper.h>
#include <annotationsrelationship.h>
#include <surfaceannotation.h>

using namespace IMATI_STL;
using namespace std;

ExtendedTrimesh::ExtendedTrimesh(){
    graph = nullptr;
}

ExtendedTrimesh::~ExtendedTrimesh()
{
    verticesId.clear();
    trianglesId.clear();
    idVertices.clear();
    idTriangles.clear();
    idTriangles.clear();
    annotations.clear();
}

ExtendedTrimesh::ExtendedTrimesh(ExtendedTrimesh* m) : IMATI_STL::TriMesh (m, true)
{
    minEdgeLength = DBL_MAX;
    buildInnerStructure();
    vector<Annotation*> anns = m->getAnnotations();
    if(!isCage){
        for (unsigned int i = 0; i < anns.size(); i++) {
            Annotation* a = anns[i]->transfer(this);
            annotations.push_back(a);
            a->setMesh(this);
        }
    }
    this->graph = m->getGraph();
}

int ExtendedTrimesh::load(string filename){
    int l_result = TriMesh::load(filename.c_str());

    if(l_result == 0){
        this->filename = filename;
        buildInnerStructure();
    }

    return l_result;
}

double ExtendedTrimesh::getMinEdgeLength()
{
    return minEdgeLength;
}

IMATI_STL::Vertex* ExtendedTrimesh::getPoint(unsigned long pointID) const{
    return idVertices.at(pointID);
}

Triangle* ExtendedTrimesh::getTriangle(unsigned long triangleID) const{
    return idTriangles.at(triangleID);
}

unsigned long ExtendedTrimesh::getPointId(IMATI_STL::Vertex* v) const{
    return verticesId.at(v);
}

unsigned long ExtendedTrimesh::getTriangleId(Triangle* t) const{
    return trianglesId.at(t);
}

std::vector<unsigned int> ExtendedTrimesh::findAnnotations(string tag)
{
    vector<unsigned int> positions;
    for(unsigned int i = 0; i < annotations.size(); i++)
        if(annotations[i]->getTag().compare(tag) == 0)
            positions.push_back(i);
    return positions;
}

vector<Annotation *> ExtendedTrimesh::getAnnotations() const{
    return this->annotations;
}

void ExtendedTrimesh::setAnnotations(const vector<Annotation *> &value){
    this->annotations = value;
}

void ExtendedTrimesh::clearAnnotations()
{
    this->annotations.clear();
}

std::vector<Vertex *> ExtendedTrimesh::cutMesh(std::vector<Point *> cut)
{
    NanoflannHelper h(this);                    //The tag of the annotation
    double vol = this->volume();
    double cnum = this->V.numels();
    double sphere_ray = 10 * vol / cnum;
    double epsilon = 0;
    vector<Vertex*> neighbors;
    vector<Vertex*> cutPoints;

    vector<vector<Point*> > nullvec;
    TriHelper::TriangleHelper t(cut, nullvec);
    for(vector<Point*>::iterator pit = cut.begin(); pit != cut.end(); pit++){
        Vertex* v = static_cast<Vertex*>(*pit);
        vector<IMATI_STL::Triangle*> toCheckTriangles;
        pair<double, Edge*> best = make_pair(DBL_MAX, nullptr);
        if(cutPoints.size() == 0){
            unsigned int i = 0;

            do{
                double tmp_ray = ++i * sphere_ray;
                neighbors = h.getNeighboursInSphere(*(v), tmp_ray);
            }while(neighbors.size() < 10);

            Utilities::findFaces(toCheckTriangles, neighbors);
        }else{
            for(IMATI_STL::Node* n = cutPoints.back()->VT()->head(); n != nullptr; n = n->next())
                toCheckTriangles.push_back(static_cast<IMATI_STL::Triangle*>(n->data));
        }
        Edge* e = nullptr;
        for(vector<IMATI_STL::Triangle*>::iterator it = toCheckTriangles.begin(); it != toCheckTriangles.end(); it++){
            double dist;
            if(Utilities::isPointInSegment(v, (*it)->e1->v1, (*it)->e1->v2))
                e = (*it)->e1;
            if(Utilities::isPointInSegment(v, (*it)->e2->v1, (*it)->e2->v2))
                e = (*it)->e2;
            if(Utilities::isPointInSegment(v, (*it)->e3->v1, (*it)->e3->v2))
                e = (*it)->e3;

            dist = (v)->distanceFromEdge((*it)->e1->v1, (*it)->e1->v2);
            if(dist < best.first)
                best = make_pair(dist, (*it)->e1);
            dist = (v)->distanceFromEdge((*it)->e2->v1, (*it)->e2->v2);
            if(dist < best.first)
                best = make_pair(dist, (*it)->e2);
            dist = (v)->distanceFromEdge((*it)->e3->v1, (*it)->e3->v2);
            if(dist < best.first)
                best = make_pair(dist, (*it)->e3);
        }

        if(e == nullptr)
            e = best.second;

        epsilon = e->length() / 100;
        Vertex* newVertex;
        if((*(v) - e->v1).length() < epsilon)
            newVertex = e->v1;
        else if((*(v) - e->v2).length() < epsilon)
            newVertex = e->v2;
        else
            newVertex = this->splitEdge(e, (v));
        if(cutPoints.size() == 0 || *(cutPoints.back()) != (*newVertex))
            cutPoints.push_back(newVertex);
    }
    cutPoints.push_back(cutPoints.front());
    buildInnerStructure();

    return cutPoints;

}

std::string ExtendedTrimesh::getFilename() const
{
    return filename;
}

int ExtendedTrimesh::saveAnnotationsAsXYZ(const char *fname, bool ascii)
{
    ofstream file;
    file.open(fname);
    if(file.is_open()){
        vector<Vertex*> annotationsVertices;
        for(unsigned int i = 0; i < annotations.size(); i++){
            vector<Vertex*> involved = annotations[i]->getInvolvedVertices();
            annotationsVertices.insert(annotationsVertices.end(), involved.begin(), involved.end());

        }

        for(unsigned int i = 0; i < annotationsVertices.size(); i++){
            Vertex* v = annotationsVertices[i];
            Point normal = v->getNormal();
            file << v->x << " " << v->y << " " << v->z << " " << normal.x << " " << normal.y << " " << normal.z << endl;
        }
        return 1;
    }

    return 0;
}

int ExtendedTrimesh::saveXYZ(const char *fname, bool ascii)
{
    ofstream file;
    file.open(fname);
    if(file.is_open()){
        for (Node* n = V.head(); n != nullptr; n = n->next()) {
            Vertex* v = static_cast<Vertex*>(n->data);
            Point normal = v->getNormal();
            file << v->x << " " << v->y << " " << v->z << " " << normal.x << " " << normal.y << " " << normal.z << endl;
        }
        return 1;
    }

    return 0;
}


bool ExtendedTrimesh::getIsTemplate() const
{
    return isTemplate;
}

void ExtendedTrimesh::setIsTemplate(bool value)
{
    isTemplate = value;
}

bool ExtendedTrimesh::getIsCage() const
{
    return isCage;
}

void ExtendedTrimesh::setIsCage(bool value)
{
    isCage = value;
}

GraphTemplate::Graph<Annotation *> *ExtendedTrimesh::getGraph() const
{
    return graph;
}

void ExtendedTrimesh::setGraph(GraphTemplate::Graph<Annotation *> *value)
{
    graph = value;
}

bool ExtendedTrimesh::addAnnotationsRelationship(Annotation *a1, Annotation *a2, string relationshipType, double weight, bool directed, void *relationship)
{
    GraphTemplate::Node<Annotation*> *n1 = graph->getNodeFromData(a1);
    GraphTemplate::Node<Annotation*> *n2 = graph->getNodeFromData(a2);
    char* relType = new char[relationshipType.length() + 1];
    strcpy(relType, relationshipType.c_str());
    GraphTemplate::Arc<Annotation*> *a = new GraphTemplate::Arc<Annotation*>(
                n1, n2, weight, directed, static_cast<unsigned int>(graph->getArcs().size()), relType);
    a->setInfo(relationship);
    graph->addArc(a);
    return true;
}

std::map<unsigned long, IMATI_STL::Vertex *> ExtendedTrimesh::getIdVertices() const
{
    return idVertices;
}

std::map<unsigned long, IMATI_STL::Triangle *> ExtendedTrimesh::getIdTriangles() const
{
    return idTriangles;
}


void ExtendedTrimesh::buildInnerStructure()
{
    unsigned long i = 0;
    verticesId.clear();
    trianglesId.clear();
    for(Node* n = V.head(); n != nullptr; n = n->next()){
        Vertex* v = static_cast<Vertex*>(n->data);
        idVertices[i] = v;
        verticesId[v] = i++;
    }

    for(Node* n = E.head(); n != nullptr; n = n->next()){
        Edge* e = static_cast<Edge*>(n->data);
        if(e->length() < minEdgeLength)
            minEdgeLength = e->length();
    }

    i = 0;

    for(Node* n = T.head(); n != nullptr; n = n->next()){
        Triangle* t = static_cast<Triangle*>(n->data);
        idTriangles[i] = t;
        trianglesId[t] = i++;
    }
}

void ExtendedTrimesh::addAnnotation(Annotation* annotation){

    this->annotations.insert(this->annotations.begin() + annotation->getId(), annotation);
}

void ExtendedTrimesh::removeAnnotation(Annotation* annotation){
    std::vector<Annotation*>::iterator ait = std::find(this->annotations.begin(), this->annotations.end(), annotation);
    if(ait != this->annotations.end())
        this->annotations.erase(ait);
}
