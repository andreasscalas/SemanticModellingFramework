#include "extendedtrimesh.h"
#include <utilities.h>
#include <iostream>

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

//! clamp cotangent values as if angles are in [1, 179]
inline double clamp_cot(const double v)
{
    const double bound = 19.1; // 3 degrees
    return (v < -bound ? -bound : (v > bound ? bound : v));
}

Point ExtendedTrimesh::computeLaplacian(unsigned int vertexId, WeightType type)
{
    IMATI_STL::Point laplacian(0, 0, 0);
    Vertex* v = idVertices[vertexId];
    double totalWeight = 0.0;
    ofstream file;
    file.open("laplacians.txt",std::ofstream::out | std::ofstream::app);
    file << "New laplacian" << std::endl;
    if(file.is_open())
    {
        file << v->x << " " << v->y << " " << v->z << std::endl << std::flush;
        for(IMATI_STL::Node* n = v->VV()->head(); n != nullptr; n = n->next())
        {
            IMATI_STL::Vertex* v_j = static_cast<IMATI_STL::Vertex*>(n->data);

            file << v_j->x << " " << v_j->y << " " << v_j->z << std::endl << std::flush;
            IMATI_STL::Edge* e = v->getEdge(v_j);
            double weight = 1.0;
            if(type == WeightType::COTANGENT)
            {
                weight = 0.0;
                IMATI_STL::Vertex* v_t1 = e->leftTriangle(v)->prevVertex(v);
                IMATI_STL::Vertex* v_t2 = e->rightTriangle(v)->nextVertex(v);
                IMATI_STL::Point v1 = (*v) - (*v_t1);
                IMATI_STL::Point v2 = (*v_j) - (*v_t1);
                IMATI_STL::Point v3 = (*v) - (*v_t2);
                IMATI_STL::Point v4 = (*v_j) - (*v_t2);
                double area1 = (v1 & v2).length();
                double area2 = (v3 & v4).length();
                double dot1 = v1 * v2;
                double dot2 = v3 * v4;
                if(dot1 > std::numeric_limits<double>::min() && area1 > std::numeric_limits<double>::min())
                    weight += clamp_cot(dot1 / area1);
                if(dot2 > std::numeric_limits<double>::min() && area2 > std::numeric_limits<double>::min())
                    weight += clamp_cot(dot2 / area2);
            }
            laplacian += ((*v_j) - (*v)) * weight;
            totalWeight += weight;
        }

        laplacian /= 2 * totalWeight;

        file << laplacian.x << " " << laplacian.y << " " << laplacian.z << std::endl << std::flush;
    }
//    cin.ignore();
    return laplacian;
}

void ExtendedTrimesh::smoothMesh(unsigned int iterations_number, WeightType type)
{
    for(unsigned int i = 0; i < iterations_number; i++)
        for(IMATI_STL::Node* n = this->V.head(); n != nullptr; n = n->next())
        {
            Vertex* v = static_cast<IMATI_STL::Vertex*>(n->data);
            Point l = computeLaplacian(verticesId[v], type);
            v->setValue((*v) + l);
        }
}

double ExtendedTrimesh::getMinEdgeLength()
{
    return minEdgeLength;
}

IMATI_STL::Vertex* ExtendedTrimesh::getPoint(unsigned long pointID) const{
    return idVertices.at(pointID);
}

void ExtendedTrimesh::setPoint(unsigned long vertexID, Point *p)
{
    setPoint(vertexID, p->x, p->y, p->z);
}

void ExtendedTrimesh::setPoint(unsigned long vertexID, double x, double y, double z)
{
    getPoint(vertexID)->setValue(x, y, z);

}

Vertex *ExtendedTrimesh::getClosestPoint(Point queryPt) const
{
    return getClosestPoints(queryPt, 1)[0];
}

std::vector<Vertex *> ExtendedTrimesh::getClosestPoints(Point queryPt, unsigned int neighboursNumber) const
{
    double point[3] = {queryPt.x, queryPt.y, queryPt.z};
    std::vector<size_t> ret_index(neighboursNumber);
    std::vector<num_t> out_dist_sqr(neighboursNumber);
    mat_index->index->knnSearch(point, neighboursNumber, &ret_index[0], &out_dist_sqr[0]);
    std::vector<IMATI_STL::Vertex*> closestPoints;
    for(std::vector<size_t>::iterator it = ret_index.begin(); it != ret_index.end(); it++)
        closestPoints.push_back(getPoint(*it));
    return closestPoints;

}

std::vector<Vertex *> ExtendedTrimesh::getNeighboursInSphere(Point queryPt, double radius) const
{
    double point[3] = {queryPt.x, queryPt.y, queryPt.z};
    const num_t search_radius = static_cast<num_t>(radius);
    std::vector<pair<size_t, num_t> > neighbors_distances;
    vector<Vertex*> neighbors;
    nanoflann::SearchParams params;
    mat_index->index->radiusSearch(point, search_radius, neighbors_distances, params);
    for(vector<pair<size_t,num_t> >::iterator it = neighbors_distances.begin(); it != neighbors_distances.end(); it++){
        pair<size_t,num_t> p = static_cast<pair<long int, double> >(*it);
        neighbors.push_back(getPoint(p.first));
    }

    return neighbors;
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

void ExtendedTrimesh::setAnnotation(unsigned int pos, Annotation *value)
{
    if(pos < annotations.size() && annotations[pos] != nullptr)
        annotations[pos] = value;
    //Non riesco a cancellare l'ex posizione in memoria

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
                neighbors = getNeighboursInSphere(*(v), tmp_ray);
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

Vertex *ExtendedTrimesh::getLowestVertex()
{
    IMATI_STL::Point* direction = new IMATI_STL::Point(0,1,0);
    return getLowestVertex(direction);
}

Vertex *ExtendedTrimesh::getLowestVertex(Point *direction)
{
    unsigned int i = 0, minPos = INT_MAX;
    double min = DBL_MAX;
    for(Node* n = V.head(); n != nullptr; n = n->next()){

        double t;
        IMATI_STL::Vertex* oldPosition = static_cast<IMATI_STL::Vertex*>(n->data);
        IMATI_STL::Point projected = (*direction) * (((*oldPosition) * (*direction)) / ((*direction) * (*direction)));
        if(std::abs(direction->x) > EPSILON)
            t = projected.x / direction->x;
        else if(std::abs(direction->y) > EPSILON)
            t = projected.y / direction->y;
        else
            t = projected.z / direction->z;

        if(t < min){
            minPos = i;
            min = t;
        }

        i++;

    }

    return getPoint(minPos);
}

std::map<IMATI_STL::Vertex *, unsigned long> ExtendedTrimesh::getVerticesId() const
{
    return verticesId;
}

void ExtendedTrimesh::buildInnerStructure()
{
    unsigned long i = 0;
    verticesId.clear();
    trianglesId.clear();
    for(Node* n = V.head(); n != nullptr; n = n->next()){
        Vertex* v = static_cast<Vertex*>(n->data);
        vector<double> point = {v->x, v->y, v->z};
        this->points_vector.push_back(point);
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

    mat_index = new my_kd_tree_t(3, this->points_vector, 10 /* max leaf */ );
    mat_index->index->buildIndex();
}

void ExtendedTrimesh::addAnnotation(Annotation* annotation){
    this->annotations.insert(this->annotations.begin() + annotation->getId(), annotation);
}

unsigned int ExtendedTrimesh::getAnnotationId(Annotation *a) const
{

    for(unsigned int i = 0; i < annotations.size(); i++)
        if(annotations[i] == a)
            return i;
    return -1;
}

void ExtendedTrimesh::removeAnnotation(Annotation* annotation){
    std::vector<Annotation*>::iterator ait = std::find(this->annotations.begin(), this->annotations.end(), annotation);
    if(ait != this->annotations.end())
        this->annotations.erase(ait);
}
