#include "annotationutilities.h"

#include <set>
#include <lineannotation.h>
#include <pointannotation.h>
#include <surfaceannotation.h>
#include <iostream>

using namespace IMATI_STL;
using namespace std;



void Utilities::findPoints(vector<Vertex*> &vertices, vector<Triangle*> triangles, vector<vector<Vertex*> > outlines){


    /* Value that says when an edge is on the boundary (if the value in its "info" field is equal to BOUNDARY_EDGE,
     * then the edge is on the boundary). */
    unsigned int BOUNDARY_EDGE = 1;

    /* Value that says when a triangle has already been used (if the value in its "info" field is equal to ALREADY_USED,
     * then the edge is on the boundary). */
    unsigned int ALREADY_USED = 9;

    for(vector<vector<Vertex*> >::iterator oit = outlines.begin(); oit != outlines.end(); oit++){
        vector<Vertex*> outline = static_cast<vector<Vertex*> >(*oit);
        for(vector<Vertex*>::iterator vit = outline.begin(); vit != outline.end(); vit++){
            Vertex* v = static_cast<Vertex*>(*vit);
            v->info = &BOUNDARY_EDGE;
        }
    }

    for(unsigned int i = 0; i < triangles.size(); i++){
        Vertex* v = triangles[i]->v1();
        for(unsigned int j = 0; j < 3; j++){
            v = triangles[i]->nextVertex(v);
            if(v->info == nullptr || (*static_cast<unsigned int*>(v->info) != ALREADY_USED && *static_cast<unsigned int*>(v->info) != BOUNDARY_EDGE)){
                vertices.push_back(v);
                v->info = &ALREADY_USED;
            }
        }
    }

    for(unsigned int i = 0; i < triangles.size(); i++){
        Vertex* v = triangles[i]->v1();
        for(unsigned int j = 0; j < 3; j++){
            v = triangles[i]->nextVertex(v);
            v->info = nullptr;
        }
    }
}

void Utilities::fixOutlineOrderToCounterclockwise(vector<int> innerVerticesIndices, vector<Vertex *> &outline, ExtendedTrimesh* model){

    int IS_POINTER = 19;
    int IS_POINTED = 20;
    vector<Triangle*> pointedTriangles;
    vector<Vertex*> innerVertices;
    vector<Vertex*> pointedVertices;

    for(vector<int>::iterator vit = innerVerticesIndices.begin(); vit != innerVerticesIndices.end(); vit++)
        innerVertices.push_back(model->getPoint(static_cast<int>(*vit)));

    for(vector<Vertex*>::iterator vit = innerVertices.begin(); vit != innerVertices.end(); vit++)
        (*vit)->info = &IS_POINTER;

    pointedVertices.insert(pointedVertices.end(), innerVertices.begin(), innerVertices.end());
    for(vector<Vertex*>::iterator vit = outline.begin(); vit != outline.end(); vit++)
        (*vit)->info = &IS_POINTER;
    pointedVertices.insert(pointedVertices.end(), outline.begin(), outline.end());

    for(vector<Vertex*>::iterator vit = pointedVertices.begin(); vit != pointedVertices.end(); vit++){
        Vertex* v = static_cast<Vertex*>(*vit);
        for(IMATI_STL::Node* n = v->VT()->head(); n != nullptr; n = n->next()){
            Triangle* t = static_cast<Triangle*>(n->data);
            if( (t->v1()->info != nullptr && *static_cast<int*>(t->v1()->info) == IS_POINTER) &&
                (t->v2()->info != nullptr && *static_cast<int*>(t->v2()->info) == IS_POINTER) &&
                (t->v3()->info != nullptr && *static_cast<int*>(t->v3()->info) == IS_POINTER) &&
                (t->info == nullptr || *static_cast<int*>(t->info) != IS_POINTED)){
                t->info = &IS_POINTED;
                pointedTriangles.push_back(t);
            }
        }
    }

    Triangle* as = outline[3]->getEdge(outline[4])->leftTriangle(outline[3]);

    if(as->info == nullptr || *static_cast<int*>(as->info) != IS_POINTED)
        std::reverse(outline.begin(), outline.end());

    for(vector<Vertex*>::iterator vit = innerVertices.begin(); vit != innerVertices.end(); vit++)
        (*vit)->info = nullptr;
    for(vector<Vertex*>::iterator vit = outline.begin(); vit != outline.end(); vit++)
        (*vit)->info = nullptr;
    for(vector<Triangle*>::iterator tit = pointedTriangles.begin(); tit != pointedTriangles.end(); tit++)
        (*tit)->info = nullptr;

}

bool Utilities::isPointInsideTriangle(Point checkPoint, Point trianglePoint1, Point trianglePoint2, Point trianglePoint3){
    Point AB = trianglePoint2 - trianglePoint1;
    Point AC = trianglePoint3 - trianglePoint1;
    Point PA = trianglePoint1 - checkPoint;
    Point PB = trianglePoint2 - checkPoint;
    Point PC = trianglePoint3 - checkPoint;
    double areaABC = (AB & AC).length() / 2.0;
    long double alfa = static_cast<long double>((PB & PC).length() / (2.0 * areaABC));
    long double beta = static_cast<long double>((PC & PA).length() / (2.0 * areaABC));
    long double gamma = static_cast<long double>((PA & PB).length() / (2.0 * areaABC));
    long double aboutOne = static_cast<long double>(1.0 + EPSILON_BC);
    if((alfa >= 0 && alfa <= aboutOne) && (beta >= 0 && beta <= aboutOne) &&
       (gamma >= 0 && gamma <= aboutOne) && (alfa + beta + gamma <= aboutOne))
        return true;
    else
        return false;
}

Vertex* Utilities::extractNearestVertex(vector<Vertex*> &frontier, map<Vertex*, double> distances){

    double minDist = DBL_MAX;
    int minPos = -1;
    for(unsigned int i = 0; i < frontier.size(); i++){
        if(distances[frontier[i]] < minDist){
            minPos = i;
            minDist = distances[frontier[i]];
        }
    }

    if(minPos == -1)
        return nullptr;

    Vertex* nearest = frontier[minPos];

    frontier.erase(frontier.begin() + minPos);

    return nearest;

}

vector<Vertex*> Utilities::dijkstra(Vertex* v1, Vertex* v2, const short int metric, const bool avoidUsed){

    vector<Vertex*> frontier;
    map<Vertex*, double> distances = {{v1, 0}};
    map<Vertex*, Vertex*> predecessors = {{v1, nullptr}};
    set<Vertex*> v21RingNeighbors;
    vector<Vertex*> shortestPath;
    Vertex* v;
    bool v2visited = false;

    if(((*v1) - (*v2)).length() == 0.0){
        return shortestPath;
    }

    frontier.push_back(v1);

    do{

        v = extractNearestVertex(frontier, distances);
        if(v == nullptr)
        {
            return shortestPath;
        }

        List* neighbors = v->VV();
        for(IMATI_STL::Node* n = neighbors->head(); n != nullptr; n = n->next()){
            Vertex* x = static_cast<Vertex*>(n->data);

            if(x == v2 && v21RingNeighbors.find(v) == v21RingNeighbors.end())
                v21RingNeighbors.insert(v);

            map<Vertex*, Vertex*>::iterator pit = predecessors.find(x);
            double distanceVX;
            if(!avoidUsed || x->info == nullptr || !(*static_cast<bool*>(x->info))){
                switch(metric){
                    case DistanceType::SEGMENT_DISTANCE:
                        distanceVX = distances[v] + x->distanceFromEdge(v1,v2);
                        break;
                    case DistanceType::COMBINED_DISTANCE:
                        distanceVX = distances[v] + (*x - *v).length() + x->distanceFromEdge(v1,v2);
                        break;
                    default:
                        distanceVX = distances[v] + (*x - *v).length();
                }
            } else
                distanceVX = DBL_MAX;

            if(pit != predecessors.end()){
                if(distances[x] > distanceVX){
                    distances[x] = distanceVX;
                    predecessors[x] = v;
                }
            } else {
                distances.insert(std::make_pair(x, distanceVX));
                predecessors.insert(std::make_pair(x, v));
                frontier.push_back(x);
            }
        }
        if(v == v2)
            v2visited = true;

    } while(!v2visited);

    shortestPath.push_back(v2);
    v = predecessors[v2];
    while(v != v1){
        shortestPath.insert(shortestPath.begin(), v);
        v = predecessors[v];
    }

    return shortestPath;
}

/**
 * @brief regionGrowing a method for computing the list of triangles that are part of the annotation identified by a list of boundaries.
 * @param boundaries The list of boundaries
 * @return the list of triangles that are part of the annotation
 */
vector<Triangle*> Utilities::regionGrowing(vector<vector<Vertex*> > boundaries){


    /* Value that says when an edge is on the boundary (if the value in its "info" field is equal to BOUNDARY_EDGE,
     * then the edge is on the boundary). */
    unsigned int BOUNDARY_EDGE = 1;

    /* Value that says when a triangle has already been used (if the value in its "info" field is equal to ALREADY_USED,
     * then the edge is on the boundary). */
    unsigned int ALREADY_USED = 9;

    // neighbors is a queue that stores the list of the reachable triangles.
    queue<Triangle*> neighbors;

    //List of triangles of the annotation (initially empty)
    vector<Triangle*> internalTriangles;

    //We iterate over the list of boundaries
    for(unsigned int i = 0; i < boundaries.size(); i++){

        //This is the i-th boundary
        vector<Vertex*> boundary = boundaries[i];
        /* For each boundary, we take the first two vertices (boundary[0] and boundary[1]) in the boundary.
         * The boundary is ordered counterclockwise, so you have to consider the triangle (t) on the left of
         * the edge connecting the two vertices.
         */
        Triangle* t = boundary[0]->getEdge(boundary[1])->leftTriangle(boundary[0]);
        internalTriangles.push_back(t);
        t->info = static_cast<void*>(&ALREADY_USED);
        //We insert the triangle (or seed of the region growing algorithm) in the queue of available triangles
        neighbors.push(t);
        //We mark each edge in the boundary as a boundary edge
        for(unsigned int i = 1; i <= boundary.size() - 1; i++){

            Vertex* v1 = boundary[i - 1];
            Vertex* v2 = boundary[i]; //% is the modulo operator. If i is equal to boundary.size() we use boundary[0].

            Edge* e = v1->getEdge(v2);
            if(e != nullptr)
                e->info = static_cast<void*>(&BOUNDARY_EDGE);
            else {
                std::cerr << "This shouldn't happen. Error in the identification of the edge." << std::endl << std::flush;
                std::cerr << "Local neighbourhood" << std::endl;
                std::cerr.precision(15);
                std::cerr << "Starting vertex: (" << v1->x << "," << v1->y << "," << v1->z << ")" << std::endl;
                unsigned int counter = 0;
                for(IMATI_STL::Node* n = v1->VE()->head(); n != nullptr; n = n->next())
                {
                    IMATI_STL::Vertex* v_ = static_cast<Edge*>(n->data)->oppositeVertex(v1);
                    std::cerr << counter + 1 << "-th neighbour: (" << v_->x << "," << v_->y << "," << v_->z << ")" << std::endl;
                    counter++;
                }

                std::cerr << "Wanted vertex: (" << v2->x << "," << v2->y << "," << v2->z << ")" << std::endl;
                counter = 0;
                for(IMATI_STL::Node* n = v1->VE()->head(); n != nullptr; n = n->next())
                {
                    IMATI_STL::Vertex* v_ = static_cast<Edge*>(n->data)->oppositeVertex(v1);
                    std::cerr << counter + 1 << "-th neighbour: (" << v_->x << "," << v_->y << "," << v_->z << ")" << std::endl;
                    counter++;
                }
                exit(5);
            }
        }

    }

    //We will analyse triangles until the queue of available triangles is not empty.
    while(neighbors.size() > 0){
        //We take the first triangle (t) from the queue and remove it from there
        Triangle* t = neighbors.front();
        neighbors.pop();
        //We take the first edge (e) of t
        Edge* e = t->e1;
        //t has 3 edges, so we will perform the analysis 3 times
        for(int i = 0; i < 3; i++){
            //If e is not on the boundary
            if(e->info == nullptr || *static_cast<unsigned int*>(e->info) != BOUNDARY_EDGE){
                //We take the triangle (t_) on the opposite side of e with respect to t.
                Triangle* t_ = e->oppositeTriangle(t);
                if(t_ == nullptr)
                    continue;   //the mesh is not watertight and the edge is on the bounday of the mesh
                //If t_ has't been used yet, we insert it in the queue of usable triangles and in the list of triangles that are part of the annotation
                if(t_->info == nullptr ||  *static_cast<unsigned int*>(t_->info) != ALREADY_USED){
                    internalTriangles.push_back(t_);
                    t_->info = static_cast<void*>(&ALREADY_USED);
                    neighbors.push(t_);
                }
            }
            //We take the next edge of t.
            e = t->nextEdge(e);
        }
    }

    //The actual algorithm is ended.

    //Ignore this for cycle, I am just removing the markings from the edges
    for(unsigned int i = 0; i < boundaries.size(); i++){

        vector<Vertex*> boundary = boundaries[i];
        for(unsigned int i = 1; i <= boundary.size() - 1; i++){

            Vertex* v1 = boundary[i - 1];
            Vertex* v2 = boundary[i];

            Edge* e = v1->getEdge(v2);
            if(e != nullptr)
                e->info = nullptr;
        }
    }


    //Ignore this for cycle, I am just removing the markings from the triangles
    for(std::vector<Triangle*>::iterator it = internalTriangles.begin(); it != internalTriangles.end(); it++)
        (*it)->info = nullptr;


    //Now the list has all the triangles in the annotation.
    return internalTriangles;

}

Vertex* Utilities::findCorrespondingVertex(Vertex* v, vector<Triangle*> neighbors){

    double bestDistance = DBL_MAX;
    Vertex* r = nullptr;
    Triangle* t = findCorrespondingTriangle(v, neighbors);
    if(t != nullptr){
        Vertex* tv1 = t->v1(), *tv2 = t->v2(), *tv3 = t->v3();
        Point p = Point::linePlaneIntersection(*v, (*v) + v->getNormal(), *tv1, *tv2, *tv3);
        Vertex* v_ = t->v1();

        for(int i = 0; i < 3; i++){
            double actualDistance = ((p)-(*v_)).length();
            if(actualDistance < bestDistance){
                bestDistance = actualDistance;
                r = v_;
            }
            v_ = t->nextVertex(v_);
        }

    }
    return r;

}

Triangle* Utilities::findCorrespondingTriangle(Vertex* v, vector<Triangle*> neighbors){

    bool found = false;
    double bestDistance = DBL_MAX;
    Triangle* t = nullptr;
    for(vector<Triangle*>::iterator it = neighbors.begin(); it != neighbors.end(); it++){
        Triangle* t_ = static_cast<Triangle*>(*it) ;
        double normalProduct = t_->getNormal() * v->getNormal();
        if(normalProduct >= 0){
            Vertex* tv1 = t_->v1(), *tv2 = t_->v2(), *tv3 = t_->v3();
            Point p = Point::linePlaneIntersection(*v, (*v) + v->getNormal(), *tv1, *tv2, *tv3);
            if(isPointInsideTriangle(p, *tv1, *tv2, *tv3)){
                double actualDistance = t_->pointTriangleDistance(v);
                if(!found){
                    t = t_;
                    bestDistance = actualDistance;
                    found = true;
                } else if(actualDistance < bestDistance){
                    t = t_;
                    bestDistance = actualDistance;
                }
            }
        }
    }

    return t;
}

vector<vector<Vertex *> > Utilities::getOutlines(vector<Triangle *> set){

    vector<std::pair<Vertex*, Vertex*> > setOutlineEdges;
    vector<vector<Vertex*> > outlines;
    Vertex* v, *v_;
    int IS_INSIDE = 738;

    for(std::vector<Triangle*>::iterator tit = set.begin(); tit != set.end(); tit++)
        (*tit)->info = &IS_INSIDE;

    for(std::vector<Triangle*>::iterator tit = set.begin(); tit != set.end(); tit++){
        Triangle* t = static_cast<Triangle*>(*tit);
        Edge* e = t->e1;
        for(int i = 0; i < 3; i++){
            Triangle* t_ = e->oppositeTriangle(t);
            if(t_ == nullptr || t_->info == nullptr || *static_cast<int*>(t_->info) != IS_INSIDE){
                Edge* e_ = t->prevEdge(e);
                v = e_->commonVertex(e);
                setOutlineEdges.push_back(std::make_pair(v, e->oppositeVertex(v)));
            }
            e = t->nextEdge(e);
        }
    }

    while(setOutlineEdges.size() != 0){

        vector<Vertex*> outline;
        v = setOutlineEdges[0].first;
        Vertex *initialVertex = v;

        std::pair<Vertex*, Vertex*> pPrev = std::make_pair(nullptr, nullptr);
        do{
            outline.push_back(v);
            for(IMATI_STL::Node *n = v->VV()->head(); n != nullptr; n = n->next()){
                v_ = static_cast<Vertex*>(n->data);
                std::pair<Vertex*, Vertex*> p = std::make_pair(v, v_);
                for(vector<pair<Vertex*, Vertex*> >::iterator pit = setOutlineEdges.begin(); pit != setOutlineEdges.end(); pit++){
                    pair<Vertex*, Vertex*> tmp = static_cast<pair<Vertex*, Vertex*> >(*pit);
                    if(p != pPrev &&
                       ((p.first == tmp.second && p.second == tmp.first) ||
                       (p.first == tmp.second && p.second == tmp.first)) ){
                        v = v_;
                        pPrev = p;
                        setOutlineEdges.erase(pit);
                        break;
                    }
                }

                if(v == v_)
                    break;
            }
        }while(v != initialVertex);
        outline.push_back(outline[0]);
        outlines.push_back(outline);
    }

    for(vector<vector<Vertex*> >::iterator oit = outlines.begin(); oit != outlines.end(); oit++){
        vector<Vertex*> outline = static_cast<vector<Vertex*> >(*oit);
        Triangle* t = nullptr;
        for (unsigned int i = 1; i < outline.size(); i++) {
            t = outline[i - 1]->getEdge(outline[i])->leftTriangle(outline[i - 1]);
            if(t != nullptr)
                break;
        }
        if(t == nullptr || std::find(set.begin(), set.end(), t) == set.end())
            std::reverse(oit->begin(), oit->end());
    }

    for(std::vector<Triangle*>::iterator tit = set.begin(); tit != set.end(); tit++){
        (*tit)->info = nullptr;
    }

    return outlines;

}


bool Utilities::isAnnotationContained(Annotation* a1, Annotation* a2) {

    if(a1->getType() == AnnotationType::Point)
    {
        if(a2->getType() == AnnotationType::Point || a2->getType() == AnnotationType::Line || a2->getType() == AnnotationType::Surface)
        {
            vector<IMATI_STL::Vertex*> a1Vertices = a1->getInvolvedVertices();
            vector<IMATI_STL::Vertex*> a2Vertices = a2->getInvolvedVertices();
            for(unsigned int i = 0; i < a1Vertices.size(); i++)
            {
                vector<IMATI_STL::Vertex*>::iterator tit = std::find(a2Vertices.begin(), a2Vertices.end(), a1Vertices[i]);
                if(tit == a2Vertices.end())
                    return false;
            }

            return true;
        } else
            return false;

    } else if(a1->getType() == AnnotationType::Line)
    {
        if(a2->getType() == AnnotationType::Line || a2->getType() == AnnotationType::Surface)
        {
            vector<IMATI_STL::Vertex*> a1Vertices = a1->getInvolvedVertices();
            vector<IMATI_STL::Vertex*> a2Vertices = a2->getInvolvedVertices();
            for(unsigned int i = 0; i < a1Vertices.size(); i++)
            {
                vector<IMATI_STL::Vertex*>::iterator tit = std::find(a2Vertices.begin(), a2Vertices.end(), a1Vertices[i]);
                if(tit == a2Vertices.end())
                    return false;
            }

            return true;
        } else
            return false;

    } else if(a1->getType() == AnnotationType::Surface && a2->getType() == AnnotationType::Surface)
    {
        SurfaceAnnotation* sa1 = dynamic_cast<SurfaceAnnotation*>(a1);
        SurfaceAnnotation* sa2 = dynamic_cast<SurfaceAnnotation*>(a2);
        vector<IMATI_STL::Triangle*> a1Triangles = sa1->getTriangles();
        vector<IMATI_STL::Triangle*> a2Triangles = sa2->getTriangles();
        double includedArea = 0.0, totalArea = 0.0;
        for(unsigned int i = 0; i < a1Triangles.size(); i++)
        {
            vector<IMATI_STL::Triangle*>::iterator tit = std::find(a2Triangles.begin(), a2Triangles.end(), a1Triangles[i]);
            if(tit != a2Triangles.end())
                includedArea += (*tit)->area();
            totalArea += a1Triangles[i]->area();
        }
        if(includedArea < 0.9 * totalArea)
            return false;
        return true;

    }

    return false;


}


void Utilities::fixHierarchyLevel(GraphTemplate::Node<Annotation *> * node, unsigned int level)
{
    node->getData()->setHierarchyLevel(level);
    vector<GraphTemplate::Arc<Annotation*>*> arcs = node->getArcs();
    for(unsigned int i = 0; i < arcs.size(); i++)
        if(arcs[i]->getN1() == node)
            fixHierarchyLevel(arcs[i]->getN2(), level + 1);
}



GraphTemplate::Tree<Annotation*> *Utilities::organizeAnnotationsHierarchically(std::vector<Annotation *> annotations)
{
    GraphTemplate::Tree<Annotation*>* tree = new GraphTemplate::Tree<Annotation*>();
    std::string REL_TYPE = "Containment";

    for(unsigned int i = 0; i < annotations.size(); i++)
        tree->addNode(annotations[i]);

    for(unsigned int i = 0; i < annotations.size(); i++){
        TNodePointer n1 = static_cast<TNodePointer >(tree->getNodes()[i]);
        for(unsigned int j = i + 1; j < annotations.size(); j++){
            TNodePointer n2 = static_cast<TNodePointer >(tree->getNodes()[j]);
            GraphTemplate::Arc<Annotation*>* a = nullptr;
            if(isAnnotationContained(annotations[i], annotations[j]))
                a = new GraphTemplate::Arc<Annotation*>(n2, n1, 1.0, true);
            else if(isAnnotationContained(annotations[j], annotations[i])){
                a = new GraphTemplate::Arc<Annotation*>(n1, n2, 1.0, true);
            }

            if(a != nullptr){
                a->setLabel(REL_TYPE);
                tree->addArc(a);
            }
        }


    }

    TNodePointer root;
    for(unsigned int i = 0; i < tree->getNodes().size(); i++)
        if(tree->getArcsFromTip(tree->getNodes()[i]).size() == 0){
            root = static_cast<TNodePointer>(tree->getNodes()[i]);
            break;
        }

    if(root == nullptr){
        cerr << "This shouldn't happen. A tree always has a root node.";
        exit(43);
    }

    tree->removeRedundancies();
//        std::queue<TNodePointer> Q;
//        Q.push(root);
//        do{
//            TNodePointer n = Q.front();

//            Q.pop();
//            vector<GraphTemplate::Arc<Annotation*>*> currentExitingArcs = tree->getArcsFromFletching(n);
//            for(unsigned int i = 0; i < currentExitingArcs.size(); i++){

//                vector<GraphTemplate::Arc<Annotation*>*> childEnteringArcs = tree->getArcsFromTip(currentExitingArcs[i]->getN2());
//                vector<GraphTemplate::Arc<Annotation*>*> childExitingArcs = tree->getArcsFromFletching(currentExitingArcs[i]->getN2());
//                if(childEnteringArcs.size() == 0) //|| childEnteringArcs[0]->getN1() != n) non sono sicuro se rimuoverlo
//                {
//                    cerr << "This shouldn't happen. If a node is son of another node, it has to have it as father";
//                    exit(44);
//                }

//                for(unsigned int j = 0; j < childEnteringArcs.size(); j++)
//                    if(childEnteringArcs[j]->getN1() != n)
//                        tree->removeArc(currentExitingArcs[i]);

////                if(childEnteringArcs.size() > 1 )
////                    tree->removeArc(currentExitingArcs[i]);

//                if(childExitingArcs.size() > 0)
//                    Q.push(static_cast<TNodePointer>(currentExitingArcs[i]->getN2()));
//            }

//        } while(!Q.empty());

    fixHierarchyLevel(root, 0);/*

    for(unsigned int i = 1; i < tree->getNodes().size(); i++)
    {
        std::vector<GraphTemplate::Arc<Annotation*>*> path = tree->shortestPathSearch(root, tree->getNodes()[i], "containment");
        std::cout << "Starting new path:" << std::endl;
        root->getData()->print(std::cout);
        GraphTemplate::Node<Annotation*>* n = root;
        for(unsigned int j = 0; j < path.size(); j++)
        {
            GraphTemplate::Node<Annotation*>* n_ = path[j]->getOppositeNode(n);
            n_->getData()->print(std::cout);
            n = n_;
            std::cout << std::endl;
        }
    }*/

    return tree;
}

GraphTemplate::Graph<Annotation *> *Utilities::extractAnnotationsAdjacency(std::vector<Annotation *> annotations)
{
    GraphTemplate::Graph<Annotation*>* graph = new GraphTemplate::Graph<Annotation*>();
    for(unsigned int i = 0; i < annotations.size(); i++)
        graph->addNode(annotations[i]);
    extractAnnotationsAdjacency(annotations, graph);
    return graph;
}

void Utilities::extractAnnotationsAdjacency(std::vector<Annotation *> annotations, GraphTemplate::Graph<Annotation *> *graph)
{
    std::string REL_TYPE = "Adjacency";
    for(unsigned int i = 0; i < annotations.size(); i++){
        SurfaceAnnotation* a1 = dynamic_cast<SurfaceAnnotation*>(annotations[i]);
        for(unsigned int j = i + 1; j < annotations.size(); j++){
            SurfaceAnnotation* a2 = dynamic_cast<SurfaceAnnotation*>(annotations[j]);
            if(a1->checkAdjacency(a2)){
                TNodePointer n1 = static_cast<TNodePointer >(graph->getNodes()[i]);
                TNodePointer n2 = static_cast<TNodePointer >(graph->getNodes()[j]);
                GraphTemplate::Arc<Annotation*>* a = new GraphTemplate::Arc<Annotation*>(n1, n2, 1.0, false);
                a->setLabel(REL_TYPE);
                graph->addArc(a);
            }
        }
    }
}



bool Utilities::checkAnnotationsIntersection(Annotation *a1, Annotation *a2)
{
    //This should be changed: we should not ask for the subtype of the pointer
    std::string type1(typeid (a1).name());
    std::string type2(typeid (a2).name());
    if(type1.compare("PointAnnotation")){
        PointAnnotation* a1p = dynamic_cast<PointAnnotation*>(a1);
        if(type2.compare("PointAnnotation"))
            return checkAnnotationsIntersection(a1p, dynamic_cast<PointAnnotation*>(a2));
        else if(type2.compare("LineAnnotation"))
            return checkAnnotationsIntersection(a1p, dynamic_cast<LineAnnotation*>(a2));
        else if(type2.compare("SurfaceAnnotation"))
            return checkAnnotationsIntersection(a1p, dynamic_cast<SurfaceAnnotation*>(a2));
        else{
            cerr << "This type of annotation is not managed in this software.";
            exit(85);
        }
    } else if(type1.compare("LineAnnotation")){
        LineAnnotation* a1l = dynamic_cast<LineAnnotation*>(a1);
        if(type2.compare("PointAnnotation"))
            return checkAnnotationsIntersection(dynamic_cast<PointAnnotation*>(a2), a1l);
        else if(type2.compare("LineAnnotation"))
            return checkAnnotationsIntersection(a1l, dynamic_cast<LineAnnotation*>(a2));
        else if(type2.compare("SurfaceAnnotation"))
            return checkAnnotationsIntersection(a1l, dynamic_cast<SurfaceAnnotation*>(a2));
        else{
            cerr << "This type of annotation is not managed in this software.";
            exit(85);
        }
    } else if(type1.compare("SurfaceAnnotation")){
        SurfaceAnnotation* a1s = dynamic_cast<SurfaceAnnotation*>(a1);
        if(type2.compare("PointAnnotation"))
            return checkAnnotationsIntersection(dynamic_cast<PointAnnotation*>(a2), a1s);
        else if(type2.compare("LineAnnotation"))
            return checkAnnotationsIntersection(dynamic_cast<LineAnnotation*>(a2), a1s);
        else if(type2.compare("SurfaceAnnotation"))
            return checkAnnotationsIntersection(a1s, dynamic_cast<SurfaceAnnotation*>(a2));
        else{
            cerr << "This type of annotation is not managed in this software.";
            exit(85);
        }
    } else{
        cerr << "This type of annotation is not managed in this software.";
        exit(85);
    }

}

GraphTemplate::Graph<Annotation *> *Utilities::buildRelationshipsGraph(std::vector<Annotation *> annotations)
{
    std::vector<Annotation*> pointAnnotations;
    std::vector<Annotation*> lineAnnotations;
    std::vector<Annotation*> surfaceAnnotations;
    for(unsigned int i = 0; i < annotations.size(); i++){
        Annotation *a = annotations[i];
        if(dynamic_cast<PointAnnotation*>(a) != nullptr)
            pointAnnotations.push_back(a);
        else if (dynamic_cast<LineAnnotation*>(a) != nullptr)
            lineAnnotations.push_back(a);
        else if(dynamic_cast<SurfaceAnnotation*>(a) != nullptr)
            surfaceAnnotations.push_back(a);
    }

    GraphTemplate::Graph<Annotation*>* graph = new GraphTemplate::Graph<Annotation*>(
                Utilities::organizeAnnotationsHierarchically(annotations));


//        std::string REL_TYPE = "Intersection";
//        for(unsigned int i = 0; i < lineAnnotations.size(); i++){
//            graph->addNode(lineAnnotations[i]);
//            GraphTemplate::Node<Annotation*>* n1 = graph->getNodes()[graph->getNodes().size() - 1]; //this should be changed

//            vector<GraphTemplate::Node<Annotation*> *> levelRiddenSurfaceAnnotations =
//                    graph->breadthFirstVisit(graph->getNodes()[0]);
//            for(int j = levelRiddenSurfaceAnnotations.size() - 1; j >= 0; j--){
//                GraphTemplate::Node<Annotation*>* n2 = levelRiddenSurfaceAnnotations[j];
//                if(dynamic_cast<SurfaceAnnotation*>(n2->getData()) == nullptr)
//                    continue;
//                bool intersects = checkLineSurfaceAnnotationsIntersection(dynamic_cast<LineAnnotation*>(lineAnnotations[i]),
//                                                        dynamic_cast<SurfaceAnnotation*>(n2->getData()));
//                if(intersects && graph->depthFirstSearch(n2, lineAnnotations[i]) == nullptr){
//                    GraphTemplate::Arc<Annotation*>* a = new GraphTemplate::Arc<Annotation*>(n1, n2, 1.0, false);
//                    a->setLabel(REL_TYPE);
//                    graph->addArc(a);
//                }
//            }
//        }

    Utilities::extractAnnotationsAdjacency(surfaceAnnotations, graph);

    return graph;
}

std::pair<Point *, Point *> Utilities::findExtremePoints(std::vector<Point *> points, Point direction)
{
    double min = DBL_MAX, max = -DBL_MAX;
    int minPos = -1, maxPos = -1;
    for(unsigned int i = 0; i < points.size(); i++)
    {
        Point projected = direction * ((*points[i]) * direction);
        double length = projected.length();
        if(projected * direction < 0 )
            length *= -1;
        if(length > max)
        {
            max = length;
            maxPos = i;
        }
        if(length < min)
        {
            min = length;
            minPos = i;
        }

    }

    return make_pair(points[minPos], points[maxPos]);
}


bool Utilities::checkAnnotationsIntersection(PointAnnotation *p1, PointAnnotation *p2)
{
    for(unsigned int i = 0; i < p1->getPoints().size(); i++)
        if(p2->isPointInAnnotation(p1->getPoints()[i]))
            return true;
    return false;
}

bool Utilities::checkAnnotationsIntersection(PointAnnotation *p, LineAnnotation *l)
{
    for(unsigned int i = 0; i < p->getPoints().size(); i++)
        if(l->isPointInAnnotation(p->getPoints()[i]))
            return true;

    return false;
}

bool Utilities::checkAnnotationsIntersection(PointAnnotation *p, SurfaceAnnotation *s)
{
    for(unsigned int i = 0; i < p->getPoints().size(); i++)
        if(s->isPointInAnnotation(p->getPoints()[i]))
            return true;

    return false;
}

bool Utilities::checkAnnotationsIntersection(LineAnnotation *l1, LineAnnotation *l2)
{
    for(unsigned int i = 0; i < l1->getPolyLines().size(); i++)
        for(unsigned int j = 0; j < l1->getPolyLines()[i].size(); j++)
            for(unsigned int k = 0; k < l2->getPolyLines().size(); k++)
                for(unsigned int l = 0; l < l2->getPolyLines()[k].size(); l++)
                    if(l1->getPolyLines()[i][j] == l2->getPolyLines()[k][l])
                        return true;

    return false;
}

bool Utilities::checkAnnotationsIntersection(LineAnnotation *l, SurfaceAnnotation *s)
{
    for(unsigned int i = 0; i < l->getPolyLines().size(); i++)
        for(unsigned int j = 0; j < l->getPolyLines()[i].size(); j++)
            if(s->isPointInAnnotation(l->getPolyLines()[i][j]))
                return true;

    return false;
}

bool Utilities::checkAnnotationsIntersection(SurfaceAnnotation *s1, SurfaceAnnotation *s2)
{
    for(unsigned int i = 0; i < s1->getOutlines().size(); i++)
        for(unsigned int j = 0; j < s1->getOutlines()[i].size(); j++)
            for(unsigned int k = 0; k < s2->getOutlines().size(); k++)
                for(unsigned int l = 0; l < s2->getOutlines()[k].size(); l++)
                    if(s1->getOutlines()[i][j] == s2->getOutlines()[k][l])
                        return true;

    return false;
}
