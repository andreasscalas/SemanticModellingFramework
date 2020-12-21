#ifndef ANNOTATION_UTILITIES_H
#define ANNOTATION_UTILITIES_H

#include "lineannotation.h"
#include "surfaceannotation.h"

#include <vector>
#include <queue>
#include <map>
#include <set>
#include <math.h>
#include <iostream>
#include <cstdio>
#include <ctime>
#include <algorithm>
#include <extendedtrimesh.h>
#include <tree.h>

class ExtendedTrimesh;

namespace Utilities {

    const double EPSILON_BC = 0.00005;
    const double EPSILON_D = 0.01;
    const short int SEGMENT_DISTANCE = 1;
    const short int EUCLIDEAN_DISTANCE = 2;
    const short int COMBINED_DISTANCE = 3;

    typedef GraphTemplate::TreeNode<Annotation*>* TNodePointer;
    typedef GraphTemplate::Node<Annotation*>* NodePointer;
    typedef GraphTemplate::Arc<Annotation*>* ArcPointer;

    void findPoints(std::vector<IMATI_STL::Vertex*> &vertices, std::vector<IMATI_STL::Triangle*> triangles, std::vector<std::vector<IMATI_STL::Vertex*> > outlines);
    void checkOutlineOrder(std::vector<int> innerVerticesIndices, std::vector<IMATI_STL::Vertex *> &outline, ExtendedTrimesh* model);
    bool isPointInsideTriangle(IMATI_STL::Point checkPoint, IMATI_STL::Point trianglePoint1, IMATI_STL::Point trianglePoint2, IMATI_STL::Point trianglePoint3);
    std::vector<IMATI_STL::Vertex*> dijkstra(IMATI_STL::Vertex* v1, IMATI_STL::Vertex* v2, const short int metric, const bool avoidUsed);
    std::vector<IMATI_STL::Triangle*> regionGrowing(std::vector<std::vector<IMATI_STL::Vertex*> > contours);
    IMATI_STL::Vertex* findCorrespondingVertex(IMATI_STL::Vertex* v, std::vector<IMATI_STL::Triangle*> otherMesh);
    IMATI_STL::Triangle* findCorrespondingTriangle(IMATI_STL::Vertex* v, std::vector<IMATI_STL::Triangle*> otherMesh);
    std::vector<std::vector<IMATI_STL::Vertex *> > getOutlines(std::vector<IMATI_STL::Triangle *> set);
    IMATI_STL::Vertex* extractVertexWithLessValue(std::vector<IMATI_STL::Vertex*> &list, std::map<IMATI_STL::Vertex*, double> values);
    void fixHierarchyLevel(GraphTemplate::Node<Annotation *> *, unsigned int);
    GraphTemplate::Tree<Annotation *> *organizeAnnotationsHierarchically(std::vector<Annotation*>);
    GraphTemplate::Graph<Annotation *> *extractAnnotationsAdjacency(std::vector<Annotation*>);
    void extractAnnotationsAdjacency(std::vector<Annotation*>,GraphTemplate::Graph<Annotation *> *);
    bool checkAnnotationsIntersection(Annotation *a1, Annotation *a2);
    GraphTemplate::Graph<Annotation *> *buildRelationshipsGraph(std::vector<Annotation*>);
    std::pair<IMATI_STL::Point*, IMATI_STL::Point*> findExtremePoints(std::vector<IMATI_STL::Point*>, IMATI_STL::Point);
    std::pair<IMATI_STL::Point*, IMATI_STL::Point*> compute2OrthogonalVersors(IMATI_STL::Point v);

}

#endif
