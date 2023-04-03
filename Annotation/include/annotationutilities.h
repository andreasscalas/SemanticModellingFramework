#ifndef ANNOTATION_UTILITIES_H
#define ANNOTATION_UTILITIES_H


#include <vector>
#include <map>
#include <extendedtrimesh.h>
#include <tree.h>
#include <pointannotation.h>
#include <lineannotation.h>
#include <surfaceannotation.h>

class SurfaceAnnotation;
namespace Utilities {

    const double EPSILON_BC = 0.00005;          //Epsilon value checking if the sum of the barycentric coordinates equals one (1+-epsilon)

    enum DistanceType {SEGMENT_DISTANCE, EUCLIDEAN_DISTANCE, COMBINED_DISTANCE};

    //Shortening types name
    typedef GraphTemplate::TreeNode<Annotation*>* TNodePointer;
    typedef GraphTemplate::Node<Annotation*>* NodePointer;
    typedef GraphTemplate::Arc<Annotation*>* ArcPointer;

    /**
     * @brief findPoints allows to fill the list of vertices with those defining the triangles inside the boundary in outlines
     * @param vertices the list of vertices to be filled
     * @param triangles the list of triangles from which the vertices should be taken
     * @param outlines the boundaries containing the triangles
     */
    void findPoints(std::vector<IMATI_STL::Vertex*> &vertices, std::vector<IMATI_STL::Triangle*> triangles, std::vector<std::vector<IMATI_STL::Vertex*> > outlines);

    /**
     * @brief fixOutlineOrderToCounterclockwise checks the order of the vertices in the outline and, if they are in clockwise order, reverts their list.
     * @param innerVerticesIndices vertices inside the outline
     * @param outline the list of vertices in the outline
     * @param model the 3D mesh onto which the outline is defined
     */
    void fixOutlineOrderToCounterclockwise(std::vector<int> innerVerticesIndices, std::vector<IMATI_STL::Vertex *> &outline, ExtendedTrimesh* model);

    /**
     * @brief isPointInsideTriangle checks wether a point in space is inside a triangle. This is true iff the points lies on the same plane of
     * the triangle and is indeed inside it.
     * @param checkPoint the query point
     * @param trianglePoint1 first point defining the triangle
     * @param trianglePoint2 second point defining the triangle
     * @param trianglePoint3 third point defining the triangle
     * @return
     */
    bool isPointInsideTriangle(IMATI_STL::Point checkPoint, IMATI_STL::Point trianglePoint1, IMATI_STL::Point trianglePoint2, IMATI_STL::Point trianglePoint3);

    /**
     * @brief extractNearestVertex support method for the Dijkstra algorithm. Given a list of vertices in the frontier, it extracts the "closest" one,
     * in terms of the value contained into the list of distances
     * @param frontier the list of vertices to be checked
     * @param distances the list of distances corresponding to the vertices in frontier
     * @return the closest vertex
     */
    IMATI_STL::Vertex* extractNearestVertex(std::vector<IMATI_STL::Vertex*> &frontier, std::map<IMATI_STL::Vertex*, double> distances);

    /**
     * @brief dijkstra approximate version of the famous Dijkstra algorithm for the shortest path computation. The method stops the search for better paths whenever
     * it finds the target vertex v2
     * @param v1 the origin of the path
     * @param v2 the target position of the path
     * @param metric the metric to be used for the distance computation
     * @param avoidUsed if checked, gives &infin; weight to the already used arcs, to avoid intersections
     * @return the shortest path (as list of successive vertices) connecting v1 and v2
     */
    std::vector<IMATI_STL::Vertex*> dijkstra(IMATI_STL::Vertex* v1, IMATI_STL::Vertex* v2, const short int metric, const bool avoidUsed);

    /**
     * @brief regionGrowing method for obtaining the set of triangles composing a region enclosed by some contours. Inside/outside ambiguity is solved imposing that the
     * contours must be ordered coherently (here counterclockwisely).
     * @param contours the contours enclosing the region.
     * @return the set of triangles composing the region.
     */
    std::vector<IMATI_STL::Triangle*> regionGrowing(std::vector<std::vector<IMATI_STL::Vertex*> > contours);

    /**
     * @brief findCorrespondingVertex searches for the "best fitting" vertex in a mesh w.r.t. a certain query vertex. Here "best fitting"
     * means closest (in an Euclidean way) vertex with concordant normal.
     * @param v the query vertex
     * @param otherMesh the mesh in which the corresponding vertex should be searched
     * @return the corresponding vertex
     */
    IMATI_STL::Vertex* findCorrespondingVertex(IMATI_STL::Vertex* v, std::vector<IMATI_STL::Triangle*> otherMesh);

    /**
     * @brief findCorrespondingTriangle searches for the closest triangle (in an Euclidean way) with concordant normal w.r.t. a certain query vertex in a certain mesh.
     * @param v the query vertex
     * @param otherMesh the mesh in which the corresponding triangle should be searched
     * @return
     */
    IMATI_STL::Triangle* findCorrespondingTriangle(IMATI_STL::Vertex* v, std::vector<IMATI_STL::Triangle*> otherMesh);

    /**
     * @brief getOutlines method that extracts the boundaries of a certain not necessarily connected set of triangles (possibly with holes)
     * @param set the triangles composing the query region
     * @return the set of boundaries, defined as a list of successive vertexes on a closed polyline
     */
    std::vector<std::vector<IMATI_STL::Vertex *> > getOutlines(std::vector<IMATI_STL::Triangle *> set);

    /**
     * @brief isAnnotationContained checks if an annotation is contained in another one. This depends on the annotation selection type:
     *  - Area annotation: an area annotation is contained into another one if all of the corresponding annotated triangles are part of the other one's.
     *  - (Poly)Line annotation: a line annotation is contained into another one if all of the corresponding annotated edges are part of the other one's.
     *  - Point annotation: a point annotation is contained into another one if all of the corresponding annotated points are part of the other one's.
     * @param a1 The first annotation
     * @param a2 The second annotation
     * @return true if a1 is contained into a2, false otherwise.
     */
    bool isAnnotationContained(Annotation* a1, Annotation* a2);

    void fixHierarchyLevel(GraphTemplate::Node<Annotation *> *, unsigned int);
    GraphTemplate::Tree<Annotation *> *organizeAnnotationsHierarchically(std::vector<Annotation*>);
    GraphTemplate::Graph<Annotation *> *extractAnnotationsAdjacency(std::vector<Annotation*>);
    void extractAnnotationsAdjacency(std::vector<Annotation*>,GraphTemplate::Graph<Annotation *> *);
    bool checkAnnotationsIntersection(Annotation *a1, Annotation *a2);
    GraphTemplate::Graph<Annotation *> *buildRelationshipsGraph(std::vector<Annotation*>);
    std::pair<IMATI_STL::Point*, IMATI_STL::Point*> findExtremePoints(std::vector<IMATI_STL::Point*>, IMATI_STL::Point);
    std::pair<IMATI_STL::Point*, IMATI_STL::Point*> compute2OrthogonalVersors(IMATI_STL::Point v);

    bool checkAnnotationsIntersection(PointAnnotation *p1, PointAnnotation *p2);
    bool checkAnnotationsIntersection(PointAnnotation *p, LineAnnotation *l);
    bool checkAnnotationsIntersection(PointAnnotation *p, SurfaceAnnotation *s);
    bool checkAnnotationsIntersection(LineAnnotation *l1, LineAnnotation *l2);
    bool checkAnnotationsIntersection(LineAnnotation *l, SurfaceAnnotation *s);
    bool checkAnnotationsIntersection(SurfaceAnnotation *s1, SurfaceAnnotation *s2);

}

#endif
