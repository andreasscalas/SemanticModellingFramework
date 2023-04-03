#include "nanoflannhelper.h"

using namespace std;
using namespace IMATI_STL;
NanoflannHelper::NanoflannHelper(ExtendedTrimesh* mesh){this->mesh = mesh;
    for(Node* n = mesh->V.head(); n != nullptr; n = n->next()){
        Vertex* v = static_cast<Vertex*>(n->data);
        vector<double> point = {v->x, v->y, v->z};
        this->points.push_back(point);
    }

    mat_index = new my_kd_tree_t(3, this->points, 10 /* max leaf */ );
    mat_index->index->buildIndex();
}

vector<Vertex*> NanoflannHelper::getNeighboursInSphere(Point queryPt, double radius){
    double point[3] = {queryPt.x, queryPt.y, queryPt.z};
    const num_t search_radius = static_cast<num_t>(radius);
    std::vector<pair<size_t,num_t> > neighbors_distances;
    vector<Vertex*> neighbors;
    nanoflann::SearchParams params;
    mat_index->index->radiusSearch(point, search_radius, neighbors_distances, params);
    for(vector<pair<size_t,num_t> >::iterator it = neighbors_distances.begin(); it != neighbors_distances.end(); it++){
        pair<size_t,num_t> p = static_cast<pair<long int, double> >(*it);
        neighbors.push_back(mesh->getPoint(p.first));
    }

    return neighbors;
}

Vertex *NanoflannHelper::getNearestNeighbors(Point queryPt, unsigned int neighborsNumber)
{
    double point[3] = {queryPt.x, queryPt.y, queryPt.z};
    std::vector<size_t> ret_index(neighborsNumber);
    std::vector<num_t> out_dist_sqr(neighborsNumber);
    mat_index->index->knnSearch(point, neighborsNumber, &ret_index[0], &out_dist_sqr[0]);
    return mesh->getPoint(ret_index[0]);
}
