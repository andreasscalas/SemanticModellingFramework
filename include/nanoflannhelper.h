#ifndef NANOFLANNHELPER_H
#define NANOFLANNHELPER_H

#include <vector>
#include "extendedtrimesh.h"
#include "imatistl.h"
#include "KDTreeVectorOfVectorsAdaptor.h"
#include "nanoflann.hpp"

class ExtendedTrimesh;

/**
 * @brief The NanoflannHelper class manages the interface with the Nanoflann library (KD-tree library).
 */
class NanoflannHelper{

    public:
        NanoflannHelper(ExtendedTrimesh* mesh);
        /**
         * @brief getNeighboursInSphere method for searching for the vertices of the mesh which are inside the sphere defined by queryPt and radius.
         * @param queryPt the center of the sphere.
         * @param radius the radius of the sphere.
         * @return the set of points that are inside the sphere.
         */
        std::vector<IMATI_STL::Vertex*> getNeighboursInSphere(IMATI_STL::Point queryPt, double radius);
        IMATI_STL::Vertex* getNearestNeighbors(IMATI_STL::Point queryPt, unsigned int neighborsNumber);

    protected:
        typedef std::vector<std::vector<double> > my_vector_of_vectors_t;
        typedef KDTreeVectorOfVectorsAdaptor< my_vector_of_vectors_t, double > my_kd_tree_t;
        typedef double num_t;

        my_vector_of_vectors_t points;
        my_kd_tree_t* mat_index;
        ExtendedTrimesh* mesh;

};

#endif // NANOFLANNHELPER_H
