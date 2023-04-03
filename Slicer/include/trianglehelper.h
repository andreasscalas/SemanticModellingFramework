#ifndef TRIANGLEHELPER_H
#define TRIANGLEHELPER_H

#include <vector>
#include <string>
#include <extendedtrimesh.h>

class ExtendedTrimesh;

namespace TriHelper {
    class TriangleHelper
    {
    public:
        TriangleHelper(std::vector<IMATI_STL::Point*> boundary, std::vector<std::vector<IMATI_STL::Point*> > holes);

        std::vector<IMATI_STL::Triangle *> getTriangles() const;

        ExtendedTrimesh *getSliceMesh() const;

    private:
        std::vector<IMATI_STL::Point*> boundary;
        std::vector<std::vector<IMATI_STL::Point*> > holes;
        std::map<long, IMATI_STL::Point*> idVertex;
        const std::string filename = "tmp";
        ExtendedTrimesh* sliceMesh;
        void launchTriangle();
    };

}

#endif // TRIANGLEHELPER_H
