#ifndef SHAPEOPBARYCENTRICCOORDINATES_H
#define SHAPEOPBARYCENTRICCOORDINATES_H

#include <Types.h>

class ShapeOpBarycentricCoordinates
{
public:
    virtual ~ShapeOpBarycentricCoordinates() {};
    virtual void computeCoordinates() = 0;
    virtual void deform() = 0;
    virtual ShapeOp::SparseMatrix* getCoordinates() const = 0;
    virtual void saveCoordinates(std::string filename) = 0;
    virtual void loadCoordinates(std::string filename) = 0;
    virtual std::vector<unsigned int> getMaxInfluenceCageVertices(unsigned int) = 0;
};

#endif // SHAPEOPBARYCENTRICCOORDINATES_H
