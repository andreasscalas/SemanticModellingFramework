#ifndef BARYCENTRICCOORDINATES_H
#define BARYCENTRICCOORDINATES_H

#include <vector>
#include <string>
#include <Eigen/Core>

class BarycentricCoordinates{

    public:
        virtual ~BarycentricCoordinates() = 0;
        virtual void computeCoordinates() = 0;
        virtual void deform() = 0;
        virtual std::vector<std::vector<double>> getCoordinates() = 0;
        virtual void saveCoordinates(std::string filename) = 0;
        virtual void loadCoordinates(std::string filename) = 0;
        virtual std::vector<unsigned int> getMaxInfluenceCageVertices(unsigned int) = 0;
        virtual void updateCageVertexPosition(unsigned int, Eigen::Vector3d) = 0;
};

#endif // BARYCENTRICCOORDINATES_H
