#ifndef BARYCENTRICCOORDINATES_H
#define BARYCENTRICCOORDINATES_H

#include <vector>
#include <string>
#include <stdlib.h>

class BarycentricCoordinates{

    public:
        virtual void computeCoordinates() = 0;
        virtual void deform() = 0;
        virtual std::vector<std::vector<double>> getCoordinates() = 0;
        virtual void saveCoordinates(std::string filename) = 0;
        virtual void loadCoordinates(std::string filename) = 0;

};

#endif // BARYCENTRICCOORDINATES_H
