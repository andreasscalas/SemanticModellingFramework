#ifndef BARYCENTRICCOORDINATES_H
#define BARYCENTRICCOORDINATES_H

#include <vector>
#include <qstring.h>
#include <stdlib.h>
#include <Eigen/Dense>

class BarycentricCoordinates{

    public:
        virtual void computeCoordinates() = 0;
        virtual void deform() = 0;
        virtual Eigen::MatrixXd getCoordinates() = 0;
        virtual void saveCoordinates(QString filename) = 0;
        virtual void loadCoordinates(QString filename) = 0;

};

#endif // BARYCENTRICCOORDINATES_H
