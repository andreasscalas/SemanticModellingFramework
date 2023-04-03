#ifndef MEANVALUECOORDINATES_H
#define MEANVALUECOORDINATES_H

#include <fstream>
#include "barycentriccoordinates.h"
#include "drawablemesh.h"
#include <sstream>
#include <stdio.h>

class MeanValueCoordinates : public BarycentricCoordinates{

    public:


        MeanValueCoordinates();
        ~MeanValueCoordinates();

        /**
        * @brief MeanValueCoordinatesJu::MeanValueCoordinatesJu Main constructor of the class. It requires the
        * model which will be deformed and the cage used for that purpose.
        * @param model the model which will be deformed
        * @param cage the cage used for the deformation
        */
        MeanValueCoordinates(DrawableMesh* model, DrawableMesh* cage);

        /**
         * @brief MeanValueCoordinatesJu::computeCoordinates Method that manages the computation of the coordinates
         */
        void computeCoordinates();

        /**
         * @brief MeanValueCoordinatesJu::saveCoordinates Method that manages the saving of the computed coordinates
         * @param filename the name of the file in which the coordinates will be saved
         */
        void saveCoordinates(QString filename);

        /**
         * @brief MeanValueCoordinatesJu::loadCoordinates Method that manages the loading of the coordinates
         * @param filename the name of the file from which the coordinates will be loaded
         */
        void loadCoordinates(QString filename);

        /**
         * @brief MeanValueCoordinatesJu::deform Method that manages the deformation of the model.
         */
        void deform();

        /**
         * @brief MeanValueCoordinatesJu::getCoordinates Method that allow the gathering of the coordinates
         * @return the coordinates set
         */
        Eigen::MatrixXd getCoordinates();

    private:
        DrawableMesh* model;
        DrawableMesh* cage;
        std::vector<std::vector<double> > coords;
        QString coordsFileName;
};

#endif // MEANVALUECOORDINATES_H
