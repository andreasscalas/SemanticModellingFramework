#ifndef GREENCOORDINATES_H
#define GREENCOORDINATES_H

#include <fstream>
#include <sstream>
#include <stdio.h>
#include <cmath>
#include "barycentriccoordinates.h"
#include "imatistl.h"
#include "gcthread.h"
#include <Eigen/LU>
#include <Eigen/Dense>
#define _USE_MATH_DEFINES

/**
 * @brief The GreenCoordinates class Class that allows the deformation of a model given a cage,
 * using the mathematical structure defined by Lipman et al. in the paper "Green Coordinates".
 */
class GreenCoordinates : public BarycentricCoordinates{

    private:

        DrawableMesh* model;                                                            //The model to deform
        DrawableMesh* cage;                                                             //The cage with which the model is deformed
        std::vector<std::pair<IMATI_STL::Point, IMATI_STL::Point> > trianglesBefore;    //The triangles (stored as pair of vectors) of the cage before the deformation
        std::vector<std::vector<double> > verticesCoords;                               //The coordinates linked to the vertices
        std::vector<std::vector<double> > trianglesCoords;                              //The coordinates linked to the triangles
        std::vector<double> scalingFactors;                                             //Factors that keep track of the stretch of the triangles during the deformation
        std::map<ulong, ulong> mverticesID;                                             //Ids of the vertices of the model
        std::map<ulong, ulong> verticesID;                                              //Ids of the vertices of the cage
        std::map<ulong, ulong> trianglesID;                                             //Ids of the triangles of the cage
        QString coordsFileName;                                                         //Generic name for the coordinates filename (in GC 2 files will be saved, one for the vertices and one for the triangles)
        const short int NUM_THREADS = 10;                                               //Number of threads which will be executed in parallel

        /**
         * @brief GreenCoordinates::cageExteriorExtension method that allows the extension of the GC
         * in the exterior of the cage
         * @param outerVertices the vertices that are outside of the cage
         */
        void cageExteriorExtension(std::vector<IMATI_STL::Vertex*> outerVertices);

        /**
         * @brief computeScalingFactors method that manage the computation of some factors that keep track
         * of the stretch of the triangles during the deformation
         */
        void computeScalingFactors();

    public:

        /**
        * @brief MeanValueCoordinatesJu::MeanValueCoordinatesJu Main constructor of the class. It requires the
        * model which will be deformed and the cage used for that purpose.
        * @param model the model which will be deformed
        * @param cage the cage used for the deformation
        */
        GreenCoordinates(DrawableMesh* model, DrawableMesh* cage);

        ~GreenCoordinates();
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

};

#endif // GREENCOORDINATES_H
