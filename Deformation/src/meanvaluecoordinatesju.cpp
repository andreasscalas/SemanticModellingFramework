#include "meanvaluecoordinatesju.h"
#include <imatistl.h>
#include <mvcthread.h>
#include <fstream>
#include <mutex>
#include <Eigen/Sparse>
#include <Eigen/Core>

using namespace IMATI_STL;
using namespace std;

MeanValueCoordinatesJu::MeanValueCoordinatesJu(DrawableMesh* model, DrawableMesh* cage){

    this->model = model;
    this->cage = cage;
    cageVertices.resize(cage->V.numels(), 3);

    /* Considering that the algorithm updates the coordinates each time
     * that a triangle that contains the corresponding cage vertices is
     * found, it is neccessary that the algorithm knows the position,
     * inside the coordinates vector, corresponding to the cage vertex. */
    for(unsigned int i = 0; i < cage->V.numels(); i++)
        cageVertices.row(i) << cage->getPoint(i)->x, cage->getPoint(i)->y, cage->getPoint(i)->z;

    coordinates.resize(model->V.numels(), cage->V.numels());
    coordinates.setZero();
	string s = this->coordsFileName;
    s.resize(model->getFilename().find_last_of("."));
    s.append(".coord");
	s.substr(s.find_first_of("/") + 1);
    coordsFileName = s.substr(s.find_first_of("/") + 1);
}

MeanValueCoordinatesJu::~MeanValueCoordinatesJu()
{
    model = nullptr;
    cage = nullptr;
    coords.clear();
}

void MeanValueCoordinatesJu::computeCoordinates(){

    if (cage->V.numels() != model->V.numels()){
        long i = 0, begin = 0, end;
        Eigen::MatrixXd tmpCoords;
        tmpCoords.setZero(coordinates.rows(), coordinates.cols());
        MainThread* threads[NUM_THREADS];
        long verticesPerThread = model->V.numels() / NUM_THREADS + 1;
        int t = 0;
        Node* ni;
        Vertex *v;
        Eigen::initParallel();
        //For each vertex of the model, the algorithm computes the corrisponding MVC
        List* V = &(model->V);
        std::mutex mtx;
        FOREACHVVVERTEX(V, v, ni){

            /* The vertices set is splitted into parts, each of them managed by a different
             * thread, to parallelize the execution and reduce the computation time*/
            if(i == begin){
                end = begin + verticesPerThread;
                threads[t] = new MVCThread(model, cageVertices, &(cage->T), &tmpCoords, cage->getVerticesId(), begin, end);
                static_cast<MVCThread*>(threads[t])->setMtx(&mtx);
                threads[t++]->startThread();
                begin = end;
            }
            i++;

        }

        for(int j = 0; j < NUM_THREADS; j++){
            threads[j]->waitThread();
        }


        std::vector<Eigen::Triplet<double>> triplets;
        for(unsigned int i = 0; i < model->V.numels(); i++){
            for(unsigned int j = 0; j < cage->V.numels(); j++){
                if(abs(tmpCoords(i,j)) > 1e-3){
                    Eigen::Triplet<double> tmp(i, j, tmpCoords(i,j));
                    triplets.push_back(tmp);
                }
            }
        }
        coordinates.setFromTriplets(triplets.begin(), triplets.end());

    } else
        coordinates.setIdentity();
	
    analyseMaxInfluenceThreshold();

}

void MeanValueCoordinatesJu::saveCoordinates(std::string filename){

    ofstream savefile;
    savefile.open(filename);
    savefile<<"Mean Value Coordinates"<<endl<<flush;
    savefile<<model->V.numels()<<" "<<cage->V.numels()<<endl<<flush;
    savefile << coordinates;
    savefile.close();

}

void MeanValueCoordinatesJu::loadCoordinates(std::string filename){

    ifstream loadfile;
        loadfile.open(filename);
        std::string line;
        std::getline(loadfile, line);
        std::getline(loadfile, line);
        istringstream* iss = new istringstream(line);
        int modelVerticesNumber, cageVerticesNumber;
        Eigen::MatrixXd tmpCoords;
        (*iss) >> modelVerticesNumber;
        (*iss) >> cageVerticesNumber;
        tmpCoords.resize(modelVerticesNumber, cageVerticesNumber);
        std::vector<Eigen::Triplet<double>> triplets;
        for(unsigned int i = 0; i < modelVerticesNumber; i++){

            int j = 0;
            std::getline(loadfile, line);
            iss = new istringstream(line);
            double coord;
            while((*iss) >> coord){
                tmpCoords(i,j) = coord;
                if(abs(coord) > 1e-4){
                    Eigen::Triplet<double> tmp(i,j,coord);
                    triplets.push_back(tmp);
                }
                j++;
            }

        }

        coordinates.setFromTriplets(triplets.begin(), triplets.end());
        analyseMaxInfluenceThreshold();

        loadfile.close();

}


void MeanValueCoordinatesJu::deform(){

    Eigen::MatrixX3d modelPoints = coordinates * cageVertices;
    for(unsigned int i = 0; i < modelPoints.rows(); i++)
    {
        double p[3] = {modelPoints(i, 0), modelPoints(i,1), modelPoints(i,2)};
        model->setPointPosition(i, p);
    }


}

vector<vector<double> > MeanValueCoordinatesJu::getCoordinates(){
    return this->coords;
}

double MeanValueCoordinatesJu::getThreshold() const
{
    return threshold;
}

void MeanValueCoordinatesJu::setThreshold(double value)
{
    threshold = value;
}

void MeanValueCoordinatesJu::analyseMaxInfluenceThreshold()
{

    Eigen::VectorXd maxValues;
    maxValues.resize(coordinates.outerSize());
    for(unsigned int i = 0; i < coordinates.outerSize(); i++)
    {
        double max = 0;
        for(Eigen::SparseMatrix<double>::InnerIterator it(coordinates, i); it; ++it)
            if(it.value()>max)
                max = it.value();
        maxValues(i) = max;
    }
    threshold = maxValues.minCoeff();

    vector<Eigen::Triplet<bool> > triplets;
    for(unsigned int i = 0; i < coordinates.outerSize(); i++)
        for(Eigen::SparseMatrix<double>::InnerIterator it(coordinates, i); it; ++it)
            if(it.value() > threshold - 1e-5)
            {
                Eigen::Triplet<bool> t(it.row(), it.col(), true);
                triplets.push_back(t);
            }

    maxInfluenceValues.resize(coordinates.rows(), coordinates.cols());
    maxInfluenceValues.setFromTriplets(triplets.begin(), triplets.end());

}

std::vector<unsigned int> MeanValueCoordinatesJu::getMaxInfluenceCageVertices(unsigned int vertexId)
{
    std::vector<unsigned int> cageVertices;
    for(Eigen::SparseMatrix<bool, Eigen::RowMajor>::InnerIterator it(maxInfluenceValues, vertexId); it; ++it)
        cageVertices.push_back(it.col());
    return cageVertices;
}

void MeanValueCoordinatesJu::updateCageVertexPosition(unsigned int i, Eigen::Vector3d p)
{
    cageVertices.row(i) = p;
}

