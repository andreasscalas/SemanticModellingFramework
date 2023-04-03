#include "meanvaluecoordinates.h"

using namespace std;

MeanValueCoordinates::MeanValueCoordinates(){

}

MeanValueCoordinates::~MeanValueCoordinates()
{
    model = nullptr;
    cage = nullptr;
    coords.clear();
}

MeanValueCoordinates::MeanValueCoordinates(DrawableMesh* model, DrawableMesh* cage){

}

void MeanValueCoordinates::computeCoordinates(){



}

void MeanValueCoordinates::saveCoordinates(QString filename){

}

void MeanValueCoordinates::loadCoordinates(QString filename){

}

void MeanValueCoordinates::deform(){



}

Eigen::MatrixXd MeanValueCoordinates::getCoordinates(){

    //return this->coords;
	Eigen::MatrixXd a;
	a.Zero(3, 3);
	return a;

}
