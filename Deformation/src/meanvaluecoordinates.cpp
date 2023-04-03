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

void MeanValueCoordinates::saveCoordinates(std::string filename){

}

void MeanValueCoordinates::loadCoordinates(std::string filename){

}

void MeanValueCoordinates::deform(){



}

vector<vector<double> > MeanValueCoordinates::getCoordinates(){

    return this->coords;

}
