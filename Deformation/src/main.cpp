#include <iostream>
#include <imatistl.h>
#include <meanvaluecoordinatesju.h>

using namespace std;
using namespace IMATI_STL;

int main(int argc, char *argv[])
{
    ImatiSTL::init();

    DrawableMesh model, cage;

    if(argc<3) ImatiSTL::error("Missing filename.\n");

    if(model.load(argv[1]) != 0) ImatiSTL::error("Can't open model file.\n");
    if(cage.load(argv[2]) != 0) ImatiSTL::error("Can't open cage file.\n");

    MeanValueCoordinatesJu coords(&model, &cage);

    coords.computeCoordinates();
    vector<vector<double> > coordinates = coords.getCoordinates();
    for(int i=0; i< coordinates.size(); i++){
        for(int j=0; j< coordinates[0].size(); j++)
            std::cout<<coordinates[i][j]<<" ";
        std::cout<<std::endl;
    }
}
