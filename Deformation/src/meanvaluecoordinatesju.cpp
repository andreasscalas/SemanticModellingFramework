#include "meanvaluecoordinatesju.h"
using namespace IMATI_STL;
using namespace std;

MeanValueCoordinatesJu::MeanValueCoordinatesJu(DrawableMesh* model, DrawableMesh* cage){

    Node* n;
    Vertex* v;
    long i = 0;
    this->model = model;
    this->cage = cage;

    /* Considering that the algorithm updates the coordinates each time
     * that a triangle that contains the corresponding cage vertices is
     * found, it is neccessary that the algorithm knows the position,
     * inside the coordinates vector, corresponding to the cage vertex. */
    List* V = &(cage->V);
    FOREACHVVVERTEX(V, v, n)
         verticesID[(long) v] = i++;


     /* For similar reasons, the size of the vector must be defined before the
      * computation of the coordinates. */
    for(i=0; i < model->V.numels(); i++){

        vector<double> vcoordLine;
        for(long j=0; j < cage->V.numels(); j++)
            vcoordLine.push_back(0);

        coords.push_back(vcoordLine);

    }


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
    verticesID.clear();
}

void MeanValueCoordinatesJu::computeCoordinates(){

    ifstream myfile(coordsFileName);

    /* If the coordinates file has not yet been created, the algorithm
     * proceeds to the computation phase. */
    if (!myfile.is_open()){
        long i = 0, begin = 0, end;
        MainThread* threads[NUM_THREADS];
        long verticesPerThread = model->V.numels() / NUM_THREADS + 1;
        int t = 0;
        Node* ni;
        Vertex *v;

        //For each vertex of the model, the algorithm computes the corrisponding MVC
        List* V = &(model->V);
        FOREACHVVVERTEX(V, v, ni){

            /* The vertices set is splitted into parts, each of them managed by a different
             * thread, to parallelize the execution and reduce the computation time*/
            if(i == begin){
                end = begin + verticesPerThread;
                threads[t] = new MVCThread(cage, &coords, verticesID, ni, begin, end);
                threads[t++]->startThread();
                begin = end;
            }
            i++;

        }

        for(int j = 0; j < NUM_THREADS; j++){
            threads[j]->waitThread();
        }

    }

}

void MeanValueCoordinatesJu::saveCoordinates(std::string filename){

    ofstream savefile;
    savefile.open(filename);
    savefile<<"Mean Value Coordinates"<<endl<<flush;
    savefile<<model->V.numels()<<" "<<cage->V.numels()<<endl<<flush;
    for(long i = 0; i < coords.size(); i++){

        for(long j = 0; j < coords[0].size(); j++)
            savefile<<coords[i][j]<<" ";

        savefile<<std::endl<<flush;
    }
    savefile.close();

}

void MeanValueCoordinatesJu::loadCoordinates(std::string filename){

    ifstream loadfile;
    loadfile.open(filename);
    std::string line;
    std::getline(loadfile, line);
    std::getline(loadfile, line);
    istringstream* iss = new istringstream(line);
    int modelVerticesNumber;
    (*iss) >> modelVerticesNumber;
    for(unsigned int i = 0; i < modelVerticesNumber; i++){

        int j = 0;
        std::getline(loadfile, line);
        iss = new istringstream(line);
        double coord;
        while((*iss) >> coord)
            coords[i][j++]=coord;

    }
    loadfile.close();

}


void MeanValueCoordinatesJu::deform(){

    long i = 0, j;
    Node *ni, *nj;
    Vertex *vi, *vj;
    // For each vertex of the model, the new position of it is calculated using the coordinates properties.
    List *MV = &(model->V), *CV = &(cage->V);
    FOREACHVVVERTEX(MV, vi, ni){

        j = 0;
        Point v(0, 0, 0);

        FOREACHVVVERTEX(CV, vj, nj){

            v += (*vj) * coords[i][j];
            j++;

        }
		 
        vi->setValue(v.x, v.y, v.z);
        i++;

    }

}

vector<vector<double> > MeanValueCoordinatesJu::getCoordinates(){
    return this->coords;
}
