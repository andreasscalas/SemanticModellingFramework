#include "slicingthread.h"

#include <imatistl.h>
#include <slicer.h>
#include <iostream>

using namespace std;
using namespace IMATI_STL;

SlicingThread::SlicingThread(unsigned int first, unsigned int last, unsigned int slicesNumber, ExtendedTrimesh* mesh,
                             vector<std::pair<Slice*, double> >* slices_height, std::mutex* mtx){

    this->first = first;
    this->last = last;
    this->slicesNumber = slicesNumber;
    this->mesh = mesh;
    this->slices_height = slices_height;
    this->mtx = mtx;

}

SlicingThread::~SlicingThread(){}

void SlicingThread::startThread(){
    tid = new thread(executeTaskHelper, this);
}

void SlicingThread::waitThread(){
    try{
        tid->join();
    }catch(int e){
        cout<<e<<endl<<flush;
    }
}

void SlicingThread::executeTask(){

    IMATI_STL::Point normal = {0, 1, 0};
    IMATI_STL::Point a, b, c;
    mesh->getBoundingBox(a, b);
    double totalHeight = abs(a.y - b.y);
    double min = std::min(a.y, b.y);
    c = mesh->getCenter();
    for(unsigned int i = first; i < last; i++){

        c.y = min + totalHeight * i / slicesNumber;
        Slicer slicer;
        slicer.setCenter(c);
        slicer.setNormal(normal);
        slicer.setMaxSimplificationError(0);
        slicer.setMesh(static_cast<ExtendedTrimesh*>(mesh));
        slicer.slice();

        vector<Slice*> slices = slicer.getSlices();
        vector<pair<Slice*, double> > slice_height;
        for(unsigned int j = 0; j < slices.size(); j++){
            slice_height.push_back(make_pair(slices[j], c.y));
        }
        mtx->lock();
        slices_height->insert(slices_height->end(), slice_height.begin(), slice_height.end());
        mtx->unlock();

    }

}

