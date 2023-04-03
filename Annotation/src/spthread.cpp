#include "spthread.h"
#include <annotationutilities.h>
#include <iostream>

using namespace std;
using namespace IMATI_STL;

SPThread::SPThread(Vertex* v1, Vertex* v2, vector<Vertex*>* path, const short int metric){
    this->v1 = v1;
    this->v2 = v2;
    this->path = path;
    this->metric = metric;
}

void SPThread::executeTask(){
    *path = Utilities::dijkstra(v1,v2, metric, false);
}

void SPThread::startThread(){ tid = new thread(MainThread::executeTaskHelper, this); }

void SPThread::waitThread(){ try{tid->join();} catch(int e){cout<<e;} }
