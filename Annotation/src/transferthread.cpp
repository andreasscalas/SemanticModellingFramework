#include "transferthread.h"
#include <utilities.h>

TransferThread::TransferThread()
{

}

short TransferThread::getMetric() const
{
    return metric;
}

void TransferThread::setMetric(short value)
{
    metric = value;
}

std::vector<IMATI_STL::Vertex *> TransferThread::getNewPath() const
{
    return newPath;
}

void TransferThread::setEndingVertex(const std::vector<IMATI_STL::Vertex *>::iterator &value)
{
    endingVertex = value;
}

void TransferThread::setStartingVertex(const std::vector<IMATI_STL::Vertex *>::iterator &value)
{
    startingVertex = value;
}

void TransferThread::executeTask(){
    std::vector<IMATI_STL::Vertex*> neighbors = helper->getNeighboursInSphere(*outline[start], sphereRay);
    std::vector<IMATI_STL::Triangle*> toCheckTriangles;
    Utilities::findFaces(toCheckTriangles, neighbors);
    IMATI_STL::Vertex* v1 = Utilities::findCorrespondingVertex(outline[start], toCheckTriangles);
    std::vector<IMATI_STL::Vertex*>::iterator vit = startingVertex;
    for (unsigned int i = start + 1; i < end; i++) {
        neighbors = helper->getNeighboursInSphere(*outline[i], sphereRay);
        Utilities::findFaces(toCheckTriangles, neighbors);
        IMATI_STL::Vertex* v2 = Utilities::findCorrespondingVertex(outline[i], toCheckTriangles);
        std::vector<IMATI_STL::Vertex*> tmp = Utilities::dijkstra(v1,v2, metric, false);
        newPath.insert(newPath.end(), tmp.begin(), tmp.end());
        v1 = v2;
    }
    std::cout<<std::flush;
}

void TransferThread::startThread(){ tid = new std::thread(MainThread::executeTaskHelper, this); }

void TransferThread::waitThread(){ try{tid->join();} catch(int e){std::cout<<e<<std::endl<<std::flush;} }

unsigned int TransferThread::getStart() const
{
    return start;
}

void TransferThread::setStart(unsigned int value)
{
    start = value;
}

unsigned int TransferThread::getEnd() const
{
    return end;
}

void TransferThread::setEnd(unsigned int value)
{
    end = value;
}

std::vector<IMATI_STL::Vertex *> TransferThread::getOutline() const
{
    return outline;
}

void TransferThread::setOutline(const std::vector<IMATI_STL::Vertex *> &value)
{
    outline = value;
}

std::mutex *TransferThread::getM() const
{
    return m;
}

void TransferThread::setM(std::mutex *value)
{
    m = value;
}

void TransferThread::setSphereRay(double value)
{
    sphereRay = value;
}

void TransferThread::setHelper(NanoflannHelper *value)
{
    helper = value;
}
