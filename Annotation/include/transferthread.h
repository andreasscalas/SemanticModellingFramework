#ifndef TRANSFERTHREAD_H
#define TRANSFERTHREAD_H

#include <vector>
#include <thread>
#include <iostream>
#include <mainthread.h>
#include <annotationutilities.h>
#include <nanoflannhelper.h>
#include <imatistl.h>
#include <mutex>
class TransferThread : MainThread
{
public:
    TransferThread();
    short getMetric() const;
    void setMetric(short value);
    std::vector<IMATI_STL::Vertex *> getNewPath() const;
    void setEndingVertex(const std::vector<IMATI_STL::Vertex *>::iterator &value);
    void setStartingVertex(const std::vector<IMATI_STL::Vertex *>::iterator &value);
    void setHelper(NanoflannHelper *value);
    void setSphereRay(double value);
    void executeTask();
    void startThread();
    void waitThread();


    unsigned int getStart() const;
    void setStart(unsigned int value);

    unsigned int getEnd() const;
    void setEnd(unsigned int value);

    std::vector<IMATI_STL::Vertex *> getOutline() const;
    void setOutline(const std::vector<IMATI_STL::Vertex *> &value);

    std::mutex *getM() const;
    void setM(std::mutex *value);

protected:
    std::vector<IMATI_STL::Vertex*>::iterator startingVertex;
    std::vector<IMATI_STL::Vertex*>::iterator endingVertex;
    std::vector<IMATI_STL::Vertex*> newPath;
    NanoflannHelper* helper;
    std::vector<IMATI_STL::Vertex*> outline;
    std::mutex* m;
    unsigned int start;
    unsigned int end;
    short int metric;
    double sphereRay;

};

#endif // TRANSFERTHREAD_H
