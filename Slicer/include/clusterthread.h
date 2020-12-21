#ifndef CLUSTERTHREAD_H
#define CLUSTERTHREAD_H

#include <mainthread.h>
#include <vector>
#include <map>
#include <limits>
#include <cfloat>
#include <mutex>
#include <iostream>
#include <Eigen/Dense>

class ClusterThread : public MainThread{
public:
    ClusterThread(unsigned int first, unsigned int last, unsigned int clustersNumber, std::vector<Eigen::VectorXd>* points,
                  std::vector<std::vector<unsigned int> >* clusters, double* bestCost, std::mutex* mtx);
    void startThread();
    void waitThread();

protected:

    void executeTask();
    unsigned int first;
    unsigned int last;
    unsigned int clustersNumber;
    std::vector<Eigen::VectorXd>* points;
    std::vector<std::vector<unsigned int> >* clusters;
    double* bestCost;
    std::mutex* mtx;
};

#endif // CLUSTERTHREAD_H
