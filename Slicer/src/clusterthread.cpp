#include "clusterthread.h"
#include <map>
#include <limits>

using namespace std;
using namespace Eigen;

ClusterThread::ClusterThread(unsigned int first, unsigned int last, unsigned int clustersNumber, std::vector<Eigen::VectorXd>* points,
                             std::vector<std::vector<unsigned int> >* clusters, double* bestCost, std::mutex* mtx)
{
    this->first = first;
    this->last = last;
    this->clustersNumber = clustersNumber;
    this->points = points;
    this->clusters = clusters;
    this->bestCost = bestCost;
    this->mtx =mtx;
}

void ClusterThread::startThread()
{
    tid = new thread(executeTaskHelper, this);
}

void ClusterThread::waitThread()
{
    tid->join();
}

void ClusterThread::executeTask()
{
    unsigned int d = static_cast<unsigned int>((*points)[0].size());
    bool changed;

    for(unsigned int l = first; l < last; l++){

        std::vector<VectorXd> centroids;
        map<unsigned int, unsigned int> pointClusterMap;
        std::vector<std::vector<unsigned int> > tmpClusters;

        for(unsigned int i = 0; i < clustersNumber; i++){
            unsigned int pos = static_cast<unsigned int>(rand() % static_cast<int>(points->size()));
            centroids.push_back((*points)[pos]);
        }

        for(unsigned int i = 0; i < clustersNumber; i++){
            vector<unsigned int> cluster;
            tmpClusters.push_back(cluster);
        }


        do{

            changed = false;
            for(unsigned int i = 0; i < points->size(); i++){

                double best_distance = numeric_limits<double>::max();
                unsigned int pos = numeric_limits<unsigned int>::max();
                for(unsigned int j = 0; j < clustersNumber; j++){
                    VectorXd centroid(d);
                    centroid = centroids[j];
                    double distance = ((*points)[i] - centroid).norm();
                    if(distance < best_distance){
                        pos = j;
                        best_distance = distance;
                    }
                }

                std::vector<unsigned int>::iterator cit = std::find(tmpClusters[pos].begin(), tmpClusters[pos].end(), i);
                if(cit == tmpClusters[pos].end()){
                    std::vector<unsigned int>::iterator oit = std::find(tmpClusters[pointClusterMap[i]].begin(), tmpClusters[pointClusterMap[i]].end(), i);
                    if(oit != tmpClusters[pointClusterMap[i]].end())
                        tmpClusters[pointClusterMap[i]].erase(oit);
                    tmpClusters[pos].push_back(i);
                    pointClusterMap[i] = pos;
                    changed = true;
                }

            }

            for(unsigned int i = 0; i < clustersNumber; i++){

                VectorXd sum(d);
                sum = VectorXd::Zero(d);
                for(unsigned int j = 0; j < tmpClusters[i].size(); j++)
                    sum += (*points)[tmpClusters[i][j]];

                centroids[i] = sum / tmpClusters[i].size();

            }

        }while(changed);

        double cost = 0;
        for(unsigned int i = 0; i < points->size(); i++)
            cost += pow(((*points)[i] - centroids[pointClusterMap[i]]).norm(), 2);

        cost /= points->size();

        if(cost < *bestCost){
            mtx->lock();
            *bestCost = cost;
            clusters->clear();
            clusters->insert(clusters->end(), tmpClusters.begin(), tmpClusters.end());
            mtx->unlock();
        }
    }

}
