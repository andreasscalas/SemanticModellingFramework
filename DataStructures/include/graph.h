#ifndef GRAPH_H
#define GRAPH_H
#include "node.h"
#include "arc.h"
#include <iterator>
#include <vector>
#include <map>
#include <queue>
#include <stack>
#include <limits>

namespace GraphTemplate {

template <class T>
class Graph
{
public:
    Graph();
    Graph(Graph<T>&);
    Graph(Graph<T>*);
    virtual std::vector<Node<T> *> breadthFirstVisit(Node<T>* root);
    virtual std::vector<Node<T> *> depthFirstVisit(Node<T>* root);
    virtual Node<T> *breadthFirstSearch(Node<T>* root, T data);
    virtual Node<T> *depthFirstSearch(Node<T>* root, T data);
    /**
     * @brief shortestPathSearch method for computing the shortest path between two given nodes in the graph.
     * It is based on an approximation of the Dijkstra algorithm.
     * @param source starting node of the path
     * @param destination destination of the path
     * @param label optional parameter, allows to restrain the search to arcs having a specific label
     * @return the found shortest path
     */
    virtual std::vector<Arc<T>*> shortestPathSearch(Node<T>* source, Node<T>* destination, std::string label = "");
    virtual bool addNode(Node<T>* n);
    virtual bool addNode(T data);
    virtual bool addArc(Arc<T>* a);
    virtual bool addArc(Node<T>* n1, Node<T>* n2, double weight, bool directed);
    virtual bool removeArc(Arc<T>* a);

    std::vector<Arc<T>*> getArcsFromFletching(Node<T>* n, std::string type = "");
    std::vector<Arc<T>*> getArcsFromTip(Node<T>* n, std::string type = "");
    //Arc<T>* getArcFromEndpoints(Node<T>* n1, Node<T>* n2, std::string type = "");
    std::vector<Arc<T>*> getArcsFromEndpoints(Node<T>* n1, Node<T>* n2, std::string type = "");
    std::vector<Node<T>* > getLeaves(Node<T>*);
    void removeRedundancies();

    Node<T> * getNodeFromData(T data);
    std::vector<Node<T> *> getNodes() const;
    void setNodes(const std::vector<Node<T> *> &value);
    std::vector<Arc<T> *> getArcs() const;
    void setArcs(const std::vector<Arc<T> *> &value);

protected:
    unsigned int reachedNodeId;
    unsigned int reachedArcId;
    std::vector<Node<T>*> nodes;
    std::vector<Arc<T>*> arcs;

    void toLowerCase(std::string&);
};


template<class T>
Graph<T>::Graph()
{

}

template<class T>
Graph<T>::Graph(Graph<T> &graph)
{
    this->nodes.insert(this->nodes.end(), graph.getNodes().begin(), graph.getNodes().end());
    this->arcs.insert(this->arcs.end(), graph.getArcs().begin(), graph.getArcs().end());
}

template<class T>
Graph<T>::Graph(Graph<T> *graph)
{
    std::vector<Node<T>*> nodes = graph->getNodes();
    std::vector<Arc<T>*> arcs = graph->getArcs();
    this->nodes.insert(this->nodes.end(), nodes.begin(), nodes.end());
    this->arcs.insert(this->arcs.end(), arcs.begin(), arcs.end());
}

template<class T>
std::vector<Node<T> *> Graph<T>::breadthFirstVisit(Node<T>* root)
{
    std::vector<Node<T>*> nodesList = {root};
    std::queue<Node<T>*> Q;
    Q.push(root);
    root->setVisited(true);
    while(!Q.empty()){
        Node<T>* w = Q.front();
        std::vector<Arc<T>*> connectedArcs = w->getArcs();
        Q.pop();
        for(unsigned int i = 0; i < connectedArcs.size(); i++){
            if(w != connectedArcs[i]->getN1() && connectedArcs[i]->isDirected())
                continue;
            Node<T>* v = connectedArcs[i]->getOppositeNode(w);
            if(!v->isVisited()){
                v->setVisited(true);
                nodesList.push_back(v);
                Q.push(v);
            }
        }
    }

    for(unsigned int i = 0; i < nodes.size(); i++)
        nodes[i]->setVisited(false);

    return nodesList;
}

template<class T>
std::vector<Node<T> *> Graph<T>::depthFirstVisit(Node<T>* root)
{
    std::vector<Node<T>*> nodesList;
    std::stack<Node<T>*> S;
    S.push(root);
    while(!S.empty()){
        Node<T>* w = S.top();
        S.pop();
        std::vector<Arc<T>*> connectedArcs = w->getArcs();
        if(!w->isVisited()){
            w->setVisited(true);
            nodesList.push_back(w);
            for(int i = connectedArcs.size() - 1; i >= 0; i--){
                if(w != connectedArcs[i]->getN1() && connectedArcs[i]->isDirected())
                    continue;
                Node<T>* v = connectedArcs[i]->getOppositeNode(w);
                if(!v->isVisited())
                    S.push(v);

            }
        }
    }

    for(unsigned int i = 0; i < nodes.size(); i++)
        nodes[i]->setVisited(false);

    return nodesList;

}

template<class T>
Node<T> *Graph<T>::depthFirstSearch(Node<T> *root, T data)
{
    std::stack<Node<T>*> S;
    S.push(root);
    Node<T>* foundNode = nullptr;

    while(!S.empty()){
        Node<T>* w = S.top();
        S.pop();
        std::vector<Arc<T>*> connectedArcs = w->getArcs();
        if(!w->isVisited()){
            if(w->getData() == data){
                foundNode = w;
                break;
            }
            w->setVisited(true);
            for(int i = connectedArcs.size() - 1; i >= 0; i--){
                if(w != connectedArcs[i]->getN1() && connectedArcs[i]->isDirected())
                    continue;
                Node<T>* v = connectedArcs[i]->getOppositeNode(w);
                if(!v->isVisited())
                    S.push(v);

            }
        }
    }

    for(unsigned int i = 0; i < nodes.size(); i++)
        nodes[i]->setVisited(false);

    return foundNode;
}

template<class T>
Node<T> *Graph<T>::breadthFirstSearch(Node<T> *root, T data)
{
    std::vector<Node<T>*> nodesList = {root};
    std::queue<Node<T>*> Q;
    Node<T>* foundNode = nullptr;
    Q.push(root);
    root->setVisited(true);
    while(!Q.empty() && foundNode == nullptr ){
        Node<T>* w = Q.front();
        std::vector<Arc<T>*> connectedArcs = w->getArcs();
        Q.pop();
        for(unsigned int i = 0; i < connectedArcs.size(); i++){
            if(w != connectedArcs[i]->getN1() && connectedArcs[i]->isDirected())
                continue;
            Node<T>* v = connectedArcs[i]->getOppositeNode(w);
            if(!v->isVisited()){
                if(w->getData() == data){
                    foundNode = w;
                    break;
                }
                v->setVisited(true);
                nodesList.push_back(v);
                Q.push(v);
            }
        }
    }

    for(unsigned int i = 0; i < nodes.size(); i++)
        nodes[i]->setVisited(false);

    return foundNode;
}

template<class T>
Node<T>* extractNearestNode(std::vector<Node<T>*> &frontier, std::map<Node<T>*, double> distances){

    double minDist = std::numeric_limits<double>::max();
    int minPos = -1;
    for(unsigned int i = 0; i < frontier.size(); i++){
        if(distances[frontier[i]] < minDist){
            minPos = static_cast<int>(i);
            minDist = distances[frontier[i]];
        }
    }

    Node<T>* nearest = frontier[minPos];

    frontier.erase(frontier.begin() + minPos);

    return nearest;

}

template<class T>
std::vector<Arc<T> *> Graph<T>::shortestPathSearch(Node<T> *source, Node<T> *destination, std::string label)
{
    std::string lowerCaseLabel = label;
    toLowerCase(lowerCaseLabel);
    std::vector<Node<T>*> frontier;
    std::map<Node<T>*, double> distances = {{source, 0}};
    std::map<Node<T>*, Node<T>*> predecessors = {{source, nullptr}};
    std::vector<Arc<T>*> shortestPath;
    Node<T>* v;
    bool v2visited = false;

    if(source == destination)
        return shortestPath;


    frontier.push_back(source);

    do{
        v = extractNearestNode(frontier, distances);
        std::vector<Node<T>*> neighbors;
        for(unsigned int i = 0; i < v->getArcs().size(); i++)
        {
            Arc<T>* a = v->getArcs()[i];
            std::string aLowerCaseLabel = a->getLabel();
            toLowerCase(aLowerCaseLabel);
            if((label.compare("") == 0 || aLowerCaseLabel.find(lowerCaseLabel) != std::string::npos) &&
               (!a->isDirected() || a->getN1() == v))
            {
                neighbors.push_back(a->getOppositeNode(v));
            }
        }

        for(unsigned int i = 0; i < neighbors.size(); i++)
        {
            Node<T>* x = neighbors[i];

            typename std::map<Node<T>*, Node<T>*>::iterator pit = predecessors.find(x);
            double distanceVX = x->getArc(v)->getWeight();

            if(pit != predecessors.end()){
                if(distances[x] > distanceVX){
                    distances[x] = distanceVX;
                    predecessors[x] = v;
                }
            } else {
                distances.insert(std::make_pair(x, distanceVX));
                predecessors.insert(std::make_pair(x, v));
                frontier.push_back(x);
            }
        }
        if(v == destination)
            v2visited = true;

    } while(!v2visited && frontier.size() > 0);

    if(!v2visited)
        return shortestPath;

    Node<T>* v_ = destination;
    v = predecessors[destination];
    shortestPath.push_back(v->getArc(v_));

    while(v != source && v != nullptr){
        v_ = v;
        v  = predecessors[v];
        shortestPath.insert(shortestPath.begin(), v->getArc(v_));
    }

    return shortestPath;

}

template<class T>
bool Graph<T>::addNode(Node<T> *n)
{
    nodes.push_back(n);
    return true;
}

template<class T>
bool Graph<T>::addNode(T data)
{
    Node<T>* n = new Node<T>(data);
    addNode(n);
    return true;
}

template<class T>
bool Graph<T>::addArc(Arc<T>* a)
{
    a->getN1()->addArc(a);
    a->getN2()->addArc(a);
    this->arcs.push_back(a);
    return true;
}

template<class T>
bool Graph<T>::addArc(Node<T> *n1, Node<T> *n2, double weight, bool directed)
{
    Arc<T>* a = new Arc<T>(n1, n2, weight, directed);
    addArc(a);
    return true;
}

template<class T>
bool Graph<T>::removeArc(Arc<T> *a)
{
    typename std::vector<Arc<T>*>::iterator it;
    for(it = this->arcs.begin(); it != this->arcs.end(); it++)
        if(*it == a)
            break;

    if(it != this->arcs.end()){
        (*it)->getN1()->removeArc(*it);
        (*it)->getN2()->removeArc(*it);
        this->arcs.erase(it);
        return true;
    }

    return false;

}

template<class T>
std::vector<Arc<T> *> Graph<T>::getArcsFromFletching(Node<T> *n, std::string type)
{
    std::vector<Arc<T> *> arcs;
    toLowerCase(type);
    for(unsigned int i = 0; i < this->arcs.size(); i++)
    {
        std::string label = this->arcs[i]->getLabel();
        toLowerCase(label);
        if(this->arcs[i]->getN1() == n &&
          (type.compare("") == 0 || type.compare(label) == 0))
            arcs.push_back(this->arcs[i]);
    }
    return arcs;
}

template<class T>
std::vector<Arc<T> *> Graph<T>::getArcsFromTip(Node<T> *n, std::string type)
{
    std::vector<Arc<T> *> arcs;
    toLowerCase(type);
    for(unsigned int i = 0; i < this->arcs.size(); i++)
    {
        std::string label = this->arcs[i]->getLabel();
        toLowerCase(label);
        if(this->arcs[i]->getN2() == n &&
          (type.compare("") == 0 || type.compare(label) == 0))
            arcs.push_back(this->arcs[i]);
    }
    return arcs;
}

/*template<class T>
Arc<T> *Graph<T>::getArcFromEndpoints(Node<T> *n1, Node<T> *n2, std::string type)
{
    std::vector<Arc<T> *> arcs = getArcsFromFletching(n1);
    toLowerCase(type);
    for(unsigned int i = 0; i < arcs.size(); i++)
    {
        std::string label = this->arcs[i]->getLabel();
        toLowerCase(label);
        if(arcs[i]->getN2() == n2 &&
          (type.compare("") == 0 || type.compare(label) == 0))
            return arcs[i];
    }

    return nullptr;
}*/

template<class T>
std::vector<Arc<T> *> Graph<T>::getArcsFromEndpoints(Node<T> *n1, Node<T> *n2, std::string type)
{
    if(n1 == n2)
    {
        std::vector<Arc<T> *> empty;
        return empty;
    }
    std::vector<Arc<T> *> arcs = n1->getArcs();
    toLowerCase(type);
    for(unsigned int i = 0; i < arcs.size(); i++)
    {
        std::string label = this->arcs[i]->getLabel();
        toLowerCase(label);
        if(arcs[i]->getN1() == n1 && arcs[i]->getN2() != n2 ||
           arcs[i]->getN2() == n1 && arcs[i]->getN1() != n2 ||
          (type.compare("") != 0 && type.compare(label) != 0))
            arcs.erase(arcs.begin() + i--);
    }

    return arcs;
}

template<class T>
std::vector<Node<T> *> Graph<T>::getLeaves(Node<T> *root)
{
    std::vector<Node<T>*> leaves;
    std::vector<Arc<T>* > exitingArcs = this->getArcsFromFletching(root);
    if(exitingArcs.size() == 0)
        leaves.push_back(root);
    else
        for(unsigned int i = 0; i < exitingArcs.size(); i++)
        {
            std::vector<Node<T>* > sonLeaves = getLeaves(exitingArcs[i]->getN2());
            leaves.insert(leaves.end(), sonLeaves.begin(), sonLeaves.end());
        }

    return leaves;
}

template<class T>
void Graph<T>::removeRedundancies()
{
    for(unsigned int i = 0; i < nodes.size(); i++)
        for(unsigned int j = 0; j < nodes.size(); j++)
            for(unsigned int z = 0; z < nodes.size(); z++)
                if(nodes[i]->getArc(nodes[j]) != nullptr &&
                   nodes[j]->getArc(nodes[z]) != nullptr &&
                   nodes[i]->getArc(nodes[z]) != nullptr)
                    removeArc(nodes[i]->getArc(nodes[z]));
}

template<class T>
Node<T> *Graph<T>::getNodeFromData(T data)
{
    for(unsigned int i = 0; i < nodes.size(); i++)
        if(nodes[i]->getData() == data)
            return nodes[i];
    return nullptr;
}

template<class T>
std::vector<Node<T> *> Graph<T>::getNodes() const
{
    return nodes;
}

template<class T>
void Graph<T>::setNodes(const std::vector<Node<T> *> &value)
{
    this->nodes = value;
}

template<class T>
std::vector<Arc<T> *> Graph<T>::getArcs() const
{
    return arcs;
}

template<class T>
void Graph<T>::setArcs(const std::vector<Arc<T> *> &value)
{
    this->arcs = value;
}

template<class T>
void Graph<T>::toLowerCase(std::string &s)
{
    char* c_s = new char[s.size() + 1];
    for(unsigned int i = 0; i < s.size(); i++)
    {
        c_s[i] = static_cast<char>(std::tolower(s[i]));
    }
    c_s[s.size()] = '\0';
    s = c_s;
}

}
#endif // GRAPH_H
