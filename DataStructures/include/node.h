#ifndef NODE_H
#define NODE_H


#include "arc.h"
#include <map>
#include <vector>
#include <iterator>
#include <climits>
namespace GraphTemplate {


template <class T>
class Node
{
public:
    Node();
    Node(const T data);

    T getData() const;
    void setData(T value);
    bool isVisited() const;
    void setVisited(bool value);
    std::vector<Arc<T> *> getTypedArcs(std::string type) const;
    std::vector<Arc<T> *> getArcs() const;
    void addArc(Arc<T>* value);
    void removeArc(Arc<T>* value);
    void setArcs(const std::vector<Arc<T> *> &value);
    Arc<T>* getArc(Node<T>* n);

protected:
    T data;
    std::vector<Arc<T>* > arcs;
    bool visited;
};

template<class T>
Node<T>::Node()
{
    visited = false;
}

template<class T>
Node<T>::Node(const T data)
{
    this->data = data;
    visited = false;
}

template<class T>
T Node<T>::getData() const
{
    return this->data;
}

template<class T>
void Node<T>::setData(T value)
{
    this->data = value;
}

template<class T>
bool Node<T>::isVisited() const
{
    return visited;
}

template<class T>
void Node<T>::setVisited(bool value)
{
    visited = value;
}

template<class T>
std::vector<Arc<T> *> Node<T>::getTypedArcs(std::string type) const
{
    std::vector<Arc<T> *> typedArcs;
    for(unsigned int i = 0; i < arcs.size(); i++)
        if(arcs[i]->label.compare(type) == 0)
            typedArcs.push_back(arcs[i]);
}

template<class T>
std::vector<Arc<T> *> Node<T>::getArcs() const
{
    return arcs;
}

template<class T>
void Node<T>::addArc(Arc<T> *value)
{
    arcs.push_back(value);
}

template<class T>
void Node<T>::removeArc(Arc<T> *value)
{
    typename std::vector<Arc<T>*>::iterator it;
    for(it = this->arcs.begin(); it != this->arcs.end(); it++)
        if(*it == value)
            break;
    if(it != this->arcs.end())
        arcs.erase(it);
}

template<class T>
void Node<T>::setArcs(const std::vector<Arc<T> *> &value)
{
    arcs = value;
}

template<class T>
Arc<T>* Node<T>::getArc(Node<T> *n)
{
    for(unsigned int i = 0; i < arcs.size(); i++)
        if((arcs[i]->getN1() == this && arcs[i]->getN2() == n) ||
           (arcs[i]->getN1() == n && arcs[i]->getN2() == this) )
            return arcs[i];

    return nullptr;
}

}

#endif // NODE_H
