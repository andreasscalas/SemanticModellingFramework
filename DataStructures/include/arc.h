#ifndef ARC_H
#define ARC_H

#include "node.h"

namespace GraphTemplate {

template <class T>
class Node;

template <class T>
class Arc
{
public:
    Arc();
    Arc(Node<T>* n1, Node<T>* n2, double weight, bool directed, unsigned int id = 0, std::string label = "");

    Node<T> *getOppositeNode(Node<T>* n);

    Node<T> *getN1() const;
    void setN1(Node<T> *value);
    bool isDirected() const;
    void setDirected(bool value);
    Node<T> *getN2() const;
    void setN2(Node<T> *value);
    double getWeight() const;
    void setWeight(double value);
    void *getInfo() const;
    void setInfo(void *value);
    unsigned int getId() const;
    void setId(unsigned int value);
    std::string getLabel() const;
    void setLabel(const std::string &value);

protected:
    unsigned int id;
    Node<T> *n1, *n2;
    std::string label;
    bool directed;
    double weight;
    void* info;
};


template<class T>
Arc<T>::Arc(Node<T> *n1, Node<T> *n2, double weight, bool directed, unsigned int id, std::string label)
{
    this->id = id;
    this->n1 = n1;
    this->n2 = n2;
    this->label = label;
    this->weight = weight;
    this->directed = directed;
}

template<class T>
Node<T> *Arc<T>::getOppositeNode(Node<T> *n)
{
    if(n == n1)
        return n2;

    return n1;
}

template <class T>
Node<T> *Arc<T>::getN1() const
{
    return n1;
}

template <class T>
void Arc<T>::setN1(Node<T> *value)
{
    n1 = value;
}

template <class T>
Node<T> *Arc<T>::getN2() const
{
    return n2;
}

template <class T>
void Arc<T>::setN2(Node<T> *value)
{
    n2 = value;
}

template <class T>
bool Arc<T>::isDirected() const
{
    return directed;
}

template <class T>
void Arc<T>::setDirected(bool value)
{
    directed = value;
}

template <class T>
double Arc<T>::getWeight() const
{
    return weight;
}

template <class T>
void Arc<T>::setWeight(double value)
{
    weight = value;
}

template <class T>
void* Arc<T>::getInfo() const
{
    return info;
}

template <class T>
void Arc<T>::setInfo(void *value)
{
    info = value;
}

template <class T>
unsigned int Arc<T>::getId() const
{
return id;
}

template <class T>
void Arc<T>::setId(unsigned int value)
{
id = value;
}

template <class T>
std::string Arc<T>::getLabel() const
{
return label;
}

template <class T>
void Arc<T>::setLabel(const std::string &value)
{
label = value;
}

}
#endif // ARC_H
