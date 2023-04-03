#ifndef TREE_H
#define TREE_H

#include "graph.h"

namespace GraphTemplate {

template <typename T>
class TreeNode : public Node<T>{
public:
    TreeNode();
    TreeNode(const T data);
    TreeNode(const Node<T>* n);
    TreeNode(const TreeNode<T>* n);
    bool addParentalRelationship(Arc<T>* a);
    TreeNode<T> *getParent() const;
    void setParent(TreeNode<T> *value);

protected:
    TreeNode<T>* parent;

};

template <typename T>
class Tree : public Graph<T>{

public:
    Tree();
    virtual std::vector<Node<T> *> depthFirstVisit(Node<T>* root) override;

    bool getPreOrder() const;
    void setPreOrder(bool value);
    virtual bool addNode(T data) override;
    virtual bool addNode(Node<T>* n) override;
    virtual bool addNode(TreeNode<T>* n);
    virtual bool addArc(Arc<T>* a) override;
    virtual bool addArc(Node<T>* n1, Node<T>* n2, double weight, bool directed) override;

protected:
    bool preOrder;
};

template<typename T>
TreeNode<T>::TreeNode(const T data)
{
    this->data = data;
}

template<typename T>
TreeNode<T>::TreeNode(const Node<T> *n)
{
    this->data = n->getData();
    this->arcs = n->getArcs();
}

template<typename T>
TreeNode<T>::TreeNode(const TreeNode<T> *n)
{
    this->data = n->getData();
    this->arcs = n->getArcs();
    this->parent = n->getParent();
}

template<typename T>
bool TreeNode<T>::addParentalRelationship(Arc<T> *a)
{
    if(!a->isDirected() || a->getN1() != this)
        return false;
    this->arcs.push_back(a);
    return true;
}

template<typename T>

TreeNode<T> *TreeNode<T>::getParent() const
{
    return this->parent;
}

template<typename T>
void TreeNode<T>::setParent(TreeNode<T> *value)
{
    this->parent = value;
}



//Implementation of Tree methods

template<typename T>
Tree<T>::Tree() : Graph<T> ()
{
    this->preOrder = false;
}

template<typename T>
std::vector<Node<T> *> Tree<T>::depthFirstVisit(Node<T> *root)
{
    std::vector<Node<T>*> nodesList;
    std::vector<Arc<T>*> connectedArcs = root->getArcs();
    for(unsigned int i = 0; i < connectedArcs.size(); i++){
        std::vector<Node<T>*> childrenVisitList;
        if(root != connectedArcs[i]->getN1() && connectedArcs[i]->isDirected())
            return childrenVisitList;
        Node<T>* v = connectedArcs[i]->getOppositeNode(root);
        childrenVisitList = depthFirstVisit(v);

        if(childrenVisitList.size() == 0)
            return childrenVisitList;
        nodesList.insert(nodesList.end(), childrenVisitList.begin(), childrenVisitList.end());

    }

    if(this->preOrder)
        nodesList.push_back(root);
    else
        nodesList.insert(nodesList.begin(), root);


    return nodesList;

}


template<typename T>
bool Tree<T>::getPreOrder() const
{
return preOrder;
}

template<typename T>
void Tree<T>::setPreOrder(bool value)
{
    preOrder = value;
}

template<typename T>
bool Tree<T>::addNode(TreeNode<T> *n)
{
    this->nodes.push_back(n);
    return true;
}

template<typename T>
bool Tree<T>::addNode(T data)
{
    TreeNode<T>* newNode = new TreeNode<T>(data);
    return addNode(newNode);
}

template<typename T>
bool Tree<T>::addNode(Node<T> *n)
{
    TreeNode<T>* newNode = new TreeNode<T>(n);
    return addNode(newNode);
}

template<typename T>
bool Tree<T>::addArc(Arc<T> *a)
{
    if(!a->isDirected())
        return false;
    a->getN1()->addArc(a);
    static_cast<TreeNode<T>*>(a->getN2())->setParent(static_cast<TreeNode<T>*>(a->getN1()));
    a->getN2()->addArc(a);
    this->arcs.push_back(a);
    return true;
}

template<typename T>
bool Tree<T>::addArc(Node<T> *n1, Node<T> *n2, double weight, bool directed)
{
    if(!directed)
        return false;
    Arc<T>* a = new Arc<T>(n1, n2, weight, true);
    return addArc(a);
}

}

#endif // TREE_H
