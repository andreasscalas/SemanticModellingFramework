#ifndef LIST_H
#define LIST_H

template <class T>
class Node
{
public:
    T data;
    double weight;
    Node* next;
};

template <class T>
class List
{
public:
    bool addElement(T element);
    bool removeElement(T element);
protected:
    Node<T>* first;

    bool addNode(Node<T>* node);
    bool removeNode(Node<T>* node);
    Node<T>* searchNode(T value);
};

#endif // LIST_H
