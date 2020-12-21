#include "list.h"

template<class T>
bool List<T>::addElement(T element)
{
    Node<T>* newElem = new Node<T>();
    newElem->data = element;
    return addNode(newElem);
}

template<class T>
bool List<T>::removeElement(T element)
{
    Node<T>* toRemoveNode = searchNode(element);
    return removeNode(toRemoveNode);
}

template<class T>
bool List<T>::addNode(Node<T> *node)
{
    if(node == nullptr)
        return false;

    if(first == nullptr)
        first = node;
    else{
        Node<T>* cursor = first;
        while(cursor->next != nullptr)
            cursor = cursor->next;
        cursor->next = node;
    }
    return true;
}

template<class T>
bool List<T>::removeNode(Node<T> *node)
{
    if(node == nullptr)
        return false;

    Node<T>* cursor = first;
    if(first == node){
        first = first->next;
    } else {
        while(cursor->next != nullptr){
            if(cursor->next == node)
                cursor->next = cursor->next->next;
            cursor = cursor->next;
        }
    }

    return false;
}

template<class T>
Node<T> *List<T>::searchNode(T value)
{
    Node<T>* cursor = first;
    while(cursor != nullptr){
        if(cursor->data == value)
            return cursor;
        cursor = cursor->next;
    }

    return nullptr;

}
