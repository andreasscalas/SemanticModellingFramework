#ifndef CNODE_H
#define CNODE_H

#include <stdlib.h>
template <class T>
class CNode{

    public:
        CNode();
        void insertBefore(T item);
        void insertAfter(T item);
        void erase();
        void eraseUntil(CNode<T>* end);
        void eraseFrom(CNode<T>* begin);

        T getItem() const;
        void setItem(const T &value);

        CNode<T> *getPrev() const;
        void setPrev(CNode<T> *value);

        CNode<T> *getNext() const;
        void setNext(CNode<T> *value);

private:
        T item;
        CNode<T>* prev, * next;
};

template<class T>
CNode<T>::CNode()
{
    item = NULL;
    prev = nullptr;
    next = nullptr;
}

template<class T>
void CNode<T>::insertBefore(T item)
{
    CNode* newNode = new CNode();
    if(prev != nullptr)
        prev->setNext(newNode);
    newNode->setItem(item);
    newNode->setPrev(prev);
    newNode->setNext(this);
    this->setPrev(newNode);
}

template<class T>
void CNode<T>::insertAfter(T item)
{
    CNode* newNode = new CNode();
    if(next != nullptr)
        next->setPrev(newNode);
    newNode->setItem(item);
    newNode->setNext(next);
    newNode->setPrev(this);
    this->setNext(newNode);

}

template<class T>
void CNode<T>::erase()
{
    if(prev != nullptr)
        prev->setNext(next);
    if(next != nullptr){
        next->setPrev(prev);
    }

    delete(this);
}

template<class T>
void CNode<T>::eraseUntil(CNode<T>* end)
{
    this->next = end;
    end->prev = this;
}

template<class T>
void CNode<T>::eraseFrom(CNode<T>* begin)
{
    this->prev = begin;
    begin->next = this;
}

template<class T>
T CNode<T>::getItem() const
{
    return item;
}

template<class T>
void CNode<T>::setItem(const T &value)
{
    item = value;
}

template<class T>
CNode<T> *CNode<T>::getPrev() const
{
    return prev;
}

template<class T>
void CNode<T>::setPrev(CNode<T> *value)
{
    prev = value;
}

template<class T>
CNode<T> *CNode<T>::getNext() const
{
    return next;
}

template<class T>
void CNode<T>::setNext(CNode<T> *value)
{
    next = value;
}


#endif // CNODE_H
