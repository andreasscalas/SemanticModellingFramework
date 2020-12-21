#ifndef CIRCULARLIST_H
#define CIRCULARLIST_H

#include <stdlib.h>
#include <iterator>
#include <doublyconnectednode.h>

template <class T>
class CircularList
{
public:
    CircularList();

    struct iterator;
    struct const_iterator : public std::iterator<std::bidirectional_iterator_tag, const T>
    {
        const_iterator() = default;
        T operator*() { return itm->data; }
        const T* operator->() { return &(itm->data); }
        const_iterator operator++() { itm = itm->next; return *this; }
        const_iterator operator--() { itm = itm->prev; return *this; }
        const_iterator operator++(int) { const_iterator ret=*this; itm = itm->next; return ret; }
        const_iterator operator--(int) { const_iterator ret=*this; itm = itm->prev; return ret; }
        bool operator==(const_iterator oth) const { return itm==oth.itm; }
        bool operator!=(const_iterator oth) const { return itm!=oth.itm; }
    private:
        AndreasStructures::DoublyConnectedNode<T>* itm = nullptr;
        const_iterator(AndreasStructures::DoublyConnectedNode<T>* i) : itm(i) {}
    friend
        class CircularList;
    friend
        struct iterator;
    };
    struct iterator : public std::iterator<std::bidirectional_iterator_tag, T>
    {
        iterator() = default;
        T& operator*() { return itm->data; }
        T* operator->() { return &(itm->data); }
        iterator operator++() { itm = itm->next; return *this; }
        iterator operator--() { itm = itm->prev; return *this; }
        iterator operator++(int) { iterator ret=*this; itm = itm->next; return ret; }
        iterator operator--(int) { iterator ret=*this; itm = itm->prev; return ret; }
        bool operator==(iterator oth) const { return itm==oth.itm; }
        bool operator!=(iterator oth) const { return itm!=oth.itm; }
        operator const_iterator() { return {itm}; }
    private:
        AndreasStructures::DoublyConnectedNode<T>* itm = nullptr;
        iterator(AndreasStructures::DoublyConnectedNode<T>* i) : itm(i) {}
    friend
        class CircularList;
    };

    typedef std::reverse_iterator<iterator> reverse_iterator;
    typedef std::reverse_iterator<const_iterator> const_reverse_iterator;



    AndreasStructures::DoublyConnectedNode<T>* searchItem(T item);
    void eraseItem(T item);
    void insertBefore(T item);
    void insertAfter(T item);
    void clear();
    unsigned long getSize() const;
    iterator begin(){
        return list;
    }
    iterator end(){
        return list;
    }

    iterator at(T item){
        return searchItem(item);
    }

private:
    unsigned long size;
    AndreasStructures::DoublyConnectedNode<T>* list;

    void initialize(T item);
};

template<class T>
CircularList<T>::CircularList()
{
    list = new AndreasStructures::DoublyConnectedNode<T>();
    clear();
}

template<class T>
AndreasStructures::DoublyConnectedNode<T> *CircularList<T>::searchItem(T item)
{
    AndreasStructures::DoublyConnectedNode<T>* n = list->next;

    if(list->data == item){
        return list;
    }

    while(n != list){
        if(n->data == item){
            return n;
        }
        n = n->next;
    }

    return new AndreasStructures::DoublyConnectedNode<T>();
}

template<class T>
void CircularList<T>::eraseItem(T item)
{
    AndreasStructures::DoublyConnectedNode<T>* toErase = searchItem(item);

    if(toErase == nullptr)
        return;

    size--;

    if(toErase == list)
        list = list->next;

    if(toErase->prev != nullptr)
        toErase->prev->next = toErase->next;
    if(toErase->next != nullptr){
        toErase->next->prev = toErase->prev;
    }

    delete(toErase);
}

template<class T>
void CircularList<T>::insertBefore(T item)
{
    if(size == 0)
        initialize(item);
    else{
        AndreasStructures::DoublyConnectedNode<T>* newNode = new AndreasStructures::DoublyConnectedNode<T>();
        if(list->next != nullptr)
            list->next->prev = newNode;
        newNode->data = item;
        newNode->next = list->next;
        newNode->prev = list;
        list->next = newNode;
    }

    size++;

}

template<class T>
void CircularList<T>::insertAfter(T item)
{
    if(size == 0)
        initialize(item);
    else{
        AndreasStructures::DoublyConnectedNode<T>* newNode = new AndreasStructures::DoublyConnectedNode<T>();
        if(list->prev != nullptr)
            list->prev->next = newNode;
        newNode->data = item;
        newNode->prev = list->prev;
        newNode->next = list;
        list->prev =(newNode);
    }

    size++;
}

template<class T>
void CircularList<T>::clear()
{
    list->data = NULL;
    list->prev = nullptr;
    list->next = nullptr;
    size = 0;
}

template<class T>
unsigned long CircularList<T>::getSize() const
{
    return size;
}

template<class T>
void CircularList<T>::initialize(T item)
{
    list->data = item;
    list->prev = list;
    list->next = list;
}


#endif // CIRCULARLIST_H
