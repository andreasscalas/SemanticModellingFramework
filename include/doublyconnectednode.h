#ifndef DOUBLYCONNECTEDNODE_H
#define DOUBLYCONNECTEDNODE_H

namespace AndreasStructures{
    template <class T>
    class DoublyConnectedNode{
        public:
            T data;
            DoublyConnectedNode<T>* prev, * next;
    };
}

#endif // DOUBLYCONNECTEDNODE_H
