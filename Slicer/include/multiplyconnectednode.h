#ifndef MULTIPLYCONNECTEDNODE_H
#define MULTIPLYCONNECTEDNODE_H

#include <vector>
#include <queue>

namespace AndreasStructures{

    template<class T>
    class MultiplyConnectedNode{

        public:
            MultiplyConnectedNode() = default;


            std::vector<MultiplyConnectedNode *> getConnectedNodes() const{
                return connectedNodes;
            }

            void setConnectedNodes(const std::vector<MultiplyConnectedNode *> &value){
                connectedNodes = value;
            }

            void removeConnectedNode(MultiplyConnectedNode* value){
                for(unsigned int i = 0; i < connectedNodes.size(); i++){
                    if(connectedNodes[i] == value)
                        connectedNodes.erase(connectedNodes.begin() + i);
                }
            }

            void addConnectedNode(MultiplyConnectedNode* value){
                connectedNodes.push_back(value);
            }

            void *getData() const{
                return data;
            }

            void setData(T value){
                data = value;
            }

            unsigned int getKey() const{
                return key;
            }

            void setKey(unsigned int value){
                key = value;
            }

            std::vector<MultiplyConnectedNode*> depthFirstVisit(){
                std::vector<MultiplyConnectedNode*> visitedNodes;
                visitedNodes.push_back(this);
                this->setVisitedFlag(true);
                for (unsigned int i = 0; i < connectedNodes.size(); i++) {
                    if(!connectedNodes[i]->getVisitedFlag()){
                        std::vector<MultiplyConnectedNode*> connectedVisited = connectedNodes[i].depthFirstVisit();
                        visitedNodes.insert(visitedNodes.begin(), connectedVisited.begin(), connectedVisited.end());
                    }
                }
                return visitedNodes;
            }

            std::vector<MultiplyConnectedNode*> breadthFirstVisit(){

                std::queue<MultiplyConnectedNode*> Q;
                Q.push(this);
                std::vector<MultiplyConnectedNode*> visitedNodes;
                this->setVisitedFlag(true);
                while(Q.size() > 0){
                    MultiplyConnectedNode* n = Q.front();
                    Q.pop();
                    visitedNodes.push_back(n);
                    for(unsigned int i = 0; i < n->getConnectedNodes().size(); i++) {
                        if(!n->getConnectedNodes()[i]->getVisitedFlag()){
                            n->getConnectedNodes()[i]->setVisitedFlag(true);
                            Q.push(n->getConnectedNodes()[i]);
                        }
                    }
                }

                return visitedNodes;
            }

            bool getVisitedFlag() const
            {
                return visitedFlag;
            }

            void setVisitedFlag(bool value)
            {
                visitedFlag = value;
            }

    protected:
            std::vector<MultiplyConnectedNode*> connectedNodes;
            T data;
            unsigned int key;
            bool visitedFlag;
    };



}
#endif // MULTIPLYCONNECTEDNODE_H
