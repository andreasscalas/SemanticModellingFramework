#ifndef TREENODE_H
#define TREENODE_H
#include <vector>
#include <queue>
#include <algorithm>
#include <multiplyconnectednode.h>

namespace AndreasStructures {

    template <class T>
    class TreeNode : public MultiplyConnectedNode<T> {

        protected:
            TreeNode<T>* father;
            std::vector<TreeNode<T>*> sons;

        public:
            TreeNode() = default;

            TreeNode<T> *getFather() const{
                return father;
            }

            void setFather(TreeNode<T> *value){
                if(this->connectedNodes.front()->getKey() == 0)
                    this->connectedNodes.front() = father;
                else
                    this->connectedNodes.insert(this->connectedNodes.begin(), value);
                father = value;
            }

            std::vector<TreeNode<T> *> getSons() const
            {
                return sons;
            }

            void setSons(const std::vector<TreeNode<T> *> &value){
                this->connectedNodes.insert(this->connectedNodes.begin() + 1, value.begin(), value.end());
                sons = value;
            }

            void addSon(TreeNode<T>* value){
                this->connectedNodes.push_back(value);
                this->sons.push_back(value);
            }

            bool removeSon(TreeNode<T>* son){
                bool exists = false;
                typename std::vector<TreeNode<T>*>::iterator nit = std::find(this->sons.begin(), this->sons.end(), son);
                if(nit != this->sons.end()){
                    this->sons.erase(nit);
                    exists = true;
                }

                return exists;
            }

            TreeNode* getRoot(){
                TreeNode* root;
                if(this->father != nullptr)
                    root = this->father->getRoot();
                else
                    root = this;

                return root;
            }

            std::vector<TreeNode<T>*> preOrderVisit(){
                std::vector<TreeNode *> offspring;

                offspring.push_back(this);

                for(typename std::vector<TreeNode*>::iterator it = sons.begin(); it != sons.end(); it++){
                    std::vector<TreeNode*> sonOffspring = (*it)->preOrderVisit();
                    offspring.insert(offspring.end(), sonOffspring.begin(), sonOffspring.end());
                }

                return offspring;
            }

            std::vector<TreeNode<T>*> postOrderVisit(){
                std::vector<TreeNode *> offspring;

                for(typename std::vector<TreeNode*>::iterator it = sons.begin(); it != sons.end(); it++){
                    std::vector<TreeNode*> sonOffspring = (*it)->postOrderVisit();
                    offspring.insert(offspring.end(), sonOffspring.begin(), sonOffspring.end());
                }

                offspring.push_back(this);

                return offspring;
            }

            std::vector<TreeNode<T>*> breadthFirstVisit(){
                std::vector<TreeNode<T>*> offspring;
                std::queue<TreeNode *> Q;
                Q.push(this);
                while(Q.size() > 0){
                    TreeNode* n = Q.front();
                    offspring.push_back(n);
                    std::vector<TreeNode*> sons = n->getSons();
                    for(unsigned int i = 0; i < sons.size(); i++)
                        Q.push(sons[i]);
                }

            }


    };
}


#endif // TREENODE_H
