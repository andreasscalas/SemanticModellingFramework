#include "treenode.h"

using namespace AndreasStructures;

TreeNode::TreeNode()
{

}

TreeNode *TreeNode::getFather() const
{
    return father;
}

void TreeNode::setFather(TreeNode *value)
{
    if(this->connectedNodes.front()->getKey() == 0)
        this->connectedNodes.front() = father;
    else
        this->connectedNodes.insert(this->connectedNodes.begin(), value);
    father = value;
}

std::vector<TreeNode *> TreeNode::getSons() const
{
    return sons;
}

void TreeNode::setSons(const std::vector<TreeNode *> &value)
{
    this->connectedNodes.insert(this->connectedNodes.begin() + 1, value.begin(), value.end());
    sons = value;
}

void TreeNode::addSon(TreeNode *value)
{
    this->connectedNodes.push_back(value);
    this->sons.push_back(value);
}

bool TreeNode::removeSon(TreeNode *son)
{
    bool exists = false;
    std::vector<TreeNode*>::iterator nit = std::find(this->sons.begin(), this->sons.end(), son);
    if(nit != this->sons.end()){
        this->sons.erase(nit);
        exists = true;
    }

    return exists;
}

TreeNode *TreeNode::getRoot()
{
    TreeNode* root;
    if(this->father != nullptr)
        root = this->father->getRoot();
    else
        root = this;

    return root;
}

std::vector<TreeNode *> TreeNode::preOrderVisit()
{
    std::vector<TreeNode *> offspring;

    offspring.push_back(this);

    for(std::vector<TreeNode*>::iterator it = sons.begin(); it != sons.end(); it++){
        std::vector<TreeNode*> sonOffspring = (*it)->preOrderVisit();
        offspring.insert(offspring.end(), sonOffspring.begin(), sonOffspring.end());
    }

    return offspring;
}

std::vector<TreeNode *> TreeNode::postOrderVisit()
{
    std::vector<TreeNode *> offspring;

    for(std::vector<TreeNode*>::iterator it = sons.begin(); it != sons.end(); it++){
        std::vector<TreeNode*> sonOffspring = (*it)->postOrderVisit();
        offspring.insert(offspring.end(), sonOffspring.begin(), sonOffspring.end());
    }

    offspring.push_back(this);

    return offspring;
}
