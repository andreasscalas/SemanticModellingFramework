#include "cagehierarchymanager.h"

using namespace std;
using namespace AndreasStructures;

CageHierarchyManager::CageHierarchyManager()
{

    hierarchy = new TreeNode<ExtendedTrimesh*>();
    hierarchy->setFather(nullptr);
    hierarchy->setKey(0);
    hierarchy->addSon(nullptr);
    reachedKey = 1;
    concurrentRoots = false;

}

AndreasStructures::TreeNode<ExtendedTrimesh *> *CageHierarchyManager::getHierarchy() const
{
    return hierarchy;
}

void CageHierarchyManager::setHierarchy(AndreasStructures::TreeNode<ExtendedTrimesh *> *value)
{
    hierarchy = value;
}


std::vector<ExtendedTrimesh *> CageHierarchyManager::getCagesList()
{
    vector<ExtendedTrimesh*> cagesList;
    vector<TreeNode<ExtendedTrimesh*> *> tree = hierarchy->breadthFirstVisit();

    for(unsigned int i = 0; i < tree.size(); i++)
        cagesList.push_back(static_cast<ExtendedTrimesh*>(tree[i]->getData()));

    return  cagesList;

}

void CageHierarchyManager::addCage(ExtendedTrimesh *&value)
{
    TreeNode<ExtendedTrimesh*>* position;
    TreeNode<ExtendedTrimesh*>* newNode = new TreeNode<ExtendedTrimesh*>();
    newNode->setData(value);
    newNode->setKey(reachedKey);

    if(concurrentRoots){

        bool contains = true;
        for(unsigned int i = 0; i < hierarchy->getSons().size(); i++)
            if(!checkContainment(value, static_cast<ExtendedTrimesh*>(hierarchy->getSons()[i]->getData()))){
                contains = false;
                break;
            }
        if(contains){
            for(unsigned int i = 0; i < hierarchy->getSons().size(); i++){
                newNode->addSon(hierarchy->getSons()[i]);
                hierarchy->getSons()[i]->setFather(newNode);
                hierarchy->removeSon(hierarchy->getSons()[i]);
            }
            hierarchy->addSon(newNode);
            concurrentRoots = false;
            return;
        }

        for(unsigned int i = 0; i < hierarchy->getSons().size(); i++)
            if(checkContainment(value, static_cast<ExtendedTrimesh*>(hierarchy->getSons()[i]->getData())))
                position = hierarchy->getSons()[i];

    }else{

        position = hierarchy->getSons()[0];

    }

    if(position == nullptr){
        hierarchy->removeSon(position);
        hierarchy->addSon(newNode);
        return;
    }else{
        if(checkContainment(static_cast<ExtendedTrimesh*>(position->getData()), value)){
            searchPosition(position, value);
            if(position != nullptr){
                newNode->setFather(position);
                position->addSon(newNode);
            }
        } else if(checkContainment(value, static_cast<ExtendedTrimesh*>(position->getData()))){
            position->setFather(newNode);
            newNode->addSon(position);
            hierarchy->removeSon(position);
            hierarchy->addSon(newNode);
        } else{
            hierarchy->addSon(newNode);
            concurrentRoots = true;
        }
    }


}

AndreasStructures::TreeNode<ExtendedTrimesh *> *CageHierarchyManager::searchPosition(TreeNode<ExtendedTrimesh*>* position, ExtendedTrimesh* cage)
{
    if(position != nullptr)
        if(position->getSons().size() != 0)
            for(unsigned int i = 0; i < position->getSons().size(); i++)
                if(checkContainment(static_cast<ExtendedTrimesh*>(position->getSons()[i]->getData()), cage))
                    return searchPosition(position->getSons()[i], cage);

    return position;
}

bool CageHierarchyManager::checkContainment(const ExtendedTrimesh *, const ExtendedTrimesh *)
{
    return true;

}
