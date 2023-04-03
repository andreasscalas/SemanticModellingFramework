#include "contourcontainmenttreenode.h"
#include <algorithm>
#include <imatistl.h>
#include <utilities.h>
using namespace std;
using namespace IMATI_STL;

ContourContainmentTreeNode::ContourContainmentTreeNode(){
    this->level = 0;
}

bool ContourContainmentTreeNode::addContainer(ContourContainmentTreeNode *container){
    this->containers.push_back(container);
	return true;
}

bool ContourContainmentTreeNode::removeContainer(ContourContainmentTreeNode *container){

    bool exists = false;
    std::vector<ContourContainmentTreeNode*>::iterator nit = std::find(this->containers.begin(), this->containers.end(), container);
    if(nit != this->containers.end()){
        this->containers.erase(nit);
        exists = true;
    }

    return exists;
}

std::vector<ContourContainmentTreeNode *> ContourContainmentTreeNode::getContainers(){
    return this->containers;
}

void ContourContainmentTreeNode::removeSonsCycles(){

    std::vector<bool> check;
    for(std::vector<TreeNode*>::iterator nit1 = this->sons.begin(); nit1 != this->sons.end(); nit1++){
        std::vector<ContourContainmentTreeNode*> containerList = (static_cast<ContourContainmentTreeNode*>(*nit1))->getContainers();
        bool isGrandson = false;
        for(std::vector<ContourContainmentTreeNode*>::iterator nit2 = containerList.begin(); nit2 != containerList.end(); nit2++){
            if((*nit2) != this){
                (static_cast<ContourContainmentTreeNode*>(*nit1))->removeContainer(this);
                this->removeSon((*nit1));
                nit1--;
                isGrandson = true;
                break;
            }

        }
        check.push_back(isGrandson);
    }

    for(unsigned int i = 0; i < check.size(); i++)
        if(!check[i])
            (static_cast<ContourContainmentTreeNode*>(sons[i]))->removeSonsCycles();


}

void ContourContainmentTreeNode::reorderBoundaries(int level){

    vector<Point*>* boundary = (static_cast<vector<Point*>* >(this->data));
    bool isClockwise = Utilities::isBoundaryClockwise(*boundary);
    this->level = level;
    if(level % 2 == 0 && isClockwise)
        std::reverse(boundary->begin(), boundary->end());
    else if(level % 2 != 0 && !isClockwise)
        std::reverse(boundary->begin(), boundary->end());

    ++level;

    for(std::vector<TreeNode*>::iterator nit = sons.begin(); nit != sons.end(); nit++)
        (static_cast<ContourContainmentTreeNode*>(*nit))->reorderBoundaries(level);
}

std::vector<ContourContainmentTreeNode *> ContourContainmentTreeNode::getSonsAtLevel(int level){
    vector<ContourContainmentTreeNode*> sons;
    if(level == this->level)
        sons.push_back(this);
    else{
        for(vector<ContourContainmentTreeNode*>::iterator nit = sons.begin(); nit != sons.end(); nit++){
            vector<ContourContainmentTreeNode*> sonSons = (*nit)->getSonsAtLevel(level);
            sons.insert(sons.end(), sonSons.begin(), sonSons.end());
        }
    }

    return sons;
}

vector<vector<Point *> *> ContourContainmentTreeNode::getContainedBoundaries(){
    vector<vector<Point *> *> boundaries = {static_cast<vector<Point*>* >(this->data)};
    for(vector<TreeNode<vector<Point*> *> *>::iterator nit = this->sons.begin(); nit != this->sons.end(); nit++){
        vector<vector<IMATI_STL::Point*> *> sonsBoundaries = (static_cast<ContourContainmentTreeNode*>(*nit))->getContainedBoundaries();
        boundaries.insert(boundaries.end(), sonsBoundaries.begin(), sonsBoundaries.end());
    }

    return boundaries;
}

std::vector<Slice*> ContourContainmentTreeNode::getContainedSlices(){

    vector<Slice*> slices;
    if(level % 2 == 0){
        Slice* s = new Slice();
        s->setBoundary(*(static_cast<vector<Point*>*>(this->data)));
        for(vector<TreeNode*>::iterator nit = sons.begin(); nit != sons.end(); nit++)
            s->addHole(*static_cast<vector<Point*>*>((*nit)->getData()));
        slices.push_back(s);
    }
    for(vector<TreeNode*>::iterator nit = sons.begin(); nit != sons.end(); nit++){
        vector<Slice*> containedSlices = (static_cast<ContourContainmentTreeNode*>(*nit))->getContainedSlices();
        slices.insert(slices.end(), containedSlices.begin(), containedSlices.end());
    }

    return slices;

}


