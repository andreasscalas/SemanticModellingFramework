#include "manode.h"

#include <imatistl.h>
#include <map>
#include <iostream>

using namespace AndreasStructures;
using namespace std;
using namespace IMATI_STL;

MANode::MANode(){
    markedFlag = false;
    visitedFlag = false;
    isTerminalNode = false;
    this->point = new IMATI_STL::Vertex();
}

MANode::MANode(MANode *n) : MANode()
{
    vector<MultiplyConnectedNode*> graph = n->breadthFirstVisit();
    vector<MANode*> newGraph;
    std::map<MANode*, MANode*> old_new;
    for (unsigned int i = 0; i < graph.size(); i++)
        graph[i]->setVisitedFlag(false);

    for (unsigned int i = 0; i < graph.size(); i++){
        MANode* oldNode = static_cast<MANode*>(graph[i]);
        MANode* newNode = new MANode();
        if(i == 0)
            newNode = this;
        newNode->setPoint(oldNode->getPoint());
        newNode->setTriangle(oldNode->getTriangle());
        newNode->setIsTerminalNode(oldNode->getIsTerminalNode());
        newNode->setMarkedFlag(false);
        newNode->setVisitedFlag(false);
        newNode->setKey(oldNode->getKey());
        newNode->setData(static_cast<Point*>(oldNode->getData()));
        old_new.insert(make_pair(oldNode, newNode));
        newGraph.push_back(newNode);
    }

    for(unsigned int i = 0; i < graph.size(); i++){

        MANode* oldNode = static_cast<MANode*>(graph[i]);
        MANode* newNode = newGraph[i];
        oldNode->setVisitedFlag(true);
        newNode->setVisitedFlag(true);

        vector<MultiplyConnectedNode*> connected = oldNode->getConnectedNodes();
        for(unsigned j = 0; j < connected.size(); j++){
            if(!connected[j]->getVisitedFlag()){
                newNode->addConnectedNode(old_new[static_cast<MANode*>(connected[j])]);
                old_new[static_cast<MANode*>(connected[j])]->addConnectedNode(newNode);
                MAArcPath* oldPath = oldNode->getCommonPath(static_cast<MANode*>(connected[j]));
                MAArcPath* newPath = new MAArcPath();
                newPath->setN1(old_new[static_cast<MANode*>(oldPath->getN1())]);
                newPath->setN2(old_new[static_cast<MANode*>(oldPath->getN2())]);
                newPath->setPath(oldPath->getPath());
                newPath->setIsTerminalPath(oldPath->getIsTerminalPath());
                newPath->setTraversedTriangles(oldPath->getTraversedTriangles());
                newPath->computeImportance();
                newNode->addPath(newPath);
                old_new[static_cast<MANode*>(connected[j])]->addPath(newPath);
            }
        }
    }


    for (unsigned int i = 0; i < graph.size(); i++){
        graph[i]->setVisitedFlag(false);
        newGraph[i]->setVisitedFlag(false);
    }

}

IMATI_STL::Vertex* MANode::getPoint() const
{
    return point;
}

void MANode::setPoint(const IMATI_STL::Vertex* value)
{
    *point = *value;
}

std::vector<AndreasStructures::MAArcPath *> MANode::getPaths() const
{
    return paths;
}

void MANode::setPaths(const std::vector<AndreasStructures::MAArcPath *> &value)
{
    paths = value;
}

void MANode::addPath(AndreasStructures::MAArcPath* path){

    this->paths.push_back(path);

}

void MANode::removePath(MAArcPath * path)
{
    for (unsigned int i = 0; i < paths.size(); i++)
        if(path == paths[i])
            paths.erase(paths.begin() + i);

}


bool MANode::getMarkedFlag() const
{
    return markedFlag;
}

void MANode::setMarkedFlag(bool value)
{
    markedFlag = value;
}

std::vector<AndreasStructures::MAArcPath*> MANode::getGraphArcs(){

    vector<MANode*> toWalkNodes = {this};
    vector<AndreasStructures::MAArcPath*> arcs;
    vector<MultiplyConnectedNode<IMATI_STL::Point*>*> graph = breadthFirstVisit();
    for(unsigned int i = 0; i < graph.size(); i++)
        graph[i]->setVisitedFlag(false);

    while(toWalkNodes.size() > 0){

        MANode* current = toWalkNodes.back();
        vector<AndreasStructures::MultiplyConnectedNode<Point*> *> connected = current->getConnectedNodes();
        toWalkNodes.pop_back();
        current->setMarkedFlag(true);
        for(vector<AndreasStructures::MultiplyConnectedNode<Point*> *>::iterator cit = connected.begin(); cit != connected.end(); cit++){

            MANode* connectedNode = static_cast<MANode*>(*cit);
            if(connectedNode->getMarkedFlag() == false){

                AndreasStructures::MAArcPath* arcPath = current->getCommonPath(connectedNode);
                vector<Point*> path = arcPath->getPath();
                arcPath->setPath(path);
                arcs.push_back(arcPath);
                if(connectedNode->getVisitedFlag() == false){

                    connectedNode->setVisitedFlag(true);
                    toWalkNodes.push_back(connectedNode);

                }

            }

        }

    }

    for(unsigned int i = 0; i < graph.size(); i++){
        graph[i]->setVisitedFlag(false);
        static_cast<MANode*>(graph[i])->setMarkedFlag(false);
    }

    return arcs;

}

AndreasStructures::MAArcPath* MANode::getCommonPath(MANode *other){

    vector<MAArcPath*> n2Paths = other->getPaths();

    for(vector<MAArcPath*>::iterator pit1 = this->paths.begin(); pit1 != this->paths.end(); pit1++)
        for(vector<MAArcPath*>::iterator pit2 = n2Paths.begin(); pit2 != n2Paths.end(); pit2++)
            if((*pit1) == (*pit2)){
                return (*pit1);
            }

    return nullptr;

}

bool MANode::getIsTerminalNode() const
{
    return isTerminalNode;
}

void MANode::setIsTerminalNode(bool value)
{
    isTerminalNode = value;
}

IMATI_STL::Triangle* MANode::getTriangle() const
{
    return triangle;
}

void MANode::setTriangle(IMATI_STL::Triangle* value)
{
    triangle = new IMATI_STL::Triangle(*value);
}

void MANode::print(ostream &stream)
{
    stream<<endl<<"*****************************************************************************************************************"<<endl;
    stream<<"Node "<<(long) this<<endl;
    stream<<"Point: ("<<point->x<<","<<point->y<<","<<point->z<<")"<<endl;
    stream<<"Triangle: [("<<triangle->v1()->x<<","<<triangle->v1()->y<<","<<triangle->v1()->z<<"), ("
                          <<triangle->v2()->x<<","<<triangle->v2()->y<<","<<triangle->v2()->z<<"), ("
                          <<triangle->v3()->x<<","<<triangle->v3()->y<<","<<triangle->v3()->z<<")]"<<endl;
    stream<<"Marked: "<<markedFlag<<endl;
    stream<<"Visited: "<<visitedFlag<<endl;
    stream<<"Terminal: "<<isTerminalNode<<endl;

    for(unsigned int i = 0; i < paths.size(); i++){
        stream<<"Path n°"<<i + 1<<":"<<endl;
        paths[i]->print(stream);
    }
    stream<<endl<<"*****************************************************************************************************************"<<endl;
}

bool MANode::getDeletedFlag() const
{
    return deletedFlag;
}

void MANode::setDeletedFlag(bool value)
{
    deletedFlag = value;
}
