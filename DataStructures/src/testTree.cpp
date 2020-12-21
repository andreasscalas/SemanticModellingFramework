#include <iostream>
#include <vector>
#include "node.h"
#include "tree.h"

int main(int argc, char *argv[]){

    GraphTemplate::Tree<int>* tree = new GraphTemplate::Tree<int>();
    std::vector<GraphTemplate::Node<int>*> list;
    for(unsigned int i = 0; i < 10; i++){
        GraphTemplate::TreeNode<int>* n = new GraphTemplate::TreeNode<int>(i);
        tree->addNode(n);
        list.push_back(n);
    }


    tree->addArc(list[0], list[1], 1, true);
    tree->addArc(list[0], list[2], 1, true);
    tree->addArc(list[1], list[3], 1, true);
    tree->addArc(list[1], list[4], 1, true);
    tree->addArc(list[2], list[5], 1, true);
    tree->addArc(list[2], list[6], 1, true);
    tree->addArc(list[3], list[7], 1, true);
    tree->addArc(list[4], list[8], 1, true);
    tree->addArc(list[5], list[9], 1, true);
    std::vector<GraphTemplate::Node<int>*> BFVList = tree->breadthFirstVisit(list[0]);
    std::cout << "Breadth-first visit" << std::endl;
    for(unsigned int i = 0; i < BFVList.size(); i++)
        std::cout << BFVList[i]->getData() << std::endl << std::flush;
    std::vector<GraphTemplate::Node<int>*> DFVList = tree->depthFirstVisit(list[0]);
    std::cout << "In-order depth-first visit" << std::endl;
    for(unsigned int i = 0; i < DFVList.size(); i++)
        std::cout << DFVList[i]->getData() << std::endl << std::flush;
    tree->setPreOrder(true);
    DFVList = tree->depthFirstVisit(list[0]);
    std::cout << "Pre-order depth-first visit" << std::endl;
    for(unsigned int i = 0; i < DFVList.size(); i++)
        std::cout << DFVList[i]->getData() << std::endl << std::flush;
    tree->setPreOrder(true);


}
