#include <iostream>
#include <node.h>
#include <graph.h>
#include <vector>

int main(int argc, char *argv[]){

    GraphTemplate::Graph<int>* graph = new GraphTemplate::Graph<int>();
    std::vector<GraphTemplate::Node<int>*> list;
    for(unsigned int i = 0; i < 10; i++){
        GraphTemplate::Node<int>* n = new GraphTemplate::Node<int>(i);
        graph->addNode(n);
        list.push_back(n);
    }


    graph->addArc(list[0], list[1], 1, true);
    graph->addArc(list[0], list[2], 1, true);
    graph->addArc(list[3], list[1], 1, true);
    graph->addArc(list[1], list[4], 1, true);
    graph->addArc(list[2], list[5], 1, false);
    graph->addArc(list[2], list[6], 1, true);
    graph->addArc(list[3], list[7], 1, true);
    graph->addArc(list[4], list[5], 1, true);
    graph->addArc(list[4], list[8], 1, true);
    graph->addArc(list[5], list[9], 1, true);
    std::vector<GraphTemplate::Node<int>*> BFVList = graph->breadthFirstVisit(list[0]);
    std::vector<GraphTemplate::Node<int>*> DFVList = graph->depthFirstVisit(list[0]);
    std::cout << "Breadth-first visit" << std::endl;
    for(unsigned int i = 0; i < BFVList.size(); i++)
        std::cout << BFVList[i]->getData() << std::endl << std::flush;

    std::cout << "Depth-first visit" << std::endl;
    for(unsigned int i = 0; i < DFVList.size(); i++)
        std::cout << DFVList[i]->getData() << std::endl << std::flush;
}
