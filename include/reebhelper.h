#ifndef REEBHELPER_H
#define REEBHELPER_H
#include <vector>
#include <map>
#include <string>
#include <slice.h>
#include <multiplyconnectednode.h>
#include <imatistl.h>

struct ReebData{
    IMATI_STL::Point* position;
    double length;
    double area;
    double quote;
    double boundary;
    unsigned int nodeType;
};

class ReebHelper
{
public:
    ReebHelper(IMATI_STL::TriMesh* mesh);
    AndreasStructures::MultiplyConnectedNode<ReebData> *getReebGraph() const;
    void setReebGraph(AndreasStructures::MultiplyConnectedNode<ReebData> *value);

private:
    AndreasStructures::MultiplyConnectedNode<ReebData>* reebGraph;

    void loadReebGraph();

};

#endif // REEBHELPER_H
