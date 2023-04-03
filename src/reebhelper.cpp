#include "reebhelper.h"
#include <fstream>

using namespace std;
using namespace IMATI_STL;
using namespace AndreasStructures;
ReebHelper::ReebHelper(IMATI_STL::TriMesh *mesh)
{
    mesh->save("tmp.wrl");
    system("/media/andreas/Volume/Progetti/andreaslib/CodiceRGArcs/RGNoGUI tmp 4 200 tmp.off 1");
    loadReebGraph();
}

void ReebHelper::loadReebGraph()
{
    ifstream myfile;
    string line;

    myfile.open("tmp.off.out");
    if (myfile.is_open()){
        map<uint, MultiplyConnectedNode<ReebData>* > nodes;
        while ( getline(myfile, line) ){
            if(line == "node"){
                MultiplyConnectedNode<ReebData>* newNode = new MultiplyConnectedNode<ReebData>();
                ReebData d;
                uint id;
                double x, y, z;
                stringstream stream;
                getline (myfile, line);
                stream << line;
                string bucket;
                stream >> bucket;
                stream >> id;
                newNode->setKey(id);
                getline (myfile, line);
                bucket.clear();
                stream.clear();
                stream << line;
                stream >> bucket;
                stream >> x;
                stream >> y;
                stream >> z;
                d.position = new IMATI_STL::Point(x, y, z);
                getline (myfile, line);
                bucket.clear();
                stream.clear();
                stream << line;
                stream >> bucket;
                stream >> d.nodeType;
                getline (myfile, line);
                bucket.clear();
                stream.clear();
                stream << line;
                stream >> bucket;
                stream >> d.length;
                getline (myfile, line);
                bucket.clear();
                stream.clear();
                stream << line;
                stream >> bucket;
                stream >> d.area;
                getline (myfile, line);
                bucket.clear();
                stream.clear();
                stream << line;
                stream >> bucket;
                stream >> d.quote;
                getline (myfile, line);
                bucket.clear();
                stream.clear();
                stream << line;
                stream >> bucket;
                stream >> d.boundary;
                newNode->setData(d);
                nodes[newNode->getKey()] = newNode;
            } else if (line.find("edge ") != std::string::npos){
                stringstream stream;
                string bucket;
                uint v1, v2, eid;
                int slicesNumber;
                stream >> bucket;
                stream >> eid;
                getline (myfile, line);
                stream.clear();
                stream << line.substr(1);
                stream >> v1;
                stream >> v2;
                getline (myfile, line);
                stream.clear();
                stream << line;
                stream >> bucket;
                stream >> slicesNumber;
                nodes[v1]->addConnectedNode(nodes[v2]);
                nodes[v2]->addConnectedNode(nodes[v1]);
            }
        }
        reebGraph = nodes[0];
    }
    myfile.close();

}

AndreasStructures::MultiplyConnectedNode<ReebData> *ReebHelper::getReebGraph() const
{
    return reebGraph;
}

void ReebHelper::setReebGraph(AndreasStructures::MultiplyConnectedNode<ReebData> *value)
{
    reebGraph = value;
}

