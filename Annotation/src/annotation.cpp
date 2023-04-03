#include <annotation.h>
#include <graph.h>

void Annotation::print(std::ostream &os)
{
   os << "id: " << id << std::endl;
   os << "tag: " << tag << std::endl;
   //os << "level: " << hierarchyLevel << std::endl;
   os << "attributes:" << std::endl;
   if(attributes.size() != 0){
       os << "[" << std::endl;
       for(unsigned int i = 0; i < attributes.size(); i++)
           attributes[i]->print(os);
       os << "]";
   }
   os << std::endl;
}

void Annotation::printJson(rapidjson::PrettyWriter<rapidjson::StringBuffer> &writer)
{
    writer.Key("id");
    writer.Uint(id);
    writer.Key("tag");
    writer.String(tag.c_str());
    GraphTemplate::Graph<Annotation*>* graph = mesh->getGraph();
    if(graph != nullptr){
        writer.Key("father");
        GraphTemplate::Node<Annotation*>* node = graph->getNodeFromData(this);
        std::vector<GraphTemplate::Arc<Annotation*>*> enteringArcs = graph->getArcsFromTip(node);
        bool isRoot = true;
        for(unsigned int i = 0; i < enteringArcs.size(); i++){
            std::size_t found = enteringArcs[i]->getLabel().find("Containment");
            if (found != std::string::npos){
                isRoot = false;
                writer.Int(enteringArcs[i]->getN1()->getData()->getId());
            }
        }
        if(isRoot)
            writer.Int(-1);
        writer.Key("level");
        writer.Uint(hierarchyLevel);

    }
    writer.Key("color");
    writer.StartArray();
    writer.Int(color[0]);
    writer.Int(color[1]);
    writer.Int(color[2]);
    writer.EndArray();

    writer.Key("attributes");
    writer.StartArray();
    for(unsigned int i = 0; i < attributes.size(); i++)
    {
        writer.StartObject();
        attributes[i]->printJson(writer);
        writer.EndObject();
    }
    writer.EndArray();
}
