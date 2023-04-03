#ifndef MANODE_H
#define MANODE_H

#include <multiplyconnectednode.h>
#include <maarcpath.h>
#include <imatistl.h>
#include <ostream>

namespace AndreasStructures {

    class MAArcPath;
    class MANode : public MultiplyConnectedNode<IMATI_STL::Point*>{

        public:

            MANode();

            MANode(MANode* n);

            IMATI_STL::Vertex* getPoint() const;
            void setPoint(const IMATI_STL::Vertex* value);

            void addPath(AndreasStructures::MAArcPath*);
            void removePath(AndreasStructures::MAArcPath*);
            std::vector<AndreasStructures::MAArcPath*> getPaths() const;
            void setPaths(const std::vector<AndreasStructures::MAArcPath *> &value);

            bool getMarkedFlag() const;
            void setMarkedFlag(bool value);

            std::vector<AndreasStructures::MAArcPath*> getGraphArcs();
            AndreasStructures::MAArcPath* getCommonPath(MANode* other);

            bool getIsTerminalNode() const;
            void setIsTerminalNode(bool value);

            IMATI_STL::Triangle* getTriangle() const;
            void setTriangle(IMATI_STL::Triangle* value);

            void print(std::ostream& stream);

            bool getDeletedFlag() const;
            void setDeletedFlag(bool value);

    protected:
            IMATI_STL::Vertex* point;
            IMATI_STL::Triangle* triangle;
            std::vector<AndreasStructures::MAArcPath*> paths;
            bool markedFlag;
            bool deletedFlag;
            bool isTerminalNode;

    };


}
#endif // MANODE_H
