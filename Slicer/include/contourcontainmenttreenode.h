#ifndef CONTOURCONTAINMENTTREENODE_H
#define CONTOURCONTAINMENTTREENODE_H
#include <vector>
#include <slice.h>
#include <treenode.h>

class Slice;

class ContourContainmentTreeNode : public AndreasStructures::TreeNode<std::vector<IMATI_STL::Point*>* >{
    public:
        ContourContainmentTreeNode();
        bool addContainer(ContourContainmentTreeNode* container);
        bool removeContainer(ContourContainmentTreeNode* container);
        std::vector<ContourContainmentTreeNode*> getContainers();
        void removeSonsCycles();
        void reorderBoundaries(int level);
        std::vector<ContourContainmentTreeNode*> getSonsAtLevel(int level);
        std::vector<std::vector<IMATI_STL::Point*> * > getContainedBoundaries();
        std::vector<Slice*> getContainedSlices();

private:
        std::vector<ContourContainmentTreeNode*> containers;
        int level, numberOfLevels;
};

#endif // CONTOURCONTAINMENTTREENODE_H
