#ifndef CAGEHIERARCHYMANAGER_H
#define CAGEHIERARCHYMANAGER_H

#include <vector>
#include <treenode.h>
#include <extendedtrimesh.h>

class CageHierarchyManager
{
public:
    CageHierarchyManager();

    AndreasStructures::TreeNode<ExtendedTrimesh *> *getHierarchy() const;
    void setHierarchy(AndreasStructures::TreeNode<ExtendedTrimesh *> *value);
    std::vector<ExtendedTrimesh*> getCagesList();
    void addCage(ExtendedTrimesh * &value);

private:
    AndreasStructures::TreeNode<ExtendedTrimesh*>* hierarchy;
    unsigned int reachedKey;
    bool concurrentRoots;

    AndreasStructures::TreeNode<ExtendedTrimesh*>* searchPosition(AndreasStructures::TreeNode<ExtendedTrimesh*>*, ExtendedTrimesh*);
    bool checkContainment(const ExtendedTrimesh*, const ExtendedTrimesh*);
};

#endif // CAGEHIERARCHYMANAGER_H
