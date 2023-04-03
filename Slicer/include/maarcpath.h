#ifndef MAARCPATH_H
#define MAARCPATH_H

#include <manode.h>
#include <imatistl.h>
#include <ostream>

namespace AndreasStructures {

    class MANode;
    class MAArcPath
    {
        public:
            static const short LOCAL_BOUNDARY_DIM = 7;

            MAArcPath();

            void addPoint(IMATI_STL::Point* point);
            std::vector<IMATI_STL::Point*> getPath() const;
            void setPath(const std::vector<IMATI_STL::Point*> &value);

            AndreasStructures::MANode* getN1() const;
            void setN1(AndreasStructures::MANode* value);

            AndreasStructures::MANode* getN2() const;
            void setN2(AndreasStructures::MANode *value);

            void addTriangle(IMATI_STL::Triangle* triangle);
            std::vector<IMATI_STL::Triangle*> getTraversedTriangles() const;
            void setTraversedTriangles(const std::vector<IMATI_STL::Triangle*> &value);

            bool getIsTerminalPath() const;
            void setIsTerminalPath(bool value);
            void computeImportance();

            double getImportance() const;

            void print(std::ostream &stream);

    protected:
            AndreasStructures::MANode* n1;
            AndreasStructures::MANode* n2;
            std::vector<IMATI_STL::Point*> path;
            std::vector<IMATI_STL::Triangle*> traversedTriangles;
            bool isTerminalPath;
            double importance;
    };

}

#endif // MAARCPATH_H
