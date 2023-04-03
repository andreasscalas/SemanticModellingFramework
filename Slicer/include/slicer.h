#ifndef SLICER_H
#define SLICER_H

#include <extendedtrimesh.h>

#include <slice.h>
#include <vector>

class Slicer{

    public:
        Slicer();

        void slice();

        IMATI_STL::Point getNormal() const;
        void setNormal(const IMATI_STL::Point &value);

        IMATI_STL::Point getCenter() const;
        void setCenter(const IMATI_STL::Point &value);

        ExtendedTrimesh *getMesh() const;
        void setMesh(ExtendedTrimesh *value);

        std::vector<Slice*> getSlices() const;
        void setSlices(const std::vector<Slice*> &value);

        double getMaxSimplificationError() const;
        void setMaxSimplificationError(double value);

        Eigen::Matrix4d getTransformationMatrix() const;
        void setTransformationMatrix(const Eigen::Matrix4d &value);

private:

        std::vector<Slice*> slices;
        IMATI_STL::Point normal;
        IMATI_STL::Point center;
        Eigen::Matrix4d transformationMatrix;
        ExtendedTrimesh* mesh;
        double maxSimplificationError;

};

#endif // SLICER_H
