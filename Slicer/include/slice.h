#ifndef SLICE_H
#define SLICE_H

#include <vector>
#include <Eigen/Dense>
#include <trianglehelper.h>
#include <extendedtrimesh.h>
#include <manode.h>

class Slice
{
public:
    Slice();


    void addHole(const std::vector<IMATI_STL::Point*> hole);
    void triangulate();
    void computeBoundingBox();
    void computeConvexHull();

    ExtendedTrimesh *getSlice() const;

    AndreasStructures::MANode *getSkeleton() const;
    void setSkeleton(AndreasStructures::MANode *value);
    void printInformation();

    std::vector<IMATI_STL::Point*> getBoundingBox();
    std::vector<IMATI_STL::Point*> getConvexHull();
    std::pair<IMATI_STL::Point*, IMATI_STL::Point*> getMinDiagonal() const;
    std::pair<IMATI_STL::Point*, IMATI_STL::Point*> getMaxDiagonal() const;
    std::vector<std::pair<IMATI_STL::Point*, double> > getBestFittedCircles();


    Eigen::Matrix4d getFromPlaneTransformation() const;
    void setFromPlaneTransformation(const Eigen::Matrix4d &value);

    std::vector<IMATI_STL::Point*> getBoundary() const;
    void setBoundary(const std::vector<IMATI_STL::Point*> &value);

    std::vector<std::vector<IMATI_STL::Point*> > getHoles() const;
    void setHoles(const std::vector<std::vector<IMATI_STL::Point*> > &value);

    Eigen::VectorXd getFeatureVector();

    double getPerimeter() const;
    double getArea() const;
    double getHoleAreaRatio() const;
    double getSolidity() const;
    double getCompactness() const;
    double getCircleVariance() const;
    double getRectangularity() const;
    double getElongation() const;

    double getThickness() const;
    void setThickness(double value);

private:

    ExtendedTrimesh* slice;
    std::vector<IMATI_STL::Point*> boundary;
    std::vector<std::vector<IMATI_STL::Point*> > holes;
    AndreasStructures::MANode* skeleton;
    std::vector<IMATI_STL::Point*> boundingBox;
    std::vector<IMATI_STL::Point*> convexHull;
    std::vector<std::pair<IMATI_STL::Point*, double> > bestFittedCircles;
    std::pair<IMATI_STL::Point*, IMATI_STL::Point*> minDiagonal;
    std::pair<IMATI_STL::Point*, IMATI_STL::Point*> maxDiagonal;
    Eigen::Matrix4d fromPlaneTransformation;

    /*************Slice descriptors**************/

    int eulerNumber;
    double perimeter;
    double area;
    double holeAreaRatio;
    double solidity;
    double compactness;
    double circleVariance;
    double rectangularity;
    double elongation;
    double thickness;


    void updateDescriptors();

};

#endif // SLICE_H
