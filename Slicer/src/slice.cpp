#include "slice.h"
#include "utilities.h"
using namespace std;
using namespace IMATI_STL;
Slice::Slice(){

    Point* a = new Point(0, 0, 0);
    vector<Point*> b = {a, a, a, a};
    this->boundingBox.insert(this->boundingBox.end(), b.begin(), b.end());
    this->minDiagonal = make_pair(a,a);
    this->maxDiagonal = make_pair(a,a);

}

void Slice::addHole(const std::vector<T_MESH::Point*> hole){
    this->holes.push_back(hole);
}

void Slice::triangulate(){

    TriHelper::TriangleHelper helper(boundary, holes);
    slice = new ExtendedTrimesh();
    slice->load("prova.ply");
    //slice = new ExtendedTrimesh(helper.getSliceMesh());
    //slice->fixConnectivity();

}

ExtendedTrimesh *Slice::getSlice() const
{
    return slice;
}

AndreasStructures::MANode *Slice::getSkeleton() const
{
    return skeleton;
}

void Slice::setSkeleton(AndreasStructures::MANode *value)
{
    skeleton = value;
}

void Slice::printInformation(){

    updateDescriptors();
    /*cout<<"Eccentricity: "<< (maxDiagonal.second - maxDiagonal.first).length() / (minDiagonal.second - minDiagonal.first).length()<<endl<<flush;
    getBestFittedCircles();
    Point center = slice->getCenter();
    Point circleCenter = bestFittedCircles[0].first;
    cout<<"Distance between ideal and real center: "<<(center - circleCenter).length()<<endl<<flush;
    vector<double> distances;
    for(unsigned int i  = 0; i < holes.size(); i++)
        distances.push_back(0);

    for(unsigned int i  = 0; i < boundary.size(); i++){
        for(unsigned int j = 0; j < holes.size(); j++){
            double bestDistance = DBL_MAX;
            for(unsigned int k = 0; k < holes[j].size(); k++){
                double distance = (boundary[i] - holes[j][k]).length();
                if(distance < bestDistance){
                    bestDistance = distance;

                }

            }
            distances[j] += bestDistance;
        }
    }

    for(unsigned int i  = 0; i < holes.size(); i++){
        distances[i] /= boundary.size();
        cout<<"Mean distance from "<<i + 1<<"th hole: "<< distances[i]<<endl<<flush;
    }
    std::cout<< "Area: "<< area << endl << flush;
    std::cout<< "Euler's number: "<< eulerNumber << endl << flush;
    std::cout<< "Hole-area ratio: "<< holeAreaRatio<< endl << flush;
    std::cout<< "Solidity: " << solidity<< endl << flush;
    std::cout<< "Compactness: " << compactness << endl << flush;
    std::cout<< "Circle variance: " << circleVariance << endl << flush;
    std::cout<< "Rectangularity: " << rectangularity << endl << flush;
    std::cout<< "Elongation: " << elongation << endl << flush;*/

}

vector<Point*> Slice::getBoundingBox(){
    return this->boundingBox;
}

vector<Point*> Slice::getConvexHull(){
    return convexHull;
}

pair<Point*, Point*> Slice::getMinDiagonal() const{
    return minDiagonal;
}

pair<Point*, Point*> Slice::getMaxDiagonal() const{
    return maxDiagonal;
}

Eigen::Matrix4d Slice::getFromPlaneTransformation() const
{
    return fromPlaneTransformation;
}

void Slice::setFromPlaneTransformation(const Eigen::Matrix4d &value)
{
    fromPlaneTransformation = value;
}

std::vector<IMATI_STL::Point*> Slice::getBoundary() const{
    return boundary;
}

void Slice::setBoundary(const std::vector<IMATI_STL::Point*> &value){
    boundary = value;
}

std::vector<std::vector<IMATI_STL::Point*> > Slice::getHoles() const{
    return holes;
}

void Slice::setHoles(const std::vector<std::vector<IMATI_STL::Point*> > &value){
    holes = value;
}

Eigen::VectorXd Slice::getFeatureVector(){
    updateDescriptors();
    Eigen::VectorXd v(7);
    v << /*area, perimeter, */eulerNumber, holeAreaRatio, solidity, compactness, circleVariance, rectangularity, elongation;
    return v;
}

double Slice::getPerimeter() const
{

    return perimeter;
}

double Slice::getArea() const
{
    return area;
}

double Slice::getHoleAreaRatio() const
{
    return holeAreaRatio;
}

double Slice::getSolidity() const
{
    return solidity;
}

double Slice::getCompactness() const
{
    return compactness;
}
double Slice::getCircleVariance() const
{
    return circleVariance;
}

double Slice::getRectangularity() const
{
    return rectangularity;
}


double Slice::getElongation() const
{
    return elongation;
}

double Slice::getThickness() const
{
    return thickness;
}

void Slice::setThickness(double value)
{
    thickness = value;
}

void Slice::updateDescriptors()
{
    computeBoundingBox();
    computeConvexHull();
    vector<double> holesArea;
    double holesTotalArea = 0;
    Slice* s = new Slice();
    s->setBoundary(boundary);
    s->triangulate();
    area = s->getSlice()->area();
    delete(s);

    eulerNumber = static_cast<int>(1 - holes.size());
    thickness = 0.0;

    if(holes.size() > 0){

        if(holes.size() == 1)
        {
            for(unsigned int i = 0; i < this->boundary.size(); i++)
            {

                unsigned int bestPos = INT_MAX;
                double bestDist = DBL_MAX;
                for(unsigned int j = 0; j < holes[0].size(); j++)
                {
                    double dist = ((*this->boundary[i]) - (*holes[0][j])).length();
                    if(dist < bestDist){
                        bestDist = dist;
                        bestPos = j;
                    }

                }
                thickness += bestDist;
            }
            thickness /= holes[0].size();
            std::cout << "Thickness: " << thickness << std::endl << std::flush;
        }

        for(unsigned int i = 0; i < holes.size(); i++){
            if(holes[i].size() > 2){
                s = new Slice();
                s->setBoundary(holes[i]);
                s->triangulate();
                holesArea.push_back(s->getSlice()->area());
                holesTotalArea += holesArea[i];
                delete(s);
            }
        }

        holeAreaRatio = holesTotalArea / area;

    }else
        holeAreaRatio = 0;


    s = new Slice();
    s->setBoundary(convexHull);
    s->triangulate();
    double convexHullArea = s->getSlice()->area();
    delete(s);
    Point c = slice->getCenter();
    perimeter = 0;
    double equivalentCircleRadius = sqrt(area / M_PI);

    circleVariance = 0;
    for(unsigned int i = 1; i < static_cast<unsigned int>(boundary.size()); i++){
        perimeter += (*(boundary[i]) - *(boundary[i - 1])).length();
        circleVariance += abs((*(boundary[i]) - c).length() - equivalentCircleRadius);
    }

    double rectangleArea = ((*(this->boundingBox[1]) - *(this->boundingBox[0])) &
                            (*(this->boundingBox[2]) - *(this->boundingBox[1]))).length();
    solidity = area / convexHullArea;
    compactness = 4 * M_PI * area / perimeter;
    rectangularity = area / rectangleArea;
    elongation = (*(maxDiagonal.second) - *(maxDiagonal.first)).length() /
                 (*(minDiagonal.second) - *(minDiagonal.first)).length();

}

void Slice::computeBoundingBox(){
    vector<Eigen::Vector2d> points = Utilities::imatiToEigen2DPointRepresentation(boundary);

    vector<Eigen::Vector2d> boundingBox = Utilities::get2DOBB(points);

    this->boundingBox.clear();
    for(unsigned int i = 0; i < static_cast<unsigned int>(boundingBox.size()); i++){
        Point* p = new Point(boundingBox[i](0), boundary[0]->y, boundingBox[i](1));
        this->boundingBox.push_back(p);
    }

    Eigen::Vector2d mp1 = (boundingBox[1] + boundingBox[0]) / 2;
    Eigen::Vector2d mp2 = (boundingBox[2] + boundingBox[1]) / 2;
    Eigen::Vector2d mp3 = (boundingBox[3] + boundingBox[2]) / 2;
    Eigen::Vector2d mp4 = (boundingBox[0] + boundingBox[3]) / 2;

    Point* p1 = new Point(mp1(0), boundary[0]->y, mp1(1));
    Point* p2 = new Point(mp2(0), boundary[0]->y, mp2(1));
    Point* p3 = new Point(mp3(0), boundary[0]->y, mp3(1));
    Point* p4 = new Point(mp4(0), boundary[0]->y, mp4(1));

    if((mp3 - mp1).norm() < (mp4 - mp2).norm()){

        minDiagonal = make_pair(p1, p3);
        maxDiagonal = make_pair(p2, p4);

    }else{

        minDiagonal = make_pair(p2, p4);
        maxDiagonal = make_pair(p1, p3);

    }
}

void Slice::computeConvexHull(){
    vector<Eigen::Vector2d> points = Utilities::imatiToEigen2DPointRepresentation(boundary);
    vector<Eigen::Vector2d> convexHull = Utilities::extractConvexHull(points);
    this->convexHull.clear();
    for(unsigned int i = 0; i < static_cast<unsigned int>(convexHull.size()); i++){
        Point* p = new Point(convexHull[i](0), boundary[0]->y, convexHull[i](1));
        this->convexHull.push_back(p);
    }
    this->convexHull.push_back(this->convexHull.front());
}

vector<pair<Point*, double> > Slice::getBestFittedCircles(){

    if(bestFittedCircles.size() == 0){

        vector<Eigen::Vector2d> boundaryPoints = Utilities::imatiToEigen2DPointRepresentation(boundary);
        pair<Eigen::Vector2d, double> br = Utilities::circleFitting(boundaryPoints);
        Point* boundaryCircleCenter = new Point(br.first(0), boundary[0]->y, br.first(1));
        bestFittedCircles.push_back(make_pair(boundaryCircleCenter, br.second));

        for(unsigned int i = 0; i < holes.size(); i++){

            vector<Eigen::Vector2d> holePoints = Utilities::imatiToEigen2DPointRepresentation(holes[i]);
            pair<Eigen::Vector2d, double> hr = Utilities::circleFitting(holePoints);
            Point* holeCircleCenter = new Point(hr.first(0), boundary[0]->y, hr.first(1));
            bestFittedCircles.push_back(make_pair(holeCircleCenter, hr.second));

        }
    }

    return bestFittedCircles;
}

