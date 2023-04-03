#ifndef DRAWABLEGRAPH_H
#define DRAWABLEGRAPH_H

#include <string>
#include <DataStructures/include/graph.h>
#include <Eigen/Dense>
#include <vtkSmartPointer.h>
#include <vtkPropAssembly.h>
#include <vtkRegularPolygonSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkLine.h>
#include <vtkProperty.h>
#include <vtkVectorText.h>
#include <vtkFollower.h>

namespace GraphTemplate {

template <typename T>
class DrawableGraph : public Graph<T>
{
public:
    DrawableGraph();
    ~DrawableGraph();
    DrawableGraph(Graph<T> &graph);
    DrawableGraph(Graph<T> *graph);
    DrawableGraph(DrawableGraph<T> &graph);
    DrawableGraph(DrawableGraph<T> *graph);

    double computeRepulsiveForce(double d);
    double computeAttractiveForce(double d);
    double cool(double temperature, unsigned int iteration);
    void computeVertexRepulsionPlacement();
    std::vector<std::vector<unsigned int> > computeOrderedAdjacencies();
    double computeAngle(Eigen::Vector2d, Eigen::Vector2d);
    void computeEdgeRepulsionPlacement();

    virtual void update();
    virtual void updateView();
    virtual void draw(vtkSmartPointer<vtkPropAssembly> assembly);

    bool getUseMainLabel() const;
    void setUseMainLabel(bool value);

    std::string getMainLabel() const;
    void setMainLabel(const std::string &value);


    double getArcsWidth() const;
    void setArcsWidth(double value);

    double getNodesRadius() const;
    void setNodesRadius(double value);

    static const unsigned int ITERATIONS_MAX = 100;
    static const unsigned int CIRCLE_RESOLUTION = 50;

    vtkSmartPointer<vtkPropAssembly> getCanvas() const;
    void setCanvas(const vtkSmartPointer<vtkPropAssembly> &value);

    double getFrameLength() const;
    void setFrameLength(double value);

    double getFrameWidth() const;
    void setFrameWidth(double value);

    Eigen::MatrixX2d getNodesPositions() const;
    void setNodesPositions(const Eigen::MatrixX2d &value);

private:
    bool useMainLabel;
    std::string mainLabel;
    double nodesRadius;
    double arcsWidth;
    double frameLength;
    double frameWidth;
    double optimalDistance;

    Eigen::MatrixX2d nodesPositions;
    Eigen::MatrixX2d nodesDisplacements;

    std::map<Node<T>*, vtkIdType> nodeToID;
    vtkSmartPointer<vtkPoints> nodesPoints;
    vtkSmartPointer<vtkCellArray> arcsLines;
    std::vector<vtkSmartPointer<vtkRegularPolygonSource> > nodesCircle;
    vtkSmartPointer<vtkPropAssembly> canvas;

};


template<class T>
DrawableGraph<T>::DrawableGraph()
{
    frameLength = 0;
    frameWidth = 0;
    nodesRadius = 1; //Probabilmente da impostare in modo dinamico a seconda dei label
    canvas = vtkSmartPointer<vtkPropAssembly>::New();
}

template<typename T>
DrawableGraph<T>::~DrawableGraph()
{
}

template<class T>
DrawableGraph<T>::DrawableGraph(Graph<T> &graph): Graph<T> (&graph)
{
    canvas = vtkSmartPointer<vtkPropAssembly>::New();
}

template<class T>
DrawableGraph<T>::DrawableGraph(Graph<T> *graph): Graph<T> (graph)
{
    canvas = vtkSmartPointer<vtkPropAssembly>::New();
}

template<typename T>
DrawableGraph<T>::DrawableGraph(DrawableGraph<T> &graph) : Graph<T> (&graph)
{
    update();
    canvas = vtkSmartPointer<vtkPropAssembly>::New();
}

template<typename T>
DrawableGraph<T>::DrawableGraph(DrawableGraph<T> *graph) : Graph<T> (graph)
{
    update();
    canvas = vtkSmartPointer<vtkPropAssembly>::New();
}

template<typename T>
double DrawableGraph<T>::computeRepulsiveForce(double d)
{
    return -pow(optimalDistance, 2) / d;
}

template<typename T>
double DrawableGraph<T>::computeAttractiveForce(double d)
{
    return pow(d, 2) / optimalDistance;
}

template<typename T>
double DrawableGraph<T>::cool(double temperature, unsigned int iteration)
{
    return temperature / (iteration + 1);
}

template<typename T>
void DrawableGraph<T>::computeVertexRepulsionPlacement()
{
    std::cout << nodesPositions << std::endl << std::endl;
    double temperature = frameWidth / 10;
    nodesDisplacements.resize(this->nodes.size(), 2);

    for(unsigned int i = 0; i < ITERATIONS_MAX; i++)
    {
        for(unsigned int j = 0; j < this->nodes.size(); j++)
        {
            Eigen::Vector2d zero = {0,0};
            nodesDisplacements.row(j) = zero;
            for(unsigned int l = 0; l < this->nodes.size(); l++)
            {
                if(j != l)
                {
                    Eigen::Vector2d distVec = nodesPositions.row(j) - nodesPositions.row(l);
                    double dist = distVec.norm();
                    nodesDisplacements.row(j) += (distVec/dist) * computeRepulsiveForce(dist);
                }
            }
        }

        for(unsigned int j = 0; j < this->arcs.size(); j++)
        {
            Arc<T>* a = this->arcs[j];
            unsigned int l1 = std::find(this->nodes.begin(), this->nodes.end(), a->getN1()) - this->nodes.begin();
            unsigned int l2 = std::find(this->nodes.begin(), this->nodes.end(), a->getN2()) - this->nodes.begin();
            Eigen::Vector2d distVec = nodesPositions.row(l1) - nodesPositions.row(l2);
            double dist = distVec.norm();
            nodesDisplacements.row(l1) -= (distVec/dist) * computeAttractiveForce(dist);
            nodesDisplacements.row(l2) += (distVec/dist) * computeAttractiveForce(dist);
        }

        for(unsigned int j = 0; j < this->nodes.size(); j++)
        {
            double displacementNorm = nodesDisplacements.row(j).norm();
            nodesPositions.row(j) += (nodesDisplacements.row(j) / displacementNorm) * std::min(displacementNorm, temperature);
            nodesPositions(j, 0) = std::min(frameWidth / 2.0, std::max(-frameWidth / 2.0, nodesPositions(j, 0)));
            nodesPositions(j, 1) = std::min(frameLength / 2.0, std::max(-frameLength / 2.0, nodesPositions(j, 1)));
        }

        //temperature = cool(temperature, i);
    }

    std::cout << nodesPositions << std::endl << std::endl;
}

template<typename T>
std::vector<std::vector<unsigned int> > DrawableGraph<T>::computeOrderedAdjacencies()
{
    std::vector<std::vector<unsigned int> > orderedAdjacencies;
    for(unsigned int i = 0; i < this->nodes.size(); i++)
    {
        Node<T>* n = this->nodes[i];
        std::vector<Arc<T> *> adjacencyEdges = n->getArcs();
        std::vector<unsigned int> orderedNeighbours;
        std::vector<double> angles;
        Arc<T>* reference = adjacencyEdges[0];
        unsigned int referenceIndex;
        if(reference->getN1() == n)
            referenceIndex = std::find(this->nodes.begin(), this->nodes.end(), reference->getN2()) - this->nodes.begin();
        else
            referenceIndex = std::find(this->nodes.begin(), this->nodes.end(), reference->getN1()) - this->nodes.begin();
        Eigen::Vector2d referencePos = nodesPositions.row(referenceIndex);
        Eigen::Vector2d originPos = this->nodesPositions.row(i);
        Eigen::Vector2d referenceVector = referencePos - originPos;
        referenceVector /= referenceVector.norm();

        for(unsigned int j = 1; j < adjacencyEdges.size(); j++)
        {
            unsigned int neighbourIndex;
            if(adjacencyEdges[j]->getN1() == n)
                neighbourIndex = std::find(this->nodes.begin(), this->nodes.end(), adjacencyEdges[j]->getN2()) - this->nodes.begin();
            else
                neighbourIndex = std::find(this->nodes.begin(), this->nodes.end(), adjacencyEdges[j]->getN1()) - this->nodes.begin();
            Eigen::Vector2d neighbourPos = nodesPositions.row(neighbourIndex);
            Eigen::Vector2d neighbourVector = neighbourPos - originPos;
            neighbourVector /= neighbourVector.norm();
            double angle = acos(referenceVector.dot(neighbourVector));
            if(referenceVector(0) * neighbourVector(1) - referenceVector(1) * neighbourVector(0) < 0)
                angle = M_PI + M_PI - angle;
            orderedNeighbours.push_back(neighbourIndex);
            angles.push_back(angle);

        }

        for(unsigned int j = 0; j < angles.size(); j++)
        {
            for(unsigned int k = j + 1; k < angles.size(); k++)
            {
                if(angles[j] > angles[k])
                {
                    unsigned int tmp1 = orderedNeighbours[j];
                    double tmp2 = angles[j];
                    orderedNeighbours[j] = orderedNeighbours[k];
                    angles[j] = angles[k];
                    orderedNeighbours[k] = tmp1;
                    angles[k] = tmp2;
                }
            }
        }

        orderedNeighbours.insert(orderedNeighbours.begin(),referenceIndex);
        orderedAdjacencies.push_back(orderedNeighbours);
    }
    return orderedAdjacencies;
}

template<typename T>
double DrawableGraph<T>::computeAngle(Eigen::Vector2d u, Eigen::Vector2d v)
{
    Eigen::Vector3d u3 = {u(0), u(1), 0};
    Eigen::Vector3d v3 = {v(0), v(1), 0};
     return atan2(u3.cross(v3).norm(), u3.dot(v3));
}

template<typename T>
void DrawableGraph<T>::computeEdgeRepulsionPlacement()
{
    std::vector<std::vector<unsigned int> > orderedAdjacencies = computeOrderedAdjacencies();
    double C1 = 3.0, C2 = 1.0, C3 = 1.0, C4 = 1.0, C5 = 1.0, C6 = 0.01;
    Eigen::Matrix2d R;
    R <<  0, 1,
         -1, 0;

    Eigen::MatrixX2d oldPos;
    double error = DBL_MAX;
    while(error > 0.1)
    {
        oldPos = nodesPositions;
        Eigen::MatrixX2d tmpForces = Eigen::MatrixX2d::Zero(this->nodes.size(), 2);
        for(unsigned int i = 0; i < this->nodes.size(); i++)
        {
            if(orderedAdjacencies[i].size() > 1)
            {
                for(unsigned int j = 1; j < orderedAdjacencies[i].size(); j++)
                {
                    Eigen::Vector2d v = nodesPositions.row(i);
                    Eigen::Vector2d v1 = nodesPositions.row(orderedAdjacencies[i][j - 1]);
                    Eigen::Vector2d v2 = nodesPositions.row(orderedAdjacencies[i][j]);
                    Eigen::Vector2d u1 = v1 - v;
                    Eigen::Vector2d u2 = v2 - v;
                    double ab = u1.norm();
                    double ac = u2.norm();
                    u1 /= ab;
                    u2 /= ac;
                    Eigen::Vector2d u3 = u1 + u2;
                    u3 /= u3.norm();
                    Eigen::Vector2d uf1 = R * u3;
                    uf1 /= uf1.norm();
                    double theta = computeAngle(u3,u1) * 2;    //I know, there isn't much sense in multiplying by 2 and then taking an half, just following the paper
                    double f = C3 * (atan(ab / C4) + atan(ac / C4)) + C5 * 1 / tan(theta / 2);
                    Eigen::Vector2d f1 = uf1 * f;
                    if(u3(0) * u1(1) - u1(0) * u3(1) <= 0)
                    {
                        tmpForces.row(orderedAdjacencies[i][j - 1]) += f1;
                        tmpForces.row(orderedAdjacencies[i][j]) -= f1;
                    } else
                    {
                        tmpForces.row(orderedAdjacencies[i][j - 1]) -= f1;
                        tmpForces.row(orderedAdjacencies[i][j]) += f1;
                    }
                    u1 *= ab;
                    u2 *= ac;
//                    std::cout << "u1=(" << u1(0) << "," << u1(1) << ")" << std::endl;
//                    std::cout << "u2=(" << u2(0) << "," << u2(1) << ")" << std::endl;
//                    std::cout << "u3=(" << u3(0) << "," << u3(1) << ")" << std::endl;
//                    std::cout << "uf1=(" << uf1(0) << "," << uf1(1) << ")" << std::endl;
//                    std::cout << "f1=(" << f1(0) << "," << f1(1) << ")" << std::endl;

                }
            }

        }
        for(unsigned int j = 0; j < this->arcs.size(); j++)
        {
            Arc<T>* a = this->arcs[j];
            unsigned int l1 = std::find(this->nodes.begin(), this->nodes.end(), a->getN1()) - this->nodes.begin();
            unsigned int l2 = std::find(this->nodes.begin(), this->nodes.end(), a->getN2()) - this->nodes.begin();
            Eigen::Vector2d fa = nodesPositions.row(l2) - nodesPositions.row(l1);
            double d = fa.norm();
            fa /= d;
            fa *= C1 * log(d/C2);
//            std::cout << "A=(" << nodesPositions.row(l1)(0) << "," << nodesPositions.row(l1)(1) << ")" << std::endl;
//            std::cout << "B=(" << nodesPositions.row(l2)(0) << "," << nodesPositions.row(l2)(1) << ")" << std::endl;
//            std::cout << "fa=(" << fa(0) << "," << fa(1) << ")" << std::endl;
            tmpForces.row(l1) += fa;
            tmpForces.row(l2) -= fa;
        }
        nodesPositions = oldPos + C6 * tmpForces;
        error = (oldPos - nodesPositions).norm();

        std::cout << nodesPositions << std::endl << std::endl;
    }
}

template<typename T>
void DrawableGraph<T>::update()
{
    srand(time(nullptr));

    Eigen::MatrixXd degrees = Eigen::MatrixXd::Zero(this->nodes.size(), this->nodes.size());
    Eigen::MatrixXd importances = Eigen::MatrixXd::Zero(this->nodes.size(), this->nodes.size());

    nodesPositions.resize(this->nodes.size(), 2);
    optimalDistance = sqrt(frameWidth * frameLength / static_cast<double>(this->nodes.size()));

    for(unsigned int i = 0; i < nodesPositions.rows(); i++)
    {
        nodesPositions(i, 0) = frameWidth * static_cast<double>(rand()) / RAND_MAX - frameWidth / 2;
        nodesPositions(i, 1) = frameLength * static_cast<double>(rand()) / RAND_MAX - frameLength / 2;
    }

    for(unsigned int i = 0; i < this->nodes.size(); i++)
        for(unsigned int j = 0; j < this->nodes.size(); j++)
        {
            std::vector<Arc<T>*> sharedEdges = this->getArcsFromEndpoints(this->nodes[i], this->nodes[j]);
            int degree = sharedEdges.size();
            double weightsSum = 0.0;
            for(unsigned int k = 0; k < sharedEdges.size(); k++)
                weightsSum += sharedEdges[k]->getWeight();
            if(degree > 0)
            {
                degrees(i,j) = degree;
                importances(i,j) = weightsSum;
            }
        }

    Eigen::VectorXd centralities = degrees.rowwise().sum();
    Eigen::VectorXd verticesInfluences = importances * centralities;
    std::cout << "Influences: " << verticesInfluences << std::endl << std::endl;
    computeVertexRepulsionPlacement();
    computeEdgeRepulsionPlacement();
    //Qui devo calcolare la posizione dei nodi
}

template<typename T>
void DrawableGraph<T>::updateView()
{
    nodesPoints = vtkSmartPointer<vtkPoints>::New();
    arcsLines = vtkSmartPointer<vtkCellArray>::New();
    nodesCircle.clear();
    nodeToID.clear();

    double lowest = DBL_MAX;
    for(unsigned int i = 0; i < this->nodes.size(); i++)
        for(unsigned int j = i + 1; j < this->nodes.size(); j++)
        {
            double dist = (this->nodesPositions.row(i) - this->nodesPositions.row(j)).norm();
            if(dist < lowest)
                lowest = dist;
        }

    for(unsigned int i = 0; i < this->nodes.size(); i++)
    {
        vtkSmartPointer<vtkRegularPolygonSource> polygonSource = vtkSmartPointer<vtkRegularPolygonSource>::New();
        vtkIdType id = nodesPoints->InsertNextPoint(nodesPositions(i,0), nodesPositions(i,1), 0);
        nodeToID.insert(std::make_pair(this->nodes[i], id));
        polygonSource->GeneratePolygonOff();
        polygonSource->SetNumberOfSides(CIRCLE_RESOLUTION);
        polygonSource->SetRadius(std::min(nodesRadius, lowest / 3));
        polygonSource->SetCenter(nodesPositions(i,0), nodesPositions(i,1), 0);
        nodesCircle.push_back(polygonSource);
        Node<T>* n = new Node<T>();

    }

    for(unsigned int i = 0; i < this->arcs.size(); i++)
    {
        vtkSmartPointer<vtkLine> edge = vtkSmartPointer<vtkLine>::New();
        edge->GetPointIds()->SetNumberOfIds(2);
        edge->GetPointIds()->SetId(0, nodeToID[this->arcs[i]->getN1()]);
        edge->GetPointIds()->SetId(1, nodeToID[this->arcs[i]->getN2()]);
        arcsLines->InsertNextCell(edge);
    }



}

template<class T>
void DrawableGraph<T>::draw(vtkSmartPointer<vtkPropAssembly> assembly)
{

    assembly->RemovePart(canvas);
    canvas = vtkSmartPointer<vtkPropAssembly>::New();
    vtkSmartPointer<vtkPolyData> data = vtkSmartPointer<vtkPolyData>::New();
    data->SetPoints(nodesPoints);
    data->SetLines(arcsLines);
    vtkSmartPointer<vtkPolyDataMapper> simpleGraphMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    simpleGraphMapper->SetInputData(data);
    vtkSmartPointer<vtkActor> simpleGraphActor = vtkSmartPointer<vtkActor>::New();
    simpleGraphActor->SetMapper(simpleGraphMapper);
    canvas->AddPart(simpleGraphActor);

    for(unsigned int i = 0; i < this->nodesCircle.size(); i++)
    {
        vtkSmartPointer<vtkPolyDataMapper> nodeCircleMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        nodeCircleMapper->SetInputConnection(this->nodesCircle[i]->GetOutputPort());
        vtkSmartPointer<vtkActor> nodeCircleActor = vtkSmartPointer<vtkActor>::New();
        nodeCircleActor->SetMapper(nodeCircleMapper);

        /*vtkSmartPointer<vtkTextActor3D> textActor = vtkSmartPointer<vtkTextActor3D>::New();
        textActor->SetInput(std::to_string(i).c_str());
        textActor->SetPosition(nodesPositions(i,0), nodesPositions(i,1), 0);
        vtkSmartPointer<vtkTextProperty> properties = textActor->GetTextProperty();
        properties->SetFontSize(10);
        properties->SetShadow(0);
        properties->SetColor(1.0, 1.0, 1.0);
        textActor->SetTextProperty(properties);*/
        vtkSmartPointer<vtkVectorText> textSource = vtkSmartPointer<vtkVectorText>::New();
        textSource->SetText(std::to_string(i).c_str());
        vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputConnection( textSource->GetOutputPort() );
        vtkSmartPointer<vtkFollower> follower = vtkSmartPointer<vtkFollower>::New();
        follower->SetMapper( mapper );
        follower->GetProperty()->SetColor( 1, 1, 1 ); // red
        follower->SetPosition(nodesPositions(i,0), nodesPositions(i,1), 0);


        canvas->AddPart(follower);
        canvas->AddPart(nodeCircleActor);
    }

    canvas->Modified();
    assembly->AddPart(canvas);
    assembly->Modified();

}


/***********Getters and setters**********/
template<class T>
bool DrawableGraph<T>::getUseMainLabel() const
{
    return useMainLabel;
}

template<class T>
void DrawableGraph<T>::setUseMainLabel(bool value)
{
    useMainLabel = value;
}

template<class T>
std::string DrawableGraph<T>::getMainLabel() const
{
    return mainLabel;
}

template<class T>
void DrawableGraph<T>::setMainLabel(const std::string &value)
{
    mainLabel = value;
}



template<class T>
double DrawableGraph<T>::getArcsWidth() const
{
    return arcsWidth;
}

template<class T>
void DrawableGraph<T>::setArcsWidth(double value)
{
    arcsWidth = value;
}

template<class T>
double DrawableGraph<T>::getNodesRadius() const
{
    return nodesRadius;
}

template<class T>
void DrawableGraph<T>::setNodesRadius(double value)
{
    nodesRadius = value;
}

template<class T>
vtkSmartPointer<vtkPropAssembly> DrawableGraph<T>::getCanvas() const
{
    return canvas;
}

template<class T>
void DrawableGraph<T>::setCanvas(const vtkSmartPointer<vtkPropAssembly> &value)
{
    canvas = value;
}

template<class T>
double DrawableGraph<T>::getFrameLength() const
{
return frameLength;
}

template<class T>
void DrawableGraph<T>::setFrameLength(double value)
{
    frameLength = value;
    nodesRadius = 0.02 * std::min(frameWidth, frameLength);
}

template<class T>
double DrawableGraph<T>::getFrameWidth() const
{
    return frameWidth;
}

template<class T>
void DrawableGraph<T>::setFrameWidth(double value)
{
    frameWidth = value;
    nodesRadius = 0.02 * std::min(frameWidth, frameLength);
}

template<class T>
Eigen::MatrixX2d DrawableGraph<T>::getNodesPositions() const
{
    return nodesPositions;
}

template<class T>
void DrawableGraph<T>::setNodesPositions(const Eigen::MatrixX2d &value)
{
    nodesPositions = value;
}


}
#endif // DRAWABLEGRAPH_H
