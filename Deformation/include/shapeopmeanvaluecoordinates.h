#ifndef SHAPEOPMEANVALUECOORDINATES_H
#define SHAPEOPMEANVALUECOORDINATES_H

#include <shapeopbarycentriccoordinates.h>
#include <extendedtrimesh.h>


class ShapeOpMeanValueCoordinates : public ShapeOpBarycentricCoordinates
{
private:
    ExtendedTrimesh* mesh;                                 //The model to deform
    ExtendedTrimesh* cage;                                 //The cage with which the model is deformed
    ShapeOp::MatrixX3* meshVertices;
    ShapeOp::MatrixX3* cageVertices;
    ShapeOp::SparseMatrix* coords;                           //The coordinates linked to the vertices
    Eigen::SparseMatrix<bool,Eigen::RowMajor> maxInfluenceValues;
    std::map<long, long> verticesID;                    //Ids of the vertices of the cage
    std::string coordsFileName;                         //Name of the file in which the coordinates are saved
    static const unsigned int NUM_THREADS = 2;         //Number of threads which will be executed in parallel

    double threshold;

    void analyseMaxInfluenceThreshold();
public:

    /**
     * @brief ShapeOpMeanValueCoordinates Main constructor of the class. It requires the
     * model which will be deformed and the cage used for that purpose.
     * @param mesh the mesh which will be deformed
     * @param cage the cage used for the deformation
     * @param meshVertices The matrix containing the vertices of the mesh for the ShapeOp solver
     * @param cageVertices The matrix containing the vertices of the cage for the ShapeOp solver
     */
    ShapeOpMeanValueCoordinates(ExtendedTrimesh* mesh, ExtendedTrimesh* cage, ShapeOp::MatrixX3* meshVertices, ShapeOp::MatrixX3* cageVertices);

    virtual ~ShapeOpMeanValueCoordinates() override;

    /**
     * @brief ShapeOpMeanValueCoordinates::computeCoordinates Method that manages the computation of the coordinates
     */
    virtual void computeCoordinates() override;

    /**
     * @brief ShapeOpMeanValueCoordinates::saveCoordinates Method that manages the saving of the computed coordinates
     * @param filename the name of the file in which the coordinates will be saved
     */
    virtual void saveCoordinates(std::string filename) override;

    /**
     * @brief ShapeOpMeanValueCoordinates::loadCoordinates Method that manages the loading of the coordinates
     * @param filename the name of the file from which the coordinates will be loaded
     */
    virtual void loadCoordinates(std::string filename) override;

    /**
     * @brief ShapeOpMeanValueCoordinates::deform Method that manages the deformation of the model.
     */
    virtual void deform() override;
    virtual ShapeOp::SparseMatrix* getCoordinates() const override;

    double getThreshold() const;
    void setThreshold(double value);

    virtual std::vector<unsigned int> getMaxInfluenceCageVertices(unsigned int) override;
};

#endif // SHAPEOPMEANVALUECOORDINATES_H
