#ifndef CONSTRAINT_H
#define CONSTRAINT_H
#include <vector>
#include <types.h>
#include <memory>

namespace ShapeOp{
    class Constraint {
    public:
        Constraint(const std::vector<int> &idI, double weight) : idList(idI), weight(weight) {}
        virtual ~Constraint() { idList.clear(); }

        virtual void project(std::shared_ptr<MatrixX3> projections) const = 0;
        virtual void addConstraint(std::vector<Triplet> &triplets, int &idO) const = 0;

        std::vector<int> getIdList() const{ return idList; }
        double getWeight() const{ return weight; }
        void setWeight(double value){ weight = value; }
        std::shared_ptr<Matrix3X> getModelPoints() const { return modelPoints; }
        void setModelPoints(const std::shared_ptr<Matrix3X> &value){ modelPoints = value; }

    protected:
        std::shared_ptr<Matrix3X> modelPoints;
        std::vector<int> idList;
        double weight;
        mutable int idO;
    };

}

#endif // CONSTRAINT_H
