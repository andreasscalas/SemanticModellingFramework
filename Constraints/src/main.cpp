#include <solver.h>
#include <closenessconstraint.h>
#include <edgestrainconstraint.h>
#include <lineconstraint.h>
#include <orientationconstraint.h>
#include <planeconstraint.h>
#include <polylinelengthconstraint.h>
#include <uniformlaplacianconstraint.h>
#include <iostream>

void print_points(const ShapeOp::MatrixX3 &p)
{
    for(int i = 0; i < p.rows(); ++i){
        std::cout << "Point " << i << " : ( ";
        ShapeOp::Vector3 current_pt = p.row(i);
        std::cout << current_pt.transpose();
        std::cout << " )" << std::endl;
    }
}

int main() {
    std::shared_ptr<ShapeOp::Matrix3X> p = std::make_shared<ShapeOp::Matrix3X>();
    p->resize(3,4);
    // The << operator reads elements row by row
    (*p) << 0.3, -0.1, 0.0, 1.0,
            0.2, 0.3, 1.0, 0.0,
            -0.5, 1.0, 0.0, -0.1;

    std::shared_ptr<ShapeOp::MatrixXX> b = std::make_shared<ShapeOp::MatrixXX>();
    b->resize(4,4);
    (*b) << 1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0;

    std::shared_ptr<ShapeOp::MatrixX3> mpt = std::make_shared<ShapeOp::MatrixX3>(p->transpose());
    std::cout <<  "Input points:" << std::endl;
    print_points(*mpt);

    ShapeOp::Solver s;
    s.setModelPoints(mpt);
    s.setCagePoints(mpt);
    s.setBarycentricCoordinates(b);
    double weight = 1.0;
    //add a plane constraint to all the vertices.
    {
        std::vector<int> id_vector;
        id_vector.push_back(0);
        auto c = std::make_shared<ShapeOp::ClosenessConstraint>(id_vector, weight, p);
        s.addConstraint(c);
    }
    {
        std::vector<int> id_vector;
        id_vector.push_back(1);
        auto c = std::make_shared<ShapeOp::ClosenessConstraint>(id_vector, weight, p);
        s.addConstraint(c);
    }
    {
        std::vector<int> id_vector;
        id_vector.push_back(2);
        auto c = std::make_shared<ShapeOp::ClosenessConstraint>(id_vector, weight, p);
        s.addConstraint(c);
    }
    {
        std::vector<int> id_vector;
        id_vector.push_back(3);
        auto c = std::make_shared<ShapeOp::ClosenessConstraint>(id_vector, weight, p);
        s.addConstraint(c);
    }
    {
    std::vector<int> id_vector;
    id_vector.push_back(0);
    id_vector.push_back(1);
    id_vector.push_back(2);
    id_vector.push_back(3);
    auto c = std::make_shared<ShapeOp::UniformLaplacianConstraint>(id_vector, 2 * weight, p, false);
    s.addConstraint(c);
    }
    if(s.initialize()){
        s.solve(10);
        mpt = s.getModelPoints();
        std::cout << "Output points:" << std::endl;
        print_points(*mpt);
    }else
        std::cout<<"Error"<<std::endl<<std::flush;


    return 0;
}
