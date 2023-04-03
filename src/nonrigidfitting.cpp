#include "nonrigidfitting.h"

#include <limits.h>
#include <vector>
#include <map>
#include <queue>

//#include <fstream>
#include <surfaceannotation.h>
#include <pointannotation.h>


#include <ortools/linear_solver/linear_solver.h>
//#include <scip/scip.h>
//#include <scip/scipdefplugins.h>
//#include <scip_exception.hpp>

#include <Constraint.h>
#include <constraintsolver.h>
#include <utilities.h>

using namespace operations_research;

NonRigidFitting::NonRigidFitting()
{

}

NonRigidFitting::NonRigidFitting(ExtendedTrimesh *template_mesh, ExtendedTrimesh *mesh_to_fit)
{
    this->template_mesh = template_mesh;
    this->mesh_to_fit = mesh_to_fit;
}


bool comp(std::pair<Annotation*, Annotation*> a, std::pair<Annotation*, Annotation*> b)
{
    if(a.first->getHierarchyLevel() > b.first->getHierarchyLevel())
        return true;
    return false;
}

void normalizeMatrix(double** M, unsigned int rowNum, unsigned int colNum)
{
    double min = std::numeric_limits<double>::max();
    double max = -std::numeric_limits<double>::max();
    for(unsigned int i = 0; i < rowNum; i++)
        for(unsigned int j = 0; j < colNum; j++)
        {
            if(M[i][j] > max)
                max = M[i][j];
            if(M[i][j] < min)
                min = M[i][j];
        }


    for(unsigned int i = 0; i < rowNum; i++)
        for(unsigned int j = 0; j < colNum; j++)
            M[i][j] = (M[i][j] - min) / (max - min);
}

double distanceFromLandmark(std::vector<IMATI_STL::Vertex*> boundary, IMATI_STL::Vertex* v, IMATI_STL::Vertex* l)
{
//    std::cout << "New distance calculation" << std::endl;
//    std::cout << "(" << v->x << "," << v->y << "," << v->z << ")" << std::endl;
//    std::cout << "(" << l->x << "," << l->y << "," << l->z << ")" << std::endl;
//    for(unsigned int i = 0; i < boundary.size(); i++)
//        std::cout << "(" << boundary[i]->x << "," << boundary[i]->y << "," << boundary[i]->z << ")" << std::endl;
    double distance = 0;
    std::vector<IMATI_STL::Vertex*>::iterator vit = std::find(boundary.begin(), boundary.end(), v);
    std::vector<IMATI_STL::Vertex*>::iterator lit = std::find(boundary.begin(), boundary.end(), l);
    if(vit == boundary.end() || lit == boundary.end())
        return std::numeric_limits<double>::max();
    unsigned int v_pos = vit - boundary.begin();
    unsigned int l_pos = lit - boundary.begin();
//    std::cout << "(" << v_pos << " " << l_pos << ")" << std::endl;
    unsigned int current = Utilities::mod(v_pos + 1, boundary.size());
    do{
        unsigned int previous = Utilities::mod(current - 1, boundary.size());

//        std::cout << "(" << current << " " << previous << ")" << std::endl;
        distance += ((*boundary[previous]) - (*boundary[current])).length();
        current = Utilities::mod(current + 1, boundary.size());
    }while(current != Utilities::mod(l_pos + 1, boundary.size()));

//    std::cout << "Distance: " << distance << std::endl;
//    std::cin.ignore();
    return distance;

}

void NonRigidFitting::startFitting()
{
    auto start = std::chrono::high_resolution_clock::now();
    std::vector<Annotation*> template_annotations = template_mesh->getAnnotations();
    std::vector<Annotation*> to_fit_annotations = mesh_to_fit->getAnnotations();
    std::vector<std::pair<Annotation*, Annotation*> > coupled_annotations;
    std::vector<unsigned int> fixed_vertices;
    bool thereIsCorrespondence = false;
    unsigned int ALREADY_USED = 9;

    unsigned int actual_vertex_id = 0;
    for(unsigned int i = 0; i < to_fit_annotations.size(); i++)
        for(unsigned int j = 0; j < template_annotations.size(); j++)
            if(template_annotations[j]->getTag().compare(to_fit_annotations[i]->getTag()) == 0 &&
               template_annotations[j]->getType() == to_fit_annotations[i]->getType() &&
               template_annotations[j]->getType() == AnnotationType::Surface)
            {
                thereIsCorrespondence = true;
                coupled_annotations.push_back(std::make_pair(template_annotations[j], to_fit_annotations[i]));
                break;
            }

    std::sort(coupled_annotations.begin(), coupled_annotations.end(), comp);
    unsigned int coupled_annotations_counter = 0;
    for(std::vector<std::pair<Annotation*, Annotation*> >::iterator it = coupled_annotations.begin(); it != coupled_annotations.end(); it++)
    {
        coupled_annotations_counter++;
        ShapeOp::Matrix3X SOPoints = so_template_points->transpose();

        ConstraintSolver solver;
        std::vector<std::shared_ptr<ShapeOp::Constraint> > constraints;
        std::vector<IMATI_STL::Vertex*> a1Points = it->first->getInvolvedVertices();
        std::vector<IMATI_STL::Vertex*> a2Points = it->second->getInvolvedVertices();
        GraphTemplate::Node<Annotation*>* a1Node = template_mesh->getGraph()->getNodeFromData(it->first);
        GraphTemplate::Node<Annotation*>* a2Node = mesh_to_fit->getGraph()->getNodeFromData(it->second);
        std::vector<GraphTemplate::Arc<Annotation*>* > a1Contained = template_mesh->getGraph()->getArcsFromFletching(a1Node, "Containment");
        std::vector<GraphTemplate::Arc<Annotation*>* > a2Contained = mesh_to_fit->getGraph()->getArcsFromFletching(a2Node, "Containment");
        std::vector<IMATI_STL::Vertex*> a1Landmarks;
        std::vector<IMATI_STL::Vertex*> a2Landmarks;
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(a1Points.size(), a2Points.size());
        Eigen::MatrixXd B = Eigen::MatrixXd::Zero(a1Points.size(), a2Points.size());
        Eigen::MatrixXd C = Eigen::MatrixXd::Zero(a1Points.size(), a2Points.size());
        Eigen::MatrixXd D = Eigen::MatrixXd::Zero(a1Points.size(), a2Points.size());
        for(unsigned int i = 0; i < a1Contained.size(); i++)
        {
            Annotation* a = a1Contained[i]->getN2()->getData();
            if(a->getType() == AnnotationType::Point )
            {
                std::vector<IMATI_STL::Vertex*> involvedVertices = dynamic_cast<PointAnnotation*>(a)->getPoints();
                if(involvedVertices.size() == 1)
                    a1Landmarks.push_back(involvedVertices[0]);
            }
        }
        for(unsigned int i = 0; i < a2Contained.size(); i++)
        {
            Annotation* a = a2Contained[i]->getN2()->getData();
            if(a->getType() == AnnotationType::Point )
            {
                std::vector<IMATI_STL::Vertex*> involvedVertices = dynamic_cast<PointAnnotation*>(a)->getPoints();
                if(involvedVertices.size() == 1)
                    a2Landmarks.push_back(involvedVertices[0]);
            }
        }

        double** coefficients = static_cast<double**>(malloc(sizeof (double*) * a1Points.size()));


        MPSolver mp_solver("mapping", MPSolver::SCIP_MIXED_INTEGER_PROGRAMMING);
        MPVariable*** variables_matrix = static_cast<MPVariable***>(malloc(sizeof(MPVariable**) * a1Points.size()));

        Eigen::MatrixX3d a1Laplacians, a2Laplacians;
        a1Laplacians.resize(a1Points.size(), 3);
        a2Laplacians.resize(a2Points.size(), 3);
        for(unsigned int i = 0; i < a1Points.size(); i++)
        {
            IMATI_STL::Point l = template_mesh->computeLaplacian(template_mesh->getPointId(a1Points[i]), ExtendedTrimesh::WeightType::UNIFORM);
            a1Laplacians(i, 0) = l.x;
            a1Laplacians(i, 1) = l.y;
            a1Laplacians(i, 2) = l.z;
        }

        for(unsigned int i = 0; i < a2Points.size(); i++)
        {
            IMATI_STL::Point l = mesh_to_fit->computeLaplacian(mesh_to_fit->getPointId(a2Points[i]), ExtendedTrimesh::WeightType::UNIFORM);
            a2Laplacians(i, 0) = l.x;
            a2Laplacians(i, 1) = l.y;
            a2Laplacians(i, 2) = l.z;
        }

        std::ofstream laplacian1File, laplacian2File;
        laplacian1File.open("coupled"+std::to_string(coupled_annotations_counter)+"annotation1laplacian");
        laplacian2File.open("coupled"+std::to_string(coupled_annotations_counter)+"annotation2laplacian");
        if(laplacian1File.is_open())
        {
            laplacian1File << a1Laplacians << std::endl;
            laplacian1File.close();
        }

        if(laplacian2File.is_open())
        {
            laplacian2File << a2Laplacians << std::endl;
            laplacian2File.close();
        }

        for(unsigned int i = 0; i < a1Points.size(); i++)
        {
            MPVariable** variables_row = static_cast<MPVariable**>(malloc(sizeof(MPVariable*) * a2Points.size()));
            coefficients[i] = static_cast<double*>(malloc(sizeof(double) * a2Points.size()));
            for(unsigned int j = 0; j < a2Points.size(); j++)
            {
                A(i,j) = (a1Laplacians.row(i) - a2Laplacians.row(j)).norm();
                C(i,j) = ((*a1Points[i]) - (*a2Points[j])).length();
                MPVariable* const x = mp_solver.MakeIntVar(0.0, 1.0, "x" + std::to_string(i) + "," + std::to_string(j));
                variables_row[j] = x;
            }
            variables_matrix[i] = variables_row;

        }

        for(unsigned int i = 0; i < a1Points.size(); i++)
        {
            SurfaceAnnotation* sa1 = dynamic_cast<SurfaceAnnotation*>(it->first);
            unsigned int boundaryIndex1;
            bool a1PointOnBorder = sa1->isPointOnBorder(a1Points[i], boundaryIndex1);
            if(!a1PointOnBorder && (a1Points[i]->info == nullptr || *static_cast<unsigned int*>(a1Points[i]->info) == ALREADY_USED))
            {
                fixed_vertices.push_back(template_mesh->getPointId(a1Points[i]));
                a1Points[i]->info = &ALREADY_USED;
            }
            for(unsigned int j = 0; j < a2Points.size(); j++)
            {
                SurfaceAnnotation* sa2 = dynamic_cast<SurfaceAnnotation*>(it->second);
                unsigned int boundaryIndex2;
                bool a2PointOnBorder = sa2->isPointOnBorder(a2Points[j], boundaryIndex2);
                B(i,j) = std::abs(a1PointOnBorder - a2PointOnBorder);
                if(a1PointOnBorder && a2PointOnBorder && a1Landmarks.size() == a2Landmarks.size() && a1Landmarks.size() > 0)
                {
                    for(unsigned int k = 0; k < a1Landmarks.size(); k++)
                    {
                        double dist1 = distanceFromLandmark(sa1->getOutlines()[boundaryIndex1], a1Points[i], a1Landmarks[k]);
                        double dist2 = distanceFromLandmark(sa2->getOutlines()[boundaryIndex2], a2Points[j], a2Landmarks[k]);
                        D(i,j) += std::abs(dist1 - dist2);
                    }
                }

            }
        }
        A.normalize();
        B.normalize();
        C.normalize();
        D.normalize();
        for(unsigned int i = 0; i < a1Points.size(); i++)
            for(unsigned int j = 0; j < a2Points.size(); j++)
                coefficients[i][j] = A(i,j) + 3 * B(i,j) + 2 * C(i,j) + 3 * D(i,j);

        for(unsigned int i = 0; i < a1Points.size(); i++)
        {
            MPConstraint* const constraint0 = mp_solver.MakeRowConstraint(1.0, 1.0);
            for(unsigned int j = 0; j < a2Points.size(); j++)
            {
                constraint0->SetCoefficient(variables_matrix[i][j], 1);
            }
        }

        LOG(INFO) << "Number of constraints = " << mp_solver.NumConstraints();

        MPObjective* const objective = mp_solver.MutableObjective();

        for(unsigned int i = 0; i < a1Points.size(); i++)
          for(unsigned int j = 0; j < a2Points.size(); j++)
              objective->SetCoefficient(variables_matrix[i][j], coefficients[i][j]);

        objective->SetMinimization();

        const MPSolver::ResultStatus result_status = mp_solver.Solve();
        // Check that the problem has an optimal solution.
        if (result_status != MPSolver::OPTIMAL)
          LOG(FATAL) << "The problem does not have an optimal solution!";

//        LOG(INFO) << "Solution:";
//        LOG(INFO) << "Optimal objective value = " << objective->Value();
//        std::ofstream resultFile, input1File, input2File;
//        resultFile.open("result"+std::to_string(counter)+".m");
//        input1File.open("first_points_set"+std::to_string(counter)+".m");
//        input2File.open("second_points_set"+std::to_string(counter++)+".m");
//        if(resultFile.is_open() && input1File.is_open() && input2File.is_open())
//        {
//            for(unsigned int i = 0; i < a1Points.size(); i++)
//            {
//                input1File << a1Points[i]->x << " " << a1Points[i]->y << " " << a1Points[i]->z << std::endl;
//                for(unsigned int j = 0; j < a2Points.size(); j++)
//                    resultFile << variables_matrix[i][j]->solution_value() << " ";
//                resultFile << std::endl;
//            }
//            for(unsigned int j = 0; j < a2Points.size(); j++)
//                input2File << a2Points[j]->x << " " << a2Points[j]->y << " " << a2Points[j]->z << std::endl;
//            resultFile.close();
//            input1File.close();
//            input2File.close();
//        }

        std::map<IMATI_STL::Vertex*, IMATI_STL::Vertex*> a1Toa2Map;

        std::ofstream annotation1_points_file;
        std::ofstream annotation2_points_file;
        annotation1_points_file.open("coupled_" + std::to_string(coupled_annotations_counter) + "annotation1_points.mat");
        if(annotation1_points_file.is_open()){
            for(unsigned int i = 0; i < a1Points.size(); i++)
                annotation1_points_file << a1Points[i]->x << " " << a1Points[i]->y << " " << a1Points[i]->z << std::endl;
            annotation1_points_file.close();
        }
        annotation2_points_file.open("coupled_" + std::to_string(coupled_annotations_counter) + "annotation2_points.mat");
        if(annotation2_points_file.is_open()){
            for(unsigned int i = 0; i < a2Points.size(); i++)
                annotation2_points_file << a2Points[i]->x << " " << a2Points[i]->y << " " << a2Points[i]->z << std::endl;
            annotation2_points_file.close();
        }
        std::ofstream correspondences_file;
        correspondences_file.open("coupled_" + std::to_string(coupled_annotations_counter) + "correspondences.mat");
        for(unsigned int i = 0; i < a1Points.size(); i++)
            for(unsigned int j = 0; j < a2Points.size(); j++)
                if(variables_matrix[i][j]->solution_value() == 1)
                {
                    if(correspondences_file.is_open())
                        correspondences_file << i << " " << j << std::endl;

                    a1Toa2Map.insert(std::make_pair(a1Points[i], a2Points[j]));
                }

        correspondences_file.close();



        mp_solver.Clear();

        for(unsigned int i = 0; i < a1Points.size(); i++)
        {
            delete[] coefficients[i];
            delete variables_matrix[i];
        }
        delete coefficients;
        delete variables_matrix;

        for(unsigned int i = 0; i < a1Points.size(); i++)
        {
            IMATI_STL::Vertex* v = a1Points[i];
            std::vector<int> id0 = { static_cast<int>(template_mesh->getPointId(v)) };
            auto c1 = std::make_shared<ShapeOp::ClosenessConstraint>(id0, 1000.0, SOPoints);
            IMATI_STL::Vertex* a2v = a1Toa2Map[v];
            ShapeOp::Vector3 positionOnA2 = {a2v->x, a2v->y, a2v->z};
            c1->setPosition(positionOnA2);
            constraints.push_back(c1);
            solver.addConstraint(c1);

        }

        if(thereIsCorrespondence)
        {
            for(IMATI_STL::Node* n = template_mesh->V.head(); n != nullptr; n = n->next())
            {
                IMATI_STL::Vertex* v = static_cast<IMATI_STL::Vertex*>(n->data);
                std::vector<int> id0 = { static_cast<int>(template_mesh->getPointId(v)) };
                auto c1 = std::make_shared<ShapeOp::ClosenessConstraint>(id0, 1.0, SOPoints);
                constraints.push_back(c1);
                solver.addConstraint(c1);

                if(v->isOnBoundary())
                    continue;
                std::vector<int> vvIds;
                vvIds.push_back(template_mesh->getPointId(v));
                for(IMATI_STL::Node* n1 = v->VV()->head(); n1 != nullptr; n1 = n1->next())
                {
                    IMATI_STL::Vertex* v_ = static_cast<IMATI_STL::Vertex*>(n1->data);
                    vvIds.push_back(template_mesh->getPointId(v_));
                }
                std::vector<unsigned int>::iterator it = std::find(fixed_vertices.begin(), fixed_vertices.end(), id0[0]);
                if(it == fixed_vertices.end())
                {
//                    auto c2 = ShapeOp::Constraint::shapeConstraintFactory("CotangentLaplacianDisplacement", vvIds, 1000.0, SOPoints);
//                    solver.addConstraint(c2);
//                    constraints.push_back(c2);
                } else {
//                    auto c2 = ShapeOp::Constraint::shapeConstraintFactory("CotangentLaplacianDisplacement", vvIds, 1000.0, SOPoints);
//                    solver.addConstraint(c2);
//                    constraints.push_back(c2);
                }
            }
            solver.setModelPointsTransposed(SOPoints);
            solver.setBarycentricCoordinates(coordinates->getCoordinates());
            solver.setCagePoints(so_cage_points);
            bool success = solver.initialize();

            success ? std::cout << "Initialized!" : std::cout << "Error in  initialization."<< std::endl;
            std::cout << std::endl;

            if(success)
            {
                solver.solve(100);
                for (unsigned int i = 0; i < so_cage_points->rows(); i++)
                    this->cage_mesh->setPoint(i, (*so_cage_points)(i, 0), (*so_cage_points)(i, 1), (*so_cage_points)(i, 2));

                coordinates->deform();
                context->updateMeshView();
                auto end = std::chrono::high_resolution_clock::now();
                double duration = std::chrono::duration_cast<std::chrono::seconds>(end - start).count();
                std::cout << "The fitting took " << duration << " seconds" << std::endl << std::flush;
            }

        }
    }



}

void NonRigidFitting::startFitting1()
{
    //auto start = std::chrono::high_resolution_clock::now();
//    std::vector<Annotation*> template_annotations = template_mesh->getAnnotations();
//    std::vector<Annotation*> to_fit_annotations = mesh_to_fit->getAnnotations();
//    std::vector<std::pair<Annotation*, Annotation*> > coupled_annotations;

//    unsigned int actual_vertex_id = 0;
//    for(unsigned int i = 0; i < to_fit_annotations.size(); i++)
//        for(unsigned int j = 0; j < template_annotations.size(); j++)
//            if(template_annotations[j]->getTag().compare(to_fit_annotations[i]->getTag()) == 0 &&
//               template_annotations[j]->getType() == to_fit_annotations[i]->getType() &&
//               template_annotations[j]->getType() == AnnotationType::Surface)
//            {
//                coupled_annotations.push_back(std::make_pair(template_annotations[j], to_fit_annotations[i]));
//                break;
//            }

//    std::sort(coupled_annotations.begin(), coupled_annotations.end(), comp);
//    std::sort(coupled_annotations.begin(), coupled_annotations.end(), comp);
//    for(std::vector<std::pair<Annotation*, Annotation*> >::iterator it = coupled_annotations.begin(); it != coupled_annotations.end(); it++)
//    {
//        std::vector<IMATI_STL::Vertex*> a1Points = it->first->getInvolvedVertices();
//        std::vector<IMATI_STL::Vertex*> a2Points = it->second->getInvolvedVertices();
//        GraphTemplate::Node<Annotation*>* a1Node = template_mesh->getGraph()->getNodeFromData(it->first);
//        GraphTemplate::Node<Annotation*>* a2Node = mesh_to_fit->getGraph()->getNodeFromData(it->second);
//        std::vector<GraphTemplate::Arc<Annotation*>* > a1Contained = template_mesh->getGraph()->getArcsFromFletching(a1Node, "Containment");
//        std::vector<GraphTemplate::Arc<Annotation*>* > a2Contained = mesh_to_fit->getGraph()->getArcsFromFletching(a2Node, "Containment");
//        std::vector<IMATI_STL::Vertex*> a1Landmarks;
//        std::vector<IMATI_STL::Vertex*> a2Landmarks;
//        for(unsigned int i = 0; i < a1Contained.size(); i++)
//        {
//            Annotation* a = a1Contained[i]->getN2()->getData();
//            if(a->getType() == AnnotationType::Point )
//            {
//                std::vector<IMATI_STL::Vertex*> involvedVertices = dynamic_cast<PointAnnotation*>(a)->getPoints();
//                if(involvedVertices.size() == 1)
//                    a1Landmarks.push_back(involvedVertices[0]);
//            }
//        }
//        for(unsigned int i = 0; i < a2Contained.size(); i++)
//        {
//            Annotation* a = a2Contained[i]->getN2()->getData();
//            if(a->getType() == AnnotationType::Point )
//            {
//                std::vector<IMATI_STL::Vertex*> involvedVertices = dynamic_cast<PointAnnotation*>(a)->getPoints();
//                if(involvedVertices.size() == 1)
//                    a2Landmarks.push_back(involvedVertices[0]);
//            }
//        }

//        SCIP* scip;
//        SCIP_CALL_EXC(SCIPcreate(& scip));
//        SCIP_CALL_EXC(SCIPincludeDefaultPlugins(scip));
        // SCIPmessagehdlrSetQuiet(SCIPgetMessagehdlr(scip), TRUE);
//        SCIP_CALL_EXC(SCIPcreateProb(scip, "CorrespondencesMapping", nullptr, nullptr,
//                                     nullptr, nullptr, nullptr, nullptr, nullptr));
//        SCIP_VAR*** variables_matrix = static_cast<SCIP_VAR***>(malloc(sizeof(SCIP_VAR**) * a1Points.size()));
//        for(unsigned int i = 0; i < a1Points.size(); i++)
//        {
//            SCIP_VAR** variables_row = static_cast<SCIP_VAR**>(malloc(sizeof(SCIP_VAR*) * a2Points.size()));
//            for(unsigned int j = 0; j < a2Points.size(); j++)
//            {
//                SCIP_VAR* var;
//                std::string varname = "x#" + std::to_string(i) + "#" + std::to_string(j);
//                SCIP_CALL_EXC(SCIPcreateVar(scip, &var, varname.c_str(), 0.0, 1.0, 1.0,
//                                            SCIP_VARTYPE_BINARY, TRUE, FALSE,
//                                            nullptr, nullptr, nullptr, nullptr, nullptr));
//                SCIP_CALL_EXC(SCIPaddVar(scip, var));
//                variables_row[j] = var;
//            }
//            variables_matrix[i] = variables_row;
//        }


//        SCIP_CONS** cons_list = static_cast<SCIP_CONS**>(malloc(sizeof(SCIP_CONS*) * a1Points.size()));
//        for( size_t i = 0; i < a1Points.size(); i++ )
//        {
//            SCIP_CONS * cons;

//            std::string consname = "row#" + std::to_string(i);
//            // create SCIP_CONS object
//            // this is an equality since there must be a queen in every row
//            SCIP_CALL_EXC( SCIPcreateConsLinear(scip, &cons, consname.c_str(), 0, NULL, NULL, 1.0, 1.0,
//                          TRUE, TRUE, TRUE, TRUE, TRUE, FALSE, FALSE, FALSE, FALSE, FALSE) );

//            // add the vars belonging to field in this row to the constraint
//            for( size_t j = 0; j < a1Points.size(); ++j )
//            SCIP_CALL_EXC( SCIPaddCoefLinear(scip, cons, variables_matrix[i][j], 1.0) );

//            // add the constraint to scip
//            SCIP_CALL_EXC( SCIPaddCons(scip, cons) );

//            // store the constraint for later on
//            cons_list[i] = cons;
//        }

//        SCIP_SOL* sol = SCIPgetBestSol(scip);
//        std::map<IMATI_STL::Vertex*, IMATI_STL::Vertex*> a1Toa2Map;
//        if(sol != NULL)
//        {

//            for(unsigned int i = 0; i < a1Points.size(); i++)
//                for(unsigned int j = 0; j < a2Points.size(); j++)
//                    if(SCIPgetSolVal(scip, sol, variables_matrix[i][j]) > 0.5)
//                        a1Toa2Map.insert(std::make_pair(a1Points[i], a2Points[j]));
//        }



//    }
}

ExtendedTrimesh *NonRigidFitting::getTemplateMesh() const
{
    return template_mesh;
}

void NonRigidFitting::setTemplateMesh(ExtendedTrimesh *value)
{
    template_mesh = value;
}

ExtendedTrimesh *NonRigidFitting::getMeshToFit() const
{
    return mesh_to_fit;
}

void NonRigidFitting::setMeshToFit(ExtendedTrimesh *value)
{
    mesh_to_fit = value;
}

ExtendedTrimesh *NonRigidFitting::getCageMesh() const
{
    return cage_mesh;
}

void NonRigidFitting::setCageMesh(ExtendedTrimesh *newCage_mesh)
{
    cage_mesh = newCage_mesh;
}

ShapeOpBarycentricCoordinates *NonRigidFitting::getCoordinates() const
{
    return coordinates;
}

void NonRigidFitting::setCoordinates(ShapeOpBarycentricCoordinates *newCoordinates)
{
    coordinates = newCoordinates;
}

ShapeOp::MatrixX3 *NonRigidFitting::getSoTemplatePoints() const
{
    return so_template_points;
}

void NonRigidFitting::setSoTemplatePoints(ShapeOp::MatrixX3 *newSo_template_points)
{
    so_template_points = newSo_template_points;
}

ShapeOp::MatrixX3 *NonRigidFitting::getSoCagePoints() const
{
    return so_cage_points;
}

void NonRigidFitting::setSoCagePoints(ShapeOp::MatrixX3 *newSo_cage_points)
{
    so_cage_points = newSo_cage_points;
}

MainWindow *NonRigidFitting::getContext() const
{
    return context;
}

void NonRigidFitting::setContext(MainWindow *newContext)
{
    context = newContext;
}
