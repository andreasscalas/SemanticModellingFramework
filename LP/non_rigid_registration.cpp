#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <ortools/linear_solver/linear_solver.h>

namespace operations_research {

/**
 * @brief norm calculate the norm of a 3-dimensional vector
 * @param n the vector
 * @return the norm of the vector
 */
double compute_norm(double* n)
{
    return sqrt(pow(n[0],2)+pow(n[1],2)+pow(n[2],2));
}

double* subtract(double* v1, double* v2)
{
    double* result = new double(3);
    result[0]=v1[0]-v2[0];
    result[1]=v1[1]-v2[1];
    result[2]=v1[2]-v2[2];
    return result;
}

void IntegerProgrammingExample() {
  // [START solver]
  // Create the mip solver with the SCIP backend.
  MPSolver solver("integer_programming_example",
                  MPSolver::SCIP_MIXED_INTEGER_PROGRAMMING);
  // [END solver]

  // [START variables]
  // x, y, and z are non-negative integer variables.

  std::ifstream pointsFile1, pointsFile2;
  pointsFile1.open("flower1_transformed.m");
  pointsFile2.open("flower2_transformed.m");
  std::string line;
  std::vector<double*> points1, points2;
  if(pointsFile1.is_open())
  {
    while ( getline (pointsFile1,line) )
    {
        double * point = new double(3);
        std::stringstream ss(line);
        for(unsigned int i = 0; i < 3; i++)
            ss>>point[i];
        points1.push_back(point);
    }
    pointsFile1.close();
  } else
      std::cerr << "File punti numero 1 non trovato." << std::endl;

  if(pointsFile2.is_open())
  {
    while ( getline (pointsFile2,line) )
    {
        double * point = new double(3);
        std::stringstream ss(line);
        for(unsigned int i = 0; i < 3; i++)
            ss>>point[i];
        points2.push_back(point);
    }
    pointsFile1.close();
  } else
      std::cerr << "File punti numero 2 non trovato." << std::endl;

  std::cout << "Punti 1:" << std::endl;
  for(unsigned int i = 0; i < points1.size(); i++)
    std::cout << points1[i][0]<< " " << points1[i][1] << " " << points1[i][2] << std::endl;

  std::cout << "Punti 2:" << std::endl;
  for(unsigned int i = 0; i < points2.size(); i++)
    std::cout << points2[i][0]<< " " << points2[i][1] << " " << points2[i][2] << std::endl;

  unsigned int VAR1_SIZE = points1.size();
  unsigned int VAR2_SIZE = points2.size();

  MPVariable*** variables_matrix = static_cast<MPVariable***>(malloc(sizeof(MPVariable**) * VAR1_SIZE));
  for(unsigned int i = 0; i < VAR1_SIZE; i++)
  {
      MPVariable** variables_row = static_cast<MPVariable**>(malloc(sizeof(MPVariable*) * VAR2_SIZE));
      for(unsigned int j = 0; j < VAR2_SIZE; j++)
      {
        MPVariable* const x = solver.MakeIntVar(0.0, 1.0, "x"+std::to_string(i)+","+std::to_string(j));
        variables_row[j] = x;
      }
      variables_matrix[i]=variables_row;

  }


  for(unsigned int i = 0; i < VAR1_SIZE; i++)
  {
      MPConstraint* const constraint0 = solver.MakeRowConstraint(1.0, 1.0);
      for(unsigned int j = 0; j < VAR2_SIZE; j++)
      {
          constraint0->SetCoefficient(variables_matrix[i][j], 1);
      }
  }

  LOG(INFO) << "Number of constraints = " << solver.NumConstraints();

  MPObjective* const objective = solver.MutableObjective();

  for(unsigned int i = 0; i < VAR1_SIZE; i++)
    for(unsigned int j = 0; j < VAR2_SIZE; j++)
        objective->SetCoefficient(variables_matrix[i][j], compute_norm(subtract(points1[i], points2[j])));

  objective->SetMinimization();

  const MPSolver::ResultStatus result_status = solver.Solve();
  // Check that the problem has an optimal solution.
  if (result_status != MPSolver::OPTIMAL) {
    LOG(FATAL) << "The problem does not have an optimal solution!";
  }

  LOG(INFO) << "Solution:";
  LOG(INFO) << "Optimal objective value = " << objective->Value();
  std::ofstream resultFile;
  resultFile.open("result.m");
  if(resultFile.is_open())
  {
      for(unsigned int i = 0; i < VAR1_SIZE; i++)
      {
          for(unsigned int j = 0; j < VAR2_SIZE; j++)
          {
            resultFile << variables_matrix[i][j]->solution_value() << " ";
          }
          resultFile << std::endl;
      }
      resultFile.close();
  }


}
}

int main(int argc, char** argv) {
  operations_research::IntegerProgrammingExample();
  return EXIT_SUCCESS;
}
// [END program]

