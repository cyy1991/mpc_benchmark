#include"Eigen/Dense"
#include <vector>
#include <algorithm>
#include <memory>
#include<iostream>
#include "qp_solver/active_set_qp_solver.h"
#include "qp_solver/qp_solver.h"

using namespace std;
using namespace Eigen;
int main()
{

    int state = 2;

    MatrixXd A (state, state);
    A << 1, 0,
         0, 1;
    //A << 0, 0, 0,
    //     0, 0, 0,
    //    0, 0, 0;
    //A<< 2, 0, 1,
    //    0, 4, 0,
    //    1, 0, 6;
    MatrixXd B(state, 1);
    //B << 1, -2, 4;
    B << -3, -4;

    /*
    MatrixXd matrix_lower_1 (1, 1);
    matrix_lower_1 << -20;    // x1 minest, x2 minest, x3 max;

    MatrixXd matrix_lower_2 (1, 1);
    matrix_lower_2 << 2;

    MatrixXd matrix_lower_3 (1, 1);
    matrix_lower_3 << 0;

    MatrixXd matrix_lower_4 (1, 1);
    matrix_lower_4 << 1;

    MatrixXd matrix_lower_5 (1, 1);
    matrix_lower_5 << 0;

    ///upper
    MatrixXd matrix_upper_1 (1, 1);
    matrix_upper_1 << 10;

    MatrixXd matrix_upper_2 (1, 1);
    matrix_upper_2 << 20;   ////  x1 minest, x2 maxest, x3 maxest.

    MatrixXd matrix_upper_3 (1, 1);
    matrix_upper_3 << 5;

    MatrixXd matrix_upper_4 (1, 1);
    matrix_upper_4 << 5;

    MatrixXd matrix_upper_5 (1, 1);
    matrix_upper_5 << 5;

    MatrixXd matrix_inequality_constrain_1 (1, state);
    matrix_inequality_constrain_1 << 3, 4, -2;

    MatrixXd matrix_inequality_constrain_2 (1, state);
    matrix_inequality_constrain_2 << -3, 2, 1;

    MatrixXd matrix_inequality_constrain_3 (1, state);
    matrix_inequality_constrain_3 << 1, 0, 0;

    MatrixXd matrix_inequality_constrain_4 (1, state);
    matrix_inequality_constrain_4 << 0, 1, 0;

    MatrixXd matrix_inequality_constrain_5 (1, state);
    matrix_inequality_constrain_5 << 0, 0, 1;
    cout << "build pass in 68"<<endl;
    
    MatrixXd matrix_inequality_constrain = MatrixXd::Zero(matrix_inequality_constrain_1.rows() * 10, matrix_inequality_constrain_1.cols());
    /////////////end here
    matrix_inequality_constrain << matrix_inequality_constrain_1,  - matrix_inequality_constrain_1,
      matrix_inequality_constrain_2, - matrix_inequality_constrain_2, 
      matrix_inequality_constrain_3, - matrix_inequality_constrain_3,
      matrix_inequality_constrain_4, - matrix_inequality_constrain_4,
      matrix_inequality_constrain_5, - matrix_inequality_constrain_5;

    cout << "build pass in 78"<<endl;
    
    
    MatrixXd matrix_inequality_boundary = MatrixXd::Zero(matrix_lower_1.rows() * 10, matrix_lower_1.cols());

    matrix_inequality_boundary << matrix_lower_1,  - matrix_upper_1, matrix_lower_2,  - matrix_upper_2,
                                  matrix_lower_3,  - matrix_upper_3, matrix_lower_4,  - matrix_upper_4, matrix_lower_5, - matrix_upper_5;

    
    cout << "build pass in 87"<<endl;
    
    
    MatrixXd matrix_equality_1 (1, state);
    matrix_equality_1 << 2, 3, 4;

    MatrixXd matrix_boundary (1, 1);
    matrix_boundary << 5;

    cout<<"build pass in 91"<<endl;


    MatrixXd matrix_equality_constrain = matrix_equality_1;

    MatrixXd matrix_equality_boundary = matrix_boundary ;
    */
    MatrixXd matrix_equality_2 = MatrixXd::Zero(1, state);
    MatrixXd matrix_equality_boundary_2 = MatrixXd::Zero(1, 1);
    MatrixXd matrix_equality_constrain = matrix_equality_2;
    MatrixXd matrix_equality_boundary = matrix_equality_boundary_2;

    MatrixXd matrix_inequality_2 = MatrixXd::Zero(1, state);
    MatrixXd matrix_inequality_boundary_2 = MatrixXd::Zero(1, 1);
    MatrixXd matrix_inequality_constrain = matrix_inequality_2;
    MatrixXd matrix_inequality_boundary = matrix_inequality_boundary_2;



    unique_ptr<QpSolver> qp_solver(new ActiveSetQpSolver(
    A, B, matrix_inequality_constrain,
         matrix_inequality_boundary, matrix_equality_constrain,
         matrix_equality_boundary));
    auto result = qp_solver->Solve();
    if (!result) {
            cout << "Linear MPC solver failed"<<endl;
            return 0;
    }
    if (result) {
            cout << "Linear MPC solver successed"<<endl;
    }
        


    MatrixXd matrix_solve = qp_solver->params();

    cout<< "the matrix_solve:\n" << matrix_solve << endl;
        


    
    
    return 0;
}

