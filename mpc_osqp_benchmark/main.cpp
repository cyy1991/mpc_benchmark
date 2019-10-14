
#include <algorithm>
#include <iomanip>
#include <limits>
#include <utility>
#include <vector>
#include "Eigen/LU"

#include "osqp.h"
#include "mpc_osqp/mpc_osqp.h"

#include<iostream>

using namespace std;
using namespace Eigen;

int main(int argc, char **argv)
{ 
  vector<vector<int>> columns;
  int state_dim = 3;
  int control_dim = 2;
  int horizon = 10;
  int num_param = state_dim * (horizon + 1) + control_dim * horizon;

  columns.resize(num_param);

  std::cout << columns.size() << std::endl;

  

  /*
  int state = 3;
  int controls = 2;
  int horizon = 5;
  int mpc_max_iteration = 200;
  double mpc_eps = 0.01;


  //MatrixXd lower_bound(controls_, 1);
  //lower_bound << -wheel_single_direction_max_degree_, max_deceleration_;

  MatrixXd matrix_ad(state, state);
  matrix_ad <<1, 0, 0,
              0, 1, 0.05,
              0, 0, 1;
  MatrixXd matrix_bd(state, controls);
  matrix_bd << 0.05, 0,
                 0,  0,
                 0,  0.05;
  MatrixXd matrix_q (state, state);
  matrix_q << 10, 0, 0,
              0, 10, 0,
              0, 0, 10;
  MatrixXd matrix_r(controls, controls);
  matrix_r << 10, 0,
              0, 10;

  MatrixXd matrix_state(state, 1);
  matrix_state << 0, 0, 1.0471;

  MatrixXd lower_bound(controls, 1);
  lower_bound << -0.2, -0.4363;
  MatrixXd upper_bound(controls, 1);
  upper_bound << 0.2, 0.4363;

  MatrixXd lower_state_bound(state, 1);
  lower_state_bound << -0.5, 2.5, -0.5;

  MatrixXd upper_state_bound(state, 1);
  upper_state_bound << 0.05, 3, 1.2;

  MatrixXd reference_state(state, 1);
  reference_state << 0.005, 2, 0;

  cout <<"reference_state: \n" << reference_state << endl;


  vector<double> control_cmd(controls, 0);

  MatrixXd control_matrix = MatrixXd::Zero(controls, 1);

  vector<MatrixXd> control(horizon, control_matrix);

  cout<<"build passed in 67"<<endl;

  MpcOsqp mpc_osqp(
       matrix_ad, matrix_bd, matrix_q, matrix_r,
        matrix_state, lower_bound, upper_bound, lower_state_bound,
        upper_state_bound, reference_state, mpc_max_iteration, horizon,
        mpc_eps);
  
  cout << "build passed in 75" << endl;

  
  if (!mpc_osqp.Solve(&control_cmd)) {
      AERROR << "MPC OSQP solver failed" << endl;
    }
    
  else {
      ADEBUG << "MPC OSQP problem solved! " <<endl ;
      control[0](0, 0) = control_cmd.at(0);
      control[0](1, 0) = control_cmd.at(1);
    }

    cout<<"build pass in 93"<<endl; */




    // Load problem data
  /*
  c_float P_x[3] = { 4.0, 1.0, 2.0, };
  c_int   P_nnz  = 3;
  c_int   P_i[3] = { 0, 0, 1, };
  c_int   P_p[3] = { 0, 1, 3, };
  c_float q[2]   = { 1.0, 1.0, };
  c_float A_x[4] = { 1.0, 1.0, 1.0, 1.0, };
  c_int   A_nnz  = 4;
  c_int   A_i[4] = { 0, 1, 0, 2, };
  c_int   A_p[3] = { 0, 2, 4, };
  c_float l[3]   = { 1.0, 0.0, 0.0, };
  c_float u[3]   = { 1.0, 0.7, 0.7, };
  c_int n = 2;
  c_int m = 3;*/

  c_float P_x[5] = { 2.0, 1.0, 4.0, 1.0, 6.0,};
  c_int   P_nnz  = 5;
  c_int   P_p[4] = { 0, 2, 3, 5, };  //index
  c_int   P_i[5] = { 0, 2, 1, 0, 2, }; //pianyi
  c_float q[3]   = { 1.0, -2.0, 4.0,};
  c_float A_x[12] = { 3.0, -3.0, 2.0, 1.0, 4.0, 2.0, 3.0, 1.0, -2.0, 1.0, 4.0, 1.0, };
  c_int   A_nnz  = 12;
  c_int   A_p[4] = { 0, 4, 8, 12, }; //index
  c_int   A_i[12] = { 0, 1, 2, 3, 0, 1, 2, 4, 0, 1, 2, 5, }; //pianyi
  c_float l[6]   = { -20.0, 2.0, 5.0, 0.0, 1.0, 0.0,};
  c_float u[6]   = { 10.0, 20.0, 5.0, 5.0, 5.0, 5.0,};
  c_int n = 3;
  c_int m = 6;

  // Exitflag
  c_int exitflag = 0;

  // Workspace structures
  OSQPWorkspace *work;
  OSQPSettings  *settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
  OSQPData      *data     = (OSQPData *)c_malloc(sizeof(OSQPData));

  // Populate data
  if (data) {
    data->n = n;
    data->m = m;
    data->P = csc_matrix(data->n, data->n, P_nnz, P_x, P_i, P_p);
    data->q = q;
    data->A = csc_matrix(data->m, data->n, A_nnz, A_x, A_i, A_p);
    data->l = l;
    data->u = u;
  }

  // Define solver settings as default
  if (settings) osqp_set_default_settings(settings);

  // Setup workspace
  // Setup workspace
  //exitflag = osqp_setup(&work, data, settings);
  work = osqp_setup(data, settings);

  // Solve Problem
  osqp_solve(work);

  
  auto status = work->info->status_val;
  std::cout <<"status :" << status << endl;

  if (status < 0 || (status != 1 && status != 2)) {
    std::cout << "failed optimization status:\n" << work->info->status <<endl;
    osqp_cleanup(work);
    c_free(data);
    c_free(settings);
    return 0;
  } else if (work->solution == NULL) {
    std::cout << "The solution from OSQP is nullptr" << endl;
    osqp_cleanup(work);
    c_free(data);
    c_free(settings);
    return 0;
  }
  for(int i = 0; i < 3; ++i)
  {
    std::cout <<"the result :"<<endl;
    std::cout<< work->solution->x[i]<<endl;
  }
  // Clean workspace
  osqp_cleanup(work);
  if (data) {
    if (data->A) c_free(data->A);
    if (data->P) c_free(data->P);
    c_free(data);
  }
  if (settings)  c_free(settings);

  return exitflag;
}



