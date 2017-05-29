
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "optitrack/OptiTrackClient.h"

#include <Eigen/Core>
#include <hiredis/hiredis.h>
#include <model/ModelInterface.h>

#include <cstdlib>
#include <string>
#include <thread>
#include <iostream>
#include <fstream>
#include <signal.h>
#include <unistd.h>

static volatile bool g_runloop = true;
void stop(int s) { g_runloop = false; }
using namespace std;

static const HiredisServerInfo kRedisServerInfo = {
  "127.0.0.1",  // hostname
  6379,         // port                                                               
  { 1, 500000 } // timeout = 1.5 seconds
};

const string world_file = "resources/demo_project/world.urdf";
const string robot_file = "resources/demo_project/sawyer.urdf";
const string robot_name = "sawyer";
const std::string JOINT_ANGLES_KEY  = "cs225a::robot::sawyer::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "cs225a::robot::sawyer::sensors::dq";
const std::string TARGET_POSITION_KEY  = "cs225a::robot::sawyer::tasks::tg_pos";
const std::string TRANSFORM_MATRIX_COL_1 = "cs225a::robot::sawyer::tasks_T_col1";
const std::string TRANSFORM_MATRIX_COL_2 = "cs225a::robot::sawyer::tasks_T_col2";
const std::string TRANSFORM_MATRIX_COL_3 = "cs225a::robot::sawyer::tasks_T_col3";
const std::string TRANSFORM_MATRIX_COL_4 = "cs225a::robot::saywer::tasks_T_col4";

void run(bool dataCollection, int n) 
{
  auto robot = new Model::ModelInterface(robot_file, Model::rbdl, Model::urdf, false);
  Eigen::Vector3d op_pos_task_pos_in_link = Eigen::Vector3d(0.0, 0.0, 0.036); 
  Eigen::MatrixXd opti(n, 3); Eigen::MatrixXd EE(n, 3); Eigen::Vector3d posn;
  
  RedisClient redis_client_;
  redis_client_.serverIs(kRedisServerInfo);
  std::unique_ptr<OptiTrackClient> optitrack_;
  
  // Set up optitrack
  optitrack_ = make_unique<OptiTrackClient>();
  optitrack_->openConnection("172.24.69.66");
  Eigen::VectorXd aug(4); Eigen::Vector3d fin; 
  
  if(dataCollection) {
    cout << "Data Collection" << endl;
    int i = 0;
    while (i < n && g_runloop) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      if (!optitrack_->getFrame()) continue;
      for (auto& pos : optitrack_->pos_rigid_bodies_) {
	opti(i, 0) = pos(0); opti(i, 1) = pos(1); opti(i, 2) = pos(2);
	cout << "Opti: " << pos.transpose() << endl;
      }
      redis_client_.getEigenMatrixDerivedString(JOINT_ANGLES_KEY, robot->_q);
      redis_client_.getEigenMatrixDerivedString(JOINT_VELOCITIES_KEY, robot->_dq);
      robot->updateModel();
      robot->position(posn, "right_l6", op_pos_task_pos_in_link);
      cout << "EE: " << posn.transpose() << endl;
      EE(i, 0) = posn(0); EE(i, 1) = posn(1); EE(i, 2) = posn(2);
      usleep(5000000);
      i++;
    }
    // Average of points in respective frames
    Eigen::VectorXd optiBar(3);
    optiBar = (opti.colwise().sum())/n;
    Eigen::VectorXd EEbar(3);
    EEbar = (EE.colwise().sum())/n;
    // Residuals of points in respective frames
    Eigen::MatrixXd optiTilde(n,3);
    Eigen::MatrixXd EETilde(n,3);
    for(int i =0;i<n;i++) {
      optiTilde.row(i) = opti.row(i)  - optiBar.transpose();
      EETilde.row(i) = EE.row(i)  - EEbar.transpose();
    }
    // Minimize R using SVD
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3,3);
    Eigen::MatrixXd a(n,3);
    Eigen::MatrixXd b(n,3);
    a = optiTilde;
    b = EETilde;
    // a'b outer matrix multiplication
    Eigen::MatrixXd H_temp(3,3);
    for(int i = 0; i<n; i++) {   
      H_temp = (a.row(i)).transpose()*b.row(i);
      H = H+H_temp;
    }
    // Find SVD
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd V;
    Eigen::MatrixXd U;
    V = svd.matrixV();
    U = svd.matrixU();
    Eigen::MatrixXd R;
    R = V*U.transpose();
    // Check if determinant of R is close to 1
    double determinant;
    determinant = R.determinant();
    if(abs(determinant - 1) <= 0.01) {   
      cout << "Correct Algorithm" << endl;
    } else {
      cout << "Algorithm at risk for failure. Det(R) ~= 1" << endl;
    }
    // Find p (i.e. translation vector)
    Eigen::VectorXd p(3);
    p = EEbar-R*optiBar;
    cout << R << endl;
    cout << p << endl;
  } else {
    cout << "Optitrack Runner..." << endl;
    Eigen::MatrixXd T(4, 4);
    T << 0.999945,  0.00541357, -0.00898438, 1.77262,
      -0.00900805,  0.00436509,    -0.99995, -0.578284,
      -0.00537408,   0.999976,  0.00441361, -0.348513,
      0, 0, 0, 1;
    while(g_runloop) {    
      if (!optitrack_->getFrame()) continue;
      for (auto& pos : optitrack_->pos_rigid_bodies_) {
	aug << pos(0), pos(1), pos(2), 1; aug = T * aug; fin << aug(0), aug(1), aug(2);
	cout << aug.transpose() << endl;
	redis_client_.setEigenMatrixDerivedString(TARGET_POSITION_KEY, fin);
      }
    } 
  }
}

int main(int argc, char** argv) 
{
  bool dataCollection;
  if(argc == 1) {dataCollection = false;}
  else {dataCollection = true;}
  RedisClient redis_client_;
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = stop;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;  
  sigaction(SIGINT, &sigIntHandler, NULL);
  if(dataCollection) {
    run(dataCollection, atoi(argv[1]));
  } else {
    run(dataCollection, 0);
  }
  return 0;
 }
