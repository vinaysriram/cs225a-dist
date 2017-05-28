
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "optitrack/OptiTrackClient.h"

#include <Eigen/Core>
#include <hiredis/hiredis.h>
#include <model/ModelInterface.h>

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

void run(bool dataCollection) 
{
  auto robot = new Model::ModelInterface(robot_file, Model::rbdl, Model::urdf, false);
  Eigen::Vector3d op_pos_task_pos_in_link = Eigen::Vector3d(0.0, 0.0, 0.0);
  Eigen::Vector3d posn;

  RedisClient redis_client_;
  redis_client_.serverIs(kRedisServerInfo);
  std::unique_ptr<OptiTrackClient> optitrack_;
  
  // Set up optitrack
  optitrack_ = make_unique<OptiTrackClient>();
  optitrack_->openConnection("172.24.69.66");
  
  Eigen::Matrix3d rot;
  Eigen::VectorXd aug(4); Eigen::Vector3d fin; Eigen::MatrixXd T(4, 4);
  T(0, 0) = 0.9996; T(0, 1) = 0.0261; T(0, 2) = -0.0141; T(0, 3) = 1.5104;
  T(1, 0) = -0.0135; T(1, 1) = -0.0238; T(1, 2) = -0.9996; T(1, 3) = -0.5664;
  T(2, 0) = -0.0264; T(2, 1) = 0.9994; T(2, 2) = -0.0234; T(2, 3) = -0.4086;
  T(3, 0) = 0; T(3, 1) = 0; T(3, 2) = 0; T(3, 3) = 1.0000;

  if(dataCollection) {
    cout << "Data Collection" << endl;
    while (g_runloop) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      if (!optitrack_->getFrame()) continue;
      for (auto& pos : optitrack_->pos_rigid_bodies_) {
	aug << pos(0), pos(1), pos(2), 1;
      }
      aug = T * aug;
      fin << aug(0), aug(1), aug(2);
      redis_client_.getEigenMatrixDerivedString(JOINT_ANGLES_KEY, robot->_q);
      redis_client_.getEigenMatrixDerivedString(JOINT_VELOCITIES_KEY, robot->_dq);
      robot->updateModel();
      robot->position(posn, "right_l6", op_pos_task_pos_in_link);
      robot->rotation(rot, "right_l6");
      fin = rot.transpose() * (fin - posn); 
      cout << fin(0) << ", " << fin(1) << ", " << fin(2) << endl;
      usleep(5000000);
    }
  } else {
    cout << "Optitrack Runner" << endl;
    while(g_runloop) {    
      if (!optitrack_->getFrame()) continue;
      for (auto& pos : optitrack_->pos_rigid_bodies_) {
	aug << pos(0), pos(1), pos(2), 1; aug = T * aug; fin << aug(0), aug(1), aug(2);
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
  run(dataCollection);
  return 0;
}
