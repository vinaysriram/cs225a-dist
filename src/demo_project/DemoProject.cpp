
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

void run() 
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
  optitrack_->openCsv("../resources/optitrack_120.csv");
  
  while (g_runloop) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (!optitrack_->getFrame()) continue;
    for (auto& pos : optitrack_->pos_rigid_bodies_) {
      cout << pos(0) << ", " << pos(1) << ", "  << pos(2) << ", ";
    }
    redis_client_.getEigenMatrixDerivedString(JOINT_ANGLES_KEY, robot->_q);
    redis_client_.getEigenMatrixDerivedString(JOINT_VELOCITIES_KEY, robot->_dq);
    robot->updateModel();
    robot->position(posn, "right_l6", op_pos_task_pos_in_link);
    cout << posn(0) << ", " << posn(1) << ", " << posn(2) << endl;
    usleep(5000000);    
  }
}

int main(int argc, char** argv) 
{
  RedisClient redis_client_;
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = stop;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;  
  sigaction(SIGINT, &sigIntHandler, NULL);
  run(); return 0;
}
