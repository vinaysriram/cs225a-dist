#include <model/ModelInterface.h>
#include "optitrack/OptiTrackClient.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include <Eigen/Dense>

#include <cmath>
#include <cfloat>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <string>
#include <signal.h>

static volatile bool runloop = true;
void stop(int) { runloop = false; }

using namespace std;

static string world_file = "";
static string robot_file = "";
static string robot_name = "";

static unsigned long long controller_counter = 0;

static std::string JOINT_TORQUES_COMMANDED_KEY = "";
static std::string JOINT_ANGLES_KEY  = "";
static std::string JOINT_VELOCITIES_KEY = "";
static std::string TIMESTAMP_KEY = "";
static std::string KP_POSITION_KEY = "";
static std::string KV_POSITION_KEY = "";
static std::string KP_ORIENTATION_KEY = "";
static std::string KV_ORIENTATION_KEY = "";
static std::string KP_JOINT_KEY = "";
static std::string KV_JOINT_KEY = "";
static std::string KP_JOINT_INIT_KEY = "";
static std::string KV_JOINT_INIT_KEY = "";
static std::string DESIRED_JOINT_POS = "";
static std::string DESIRED_POS_KEY_OP = "";
static std::string TARGET_POS_KEY_OP = "";
static std::string J5_POS_KEY_OP = "";
static std::string J6_POS_KEY_OP = "";
static std::string ROT_5_X = "";
static std::string ROT_5_Y = "";
static std::string ROT_5_Z = "";
static std::string ROT_6_X = "";
static std::string ROT_6_Y = "";
static std::string ROT_6_Z = "";
static std::string BARREL = "";

void parseCommandline(int argc, char** argv)
{
  if (argc != 4) {
    cout << "Usage: project_controller <path-to-world.urdf> <path-to-robot.urdf> <robot-name>" << endl;
    exit(0);
  }
  world_file = string(argv[1]);
  robot_file = string(argv[2]);
  robot_name = string(argv[3]);
}

int main(int argc, char** argv)
{  
  parseCommandline(argc, argv);
  JOINT_TORQUES_COMMANDED_KEY = "cs225a::robot::" + robot_name + "::actuators::fgc";
  JOINT_ANGLES_KEY            = "cs225a::robot::" + robot_name + "::sensors::q";
  JOINT_VELOCITIES_KEY        = "cs225a::robot::" + robot_name + "::sensors::dq";
  TIMESTAMP_KEY               = "cs225a::robot::" + robot_name + "::timestamp";
  KP_POSITION_KEY             = "cs225a::robot::" + robot_name + "::tasks::kp_pos";
  KV_POSITION_KEY             = "cs225a::robot::" + robot_name + "::tasks::kv_pos";
  KP_ORIENTATION_KEY          = "cs225a::robot::" + robot_name + "::tasks::kp_ori";
  KV_ORIENTATION_KEY          = "cs225a::robot::" + robot_name + "::tasks::kv_ori";
  KP_JOINT_KEY                = "cs225a::robot::" + robot_name + "::tasks::kp_joint";
  KV_JOINT_KEY                = "cs225a::robot::" + robot_name + "::tasks::kv_joint";
  KP_JOINT_INIT_KEY           = "cs225a::robot::" + robot_name + "::tasks::kp_joint_init";
  KV_JOINT_INIT_KEY           = "cs225a::robot::" + robot_name + "::tasks::kv_joint_init";
  DESIRED_JOINT_POS           = "cs225a::robot::" + robot_name + "::tasks::jt_pos_des";
  TARGET_POS_KEY_OP           = "cs225a::robot::" + robot_name + "::tasks::tg_pos";
  J5_POS_KEY_OP               = "cs225a::robot::" + robot_name + "::tasks::j5_pos";
  J6_POS_KEY_OP               = "cs225a::robot::" + robot_name + "::tasks::j6_pos";   
  ROT_5_X                     = "cs225a::robot::" + robot_name + "::tasks::rot_5x";
  ROT_5_Y                     = "cs225a::robot::" + robot_name + "::tasks::rot_5y";
  ROT_5_Z                     = "cs225a::robot::" + robot_name + "::tasks::rot_5z";
  ROT_6_X                     = "cs225a::robot::" + robot_name + "::tasks::rot_6x";
  ROT_6_Y                     = "cs225a::robot::" + robot_name + "::tasks::rot_6y";
  ROT_6_Z                     = "cs225a::robot::" + robot_name + "::tasks::rot_6z";
  BARREL                      = "cs225a::robot::" + robot_name + "::tasks::barrel";
  
  cout << "Loading URDF world model file: " << world_file << endl;
  cout << JOINT_ANGLES_KEY << endl;
  cout << JOINT_VELOCITIES_KEY << endl;

  // Make sure redis-server is running at localhost with default port 6379
  HiredisServerInfo info;
  info.hostname_ = "127.0.0.1";
  info.port_ = 6379;
  info.timeout_ = { 1, 500000 }; // 1.5 seconds
  auto redis_client = RedisClient();
  redis_client.serverIs(info);
  
  // Load robot
  auto robot = new Model::ModelInterface(robot_file, Model::rbdl, Model::urdf, false);
  robot->updateModel();
  const int dof = robot->dof();
  // Create a loop timer
  const double control_freq = 1000;
  LoopTimer timer;
  timer.setLoopFrequency(control_freq);   // 1 KHz
  // timer.setThreadHighPriority();  // make timing more accurate. requires running executable as sudo.
  timer.setCtrlCHandler(stop);    // exit while loop on ctrl-c
  timer.initializeTimer(1e6); // 1 ms pause before starting loop

  /*******************************
   ***** JOINT SPACE CONTROL *****
   *******************************/
  
  string redis_buf; double kp_joint_init, kv_joint_init;
  Eigen::VectorXd command_torques = Eigen::VectorXd::Zero(dof);
  Eigen::VectorXd q_err(dof), g(dof), q_initial(dof);
  
  Eigen::Vector3d x_initial;
  const double TOLERANCE_Q_INIT = 0.1;
  const double TOLERANCE_DQ_INIT = 0.1;
  
  redis_client.getEigenMatrixDerivedString(DESIRED_JOINT_POS, q_initial);
  cout << "Joint Space Controller: Initializing to Home Position." << endl;
  
  while (runloop) {
    // wait for next scheduled loop (controller must run at precise rate)
    timer.waitForNextLoop();
    
    // read from Redis current sensor values
    redis_client.getEigenMatrixDerivedString(JOINT_ANGLES_KEY, robot->_q);
    redis_client.getEigenMatrixDerivedString(JOINT_VELOCITIES_KEY, robot->_dq);
    
    // Update the model
    robot->updateModel();
    
    // Read in KP and KV from Redis (can be changed on the fly in Redis)
    redis_client.getCommandIs(KP_JOINT_INIT_KEY, redis_buf);
    kp_joint_init = stoi(redis_buf);
    redis_client.getCommandIs(KV_JOINT_INIT_KEY, redis_buf);
    kv_joint_init = stoi(redis_buf);
    
    // Break if the robot has converged to q_initial
    q_err = robot->_q - q_initial;
    if (q_err.norm() < TOLERANCE_Q_INIT && robot->_dq.norm() < TOLERANCE_DQ_INIT) {
      robot->position(x_initial, "right_l6", Eigen::Vector3d::Zero());
      break;
    }
    
    // Compute torques
    robot->gravityVector(g);
    command_torques = robot->_M * (-kp_joint_init * q_err - kv_joint_init * robot->_dq) + g;
    
    // Send torques
    redis_client.setEigenMatrixDerivedString(JOINT_TORQUES_COMMANDED_KEY, command_torques);
    controller_counter++;
  }
  
  /*************************************
   ***** OPERATIONAL SPACE CONTROL *****
   *************************************/
  
  double kp_pos, kv_pos;
  double kp_ori, kv_ori;
  double kp_joint, kv_joint;
  
  // Initialize controller variables
  Eigen::MatrixXd lambda(3, 3);
  Eigen::MatrixXd jw(3, dof);
  Eigen::MatrixXd N(7, 7);

  Eigen::Vector3d target_pos;
  Eigen::Vector3d x_hat, y_hat, z_hat;
  Eigen::Vector3d current_j6_pos;
  Eigen::Vector3d current_j5_pos;
  Eigen::Vector3d current_angl_vel;
  
  Eigen::Matrix3d rotation_j5;
  Eigen::Matrix3d rotation_j6;
  Eigen::Matrix3d rotation_desired;

  Eigen::Vector3d rotation_vec1;
  Eigen::Vector3d rotation_vec2;
  Eigen::Vector3d orient_error;
 
  Eigen::VectorXd diff_rotation(3);
  Eigen::VectorXd diff_joint(dof);
  
  Eigen::Vector3d offset_init;
  Eigen::Vector3d barrel_tip_pos;
  Eigen::Vector3d measurementVector;
  Eigen::Vector3d offsetVector;
  
  measurementVector << -0.0882647, -0.159753, -0.05;
  //measurementVector << -0.100, -0.156, -0.05;
  offsetVector << 0, 0, 0.03;
  target_pos = x_initial; target_pos(0) = target_pos(0) + 0.5;  
  redis_client.setEigenMatrixDerivedString(TARGET_POS_KEY_OP, target_pos);
  cout << "Joint position initialized. Switching to operational space controller." << endl;
  
  // While window is open:
  while (runloop) {
    
    // Wait for next scheduled loop (controller must run at precise rate)
    timer.waitForNextLoop();
    
    // Get current simulation timestamp from Redis
    redis_client.getCommandIs(TIMESTAMP_KEY, redis_buf);
    double t_curr = stod(redis_buf);
    
    // Read from Redis current sensor value
    robot->position(current_j6_pos, "right_l6", offsetVector);
    robot->position(current_j5_pos, "right_l5", Eigen::Vector3d::Zero());
    robot->rotation(rotation_j5, "right_l5");
    rotation_j6.col(0) = -rotation_j5.col(2);
    rotation_j6.col(1) = rotation_j5.col(0);
    rotation_j6.col(2) = rotation_j6.col(0).cross(rotation_j6.col(1));
    
    //barrel_tip_pos = current_j5_pos;
    barrel_tip_pos = rotation_j6 * measurementVector + current_j6_pos;
    
    redis_client.getEigenMatrixDerivedString(JOINT_ANGLES_KEY, robot->_q);
    redis_client.getEigenMatrixDerivedString(JOINT_VELOCITIES_KEY, robot->_dq);
    redis_client.getEigenMatrixDerivedString(TARGET_POS_KEY_OP, target_pos);    
    
    x_hat = barrel_tip_pos - target_pos; x_hat.normalize();
    z_hat << 0, x_hat(2), -x_hat(1); z_hat.normalize();
    y_hat = z_hat.cross(x_hat);
    
    rotation_desired.col(0) = x_hat;
    rotation_desired.col(1) = y_hat;
    rotation_desired.col(2) = z_hat;
    
    // Update the model
    robot->updateModel();
    
    // Read in KP and KV from Redis (can be changed on the fly in Redis)
    redis_client.getCommandIs(KP_POSITION_KEY, redis_buf);
    kp_pos = stoi(redis_buf);
    redis_client.getCommandIs(KV_POSITION_KEY, redis_buf);
    kv_pos = stoi(redis_buf);
    redis_client.getCommandIs(KP_ORIENTATION_KEY, redis_buf);
    kp_ori = stoi(redis_buf);
    redis_client.getCommandIs(KV_ORIENTATION_KEY, redis_buf);
    kv_ori = stoi(redis_buf);
    redis_client.getCommandIs(KP_JOINT_KEY, redis_buf);
    kp_joint = stoi(redis_buf);
    redis_client.getCommandIs(KV_JOINT_KEY, redis_buf);
    kv_joint = stoi(redis_buf);
    
    /* Set Fields */
    robot->angularVelocity(current_angl_vel, "right_l5");
    robot->Jw(jw, "right_l5");
    robot->nullspaceMatrix(N, jw);
    robot->gravityVector(g);
    robot->orientationError(orient_error, rotation_desired, rotation_j5);
    
    /*
    orient_error << 0, 0, 0;
    for(int i = 0; i < 3; i++) {
      rotation_vec1 << rotation_j5(0, i), rotation_j5(1, i), rotation_j5(2, i);
      rotation_vec2 << rotation_desired(0, i), rotation_desired(1, i), rotation_desired(2, i);
      orient_error = orient_error + rotation_vec1.cross(rotation_vec2);
    }
    orient_error = -orient_error/2;
    */

    robot->taskInertiaMatrixWithPseudoInv(lambda, jw);
    cout << orient_error.transpose() << endl;

    diff_rotation  = (-kp_ori*orient_error) + (-kv_ori*current_angl_vel);    
    diff_joint = kp_joint * (q_initial - robot->_q) - kv_joint * robot->_dq;
    command_torques = jw.transpose() * lambda * diff_rotation + N.transpose() * robot->_M * diff_joint + g;
    
    redis_client.setEigenMatrixDerivedString(J6_POS_KEY_OP, current_j6_pos);
    redis_client.setEigenMatrixDerivedString(J5_POS_KEY_OP, current_j5_pos);
    redis_client.setEigenMatrixDerivedString(BARREL, barrel_tip_pos);
    redis_client.setEigenMatrixDerivedString(JOINT_TORQUES_COMMANDED_KEY, command_torques);
    redis_client.setEigenMatrixDerivedString(ROT_5_X, rotation_j5.col(0));
    redis_client.setEigenMatrixDerivedString(ROT_5_Y, rotation_j5.col(1));
    redis_client.setEigenMatrixDerivedString(ROT_5_Z, rotation_j5.col(2));
    redis_client.setEigenMatrixDerivedString(ROT_6_X, rotation_j6.col(0));
    redis_client.setEigenMatrixDerivedString(ROT_6_Y, rotation_j6.col(1));
    redis_client.setEigenMatrixDerivedString(ROT_6_Z, rotation_j6.col(2));
    controller_counter++;
  }
  
  command_torques.setZero();
  redis_client.setEigenMatrixDerivedString(JOINT_TORQUES_COMMANDED_KEY, command_torques);
  return 0;
}
