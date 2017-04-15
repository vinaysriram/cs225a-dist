#include <model/ModelInterface.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <unistd.h>
#include <iostream>
#include <string>

using namespace std;

// Location of URDF files specifying world and robot information
const string world_file = "resources/hw0/world.urdf";
const string robot_file = "resources/hw0/RRPbot.urdf";
const string robot_name = "RRPbot";

// Redis is just a key value store, publish/subscribe is also possible
// The visualizer and simulator will have keys like "cs225a::robot::{ROBOTNAME}::sensors::q"
// You can hardcode the robot name in like below or read them in from cli
// redis keys:
// - write:
const std::string JOINT_ANGLES_KEY  = "cs225a::robot::RRPbot::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "cs225a::robot::RRPbot::sensors::dq";
const std::string MASS_MATRIX_KEY = "cs225a::robot::RRPbot::actuator::m11_m22_m33";
const std::string redis_ctrl_flag = "cs225a::robot::RRPbot::comm::redis_ctrl_flag";
const std::string redis_term_flag = "cs225a::robot::RRPbot::comm::redis_term_flag";

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;
	
	// Make sure redis-server is running at localhost with default port 6379
	// start redis client
	HiredisServerInfo info;
	info.hostname_ = "127.0.0.1";
	info.port_ = 6379;
	info.timeout_ = { 1, 500000 }; // 1.5 seconds
	auto redis_client = RedisClient();
	redis_client.serverIs(info);

	// load robots
	auto robot = new Model::ModelInterface(robot_file, Model::rbdl, Model::urdf, false);

	/*
	These are mathematical vectors from the library Eigen, you can read up on the documentation online.
	You can input your joint information and read sensor data C++ style "<<" or ">>". Make sure you only 
	expect to read or are writing #D.O.F. number of values.
	*/
	//robot->_q << 0, M_PI/3, 0.2; // Joint 1,2,3 Coordinates (radians, radians, meters)

	int option = 2;
	string returnValue;
	int dof = robot->dof();
	redis_client.setCommandIs(redis_ctrl_flag, "false");
	robot->_dq << 0, 0, 0; // Joint 1,2,3 Velocities (radians/sec, radians/sec, meters/sec), not used here 
	redis_client.setCommandIs(redis_term_flag, "false");
	redis_client.setEigenMatrixDerivedString(JOINT_VELOCITIES_KEY, robot->_dq);
	Eigen::VectorXd selectedEntries(7);
	Eigen::VectorXd g(dof);
	double theta2, d3; int i = 0;
	for(double param = -1; param <= 1; param += 0.01) {
	  theta2 = param * (M_PI/2);
	  d3 = (param + 1.0) / 10;
	  switch(option) {
	  case 1: robot->_q << 0, theta2, 0.1; selectedEntries(0) = theta2; break;
	  case 2: robot->_q << 0, 0, d3; selectedEntries(0) = d3; break;
	  case 3: robot->_q << 0, M_PI/2, d3; selectedEntries(0) = d3; break;
	  case 4: robot->_q << theta2, theta2, d3; selectedEntries(0) = d3; break;
	  }
	  
	  redis_client.setEigenMatrixDerivedString(JOINT_ANGLES_KEY,robot->_q);	  
	  robot->updateModel();
	  robot->gravityVector(g);
	  usleep(25000);
	  
	  do {
	    redis_client.getCommandIs(redis_ctrl_flag, returnValue);
	  } while(returnValue.compare("false") == 0);
	  
	  selectedEntries(1) = robot->_M(0,0);
	  selectedEntries(2) = robot->_M(1,1);
	  selectedEntries(3) = robot->_M(2,2);
	  selectedEntries(4) = g(0);
	  selectedEntries(5) = g(1);
	  selectedEntries(6) = g(2);
	  redis_client.setEigenMatrixDerivedString(MASS_MATRIX_KEY, selectedEntries);
	  redis_client.setCommandIs(redis_ctrl_flag, "false");
	  i++;
	  
	}
	cout << "number of lines: " << i << endl;
	redis_client.setCommandIs(redis_term_flag, "true");
	
	/* 
	Here we use our redis set method to serialize an 'Eigen' vector into a specific Redis Key
	Changing set to get populates the 'Eigen' vector given
	This key is then read by the physics integrator or visualizer to update the system
	*/
	redis_client.setEigenMatrixDerivedString(JOINT_ANGLES_KEY,robot->_q);
	redis_client.setEigenMatrixDerivedString(JOINT_VELOCITIES_KEY, robot->_dq);
      
	// operational space
	std::string op_pos_task_link_name = "link2"; // Link of the "Task" or "End Effector"

	// Position of Task Frame in relation to Link Frame (When using custom E.E. attachment, etc..)
	Eigen::Vector3d op_pos_task_pos_in_link = Eigen::Vector3d(0.0, 0.0, 0.0); 
	Eigen::MatrixXd op_pos_task_jacobian(3,dof); // Empty Jacobian Matrix sized to right size

	robot->Jv(op_pos_task_jacobian,op_pos_task_link_name,op_pos_task_pos_in_link); // Read jacobian into op_pos_task_jacobian
	cout << op_pos_task_jacobian << endl; // Print Jacobian

	cout << robot->_M << endl; // Print Mass Matrix, you can index into this variable (and all 'Eigen' types)!
	cout << g << endl;

	// This function retrives absolute position of Task Frame in Base Frame
	//Eigen::Vector3d initial_position;
	//robot->position(initial_position,op_pos_task_link_name,op_pos_task_pos_in_link);
	
	// Wait for user input before quit
	int wait;
	cin >> wait;

    return 0;
}

