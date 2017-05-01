#include <model/ModelInterface.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <string>

#include <signal.h>
static volatile bool runloop = true;
void stop(int) { runloop = false; }

using namespace std;

// Location of URDF files specifying world and robot information
static string world_file = "";
static string robot_file = "";
static string robot_name = "";

static unsigned long long controller_counter = 0;

// Redis keys:
// - write:
static std::string JOINT_TORQUES_COMMANDED_KEY = "";

// - read:
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
static std::string OPTION_KEY = "";
static std::string ROTATION_MATRIX_KEY = "";
static std::string DESIRED_POS_KEY_OP = "";
static std::string DESIRED_POS_KEY_JOINT = "";

// Function to parse command line arguments
void parseCommandline(int argc, char** argv);

int main(int argc, char** argv) {

	// Parse command line and set redis keys
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
	OPTION_KEY                  = "cs225a::robot::" + robot_name + "::control::option_key";
	ROTATION_MATRIX_KEY         = "cs225a::robot::" + robot_name + "::tasks::rotation_matrix";
	DESIRED_POS_KEY_OP          = "cs225a::robot::" + robot_name + "::tasks::desired_task_pos";
	
	cout << "Loading URDF world model file: " << world_file << endl;
	cout << JOINT_ANGLES_KEY << endl;
	cout << JOINT_VELOCITIES_KEY << endl;

	// Start redis client
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

	/**
 	 * JOINT SPACE CONTROL
 	 * -------------------
 	 * Controller to initialize robot to desired joint position.
 	 * Uses kp = 400 and kv = 40 by default.
 	 */

	// Set gains in Redis if not initialized
	string redis_buf;
	double kp_joint_init = 30, kv_joint_init = 10;
	if (!redis_client.getCommandIs(KP_JOINT_INIT_KEY)) {
		redis_buf = to_string(kp_joint_init);
		redis_client.setCommandIs(KP_JOINT_INIT_KEY, redis_buf);
	}
	if (!redis_client.getCommandIs(KV_JOINT_INIT_KEY)) {
		redis_buf = to_string(kv_joint_init);
		redis_client.setCommandIs(KV_JOINT_INIT_KEY, redis_buf);
	}

	// Initialize controller variables
	Eigen::VectorXd command_torques = Eigen::VectorXd::Zero(dof);
	Eigen::VectorXd q_err(dof), g(dof);

	Eigen::VectorXd q_initial = Eigen::VectorXd::Zero(dof); // Desired initial joint position
	if (robot_name == "kuka_iiwa") {
		q_initial << 0, 30, 0, 60, 0, 90, 0;
		q_initial *= M_PI / 180.0;
	}
	Eigen::Vector3d x_initial; // Resulting end effector position from initialization
	const double TOLERANCE_Q_INIT = 0.1;
	const double TOLERANCE_DQ_INIT = 0.1;

	// While window is open:
	cout << "Joint space controller: initializing joint position..." << endl;
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
			robot->position(x_initial, "link6", Eigen::Vector3d::Zero());
			break;
		}

		// Compute torques
		robot->gravityVector(g);
		command_torques = robot->_M * (-kp_joint_init * q_err - kv_joint_init * robot->_dq) + g;

		// Send torques
		redis_client.setEigenMatrixDerivedString(JOINT_TORQUES_COMMANDED_KEY, command_torques);
		controller_counter++;
	}
	cout << "Joint position initialized. Switching to operational space controller." << endl;

	/**
 	 * OPERATIONAL SPACE CONTROL
 	 * -------------------------
 	 * Controller to implement for HW2
 	 */

	// Set gains in Redis if not initialized
	double kp_pos   = 0, kv_pos   = 0;
	double kp_ori   = 0, kv_ori   = 0;
	double kp_joint = 0, kv_joint = 0;
	if (!redis_client.getCommandIs(KP_POSITION_KEY)) {
		redis_buf = to_string(kp_pos);
		redis_client.setCommandIs(KP_POSITION_KEY, redis_buf);
	}
	if (!redis_client.getCommandIs(KV_POSITION_KEY)) {
		redis_buf = to_string(kv_pos);
		redis_client.setCommandIs(KV_POSITION_KEY, redis_buf);
	}
	if (!redis_client.getCommandIs(KP_ORIENTATION_KEY)) {
		redis_buf = to_string(kp_ori);
		redis_client.setCommandIs(KP_ORIENTATION_KEY, redis_buf);
	}
	if (!redis_client.getCommandIs(KV_ORIENTATION_KEY)) {
		redis_buf = to_string(kv_ori);
		redis_client.setCommandIs(KV_ORIENTATION_KEY, redis_buf);
	}
	if (!redis_client.getCommandIs(KP_JOINT_KEY)) {
		redis_buf = to_string(kp_joint);
		redis_client.setCommandIs(KP_JOINT_KEY, redis_buf);
	}
	if (!redis_client.getCommandIs(KV_JOINT_KEY)) {
		redis_buf = to_string(kv_joint);
		redis_client.setCommandIs(KV_JOINT_KEY, redis_buf);
	}
	
	// Initialize controller variables
	Eigen::MatrixXd lambda_small(3, 3);
	Eigen::MatrixXd lambda_big(6, 6);
	Eigen::MatrixXd jv(3, dof);
	Eigen::MatrixXd jw(3, dof);
	Eigen::MatrixXd j(6, dof);
	Eigen::MatrixXd N(7, 7);

	Eigen::Vector3d current_angl_vel;
	Eigen::Vector3d current_task_vel;
	Eigen::Vector3d current_task_pos;
	Eigen::Vector3d desired_task_pos;	
	Eigen::Vector3d desired_task_vel;

	Eigen::VectorXd desired_joint_pos = robot->_q;
	cout << desired_joint_pos.transpose() << endl;
	Eigen::Vector3d pos_in_frame = Eigen::Vector3d::Zero();
	cout << pos_in_frame.transpose() << endl;
	
	Eigen::Matrix3d rotation_current;
	Eigen::Matrix3d rotation_desired;
	Eigen::Vector3d rotation_vec1;
	Eigen::Vector3d rotation_vec2;
	Eigen::Vector3d orient_error;

	Eigen::VectorXd diff_rotation(6);
	Eigen::VectorXd diff_position(3);
	Eigen::VectorXd diff_joint2(dof);
	Eigen::VectorXd diff_joint(dof);
	Eigen::VectorXd diff_pos2(3);
	
	double s;
	
	/* Initialize Desired Option/Position/Orientation */

	redis_client.getCommandIs(OPTION_KEY, redis_buf); int option = stoi(redis_buf); 
	redis_client.getEigenMatrixDerivedString(ROTATION_MATRIX_KEY, rotation_desired);
	cerr << rotation_desired << endl;
	redis_client.getEigenMatrixDerivedString(DESIRED_POS_KEY_OP, desired_task_pos);
	cerr << desired_task_pos << endl;
	
	// While window is open:
	while (runloop) {
	  
	  // Wait for next scheduled loop (controller must run at precise rate)
	  timer.waitForNextLoop();

		// Get current simulation timestamp from Redis
		redis_client.getCommandIs(TIMESTAMP_KEY, redis_buf);
		double t_curr = stod(redis_buf);

		// Read from Redis current sensor values
		redis_client.getEigenMatrixDerivedString(JOINT_ANGLES_KEY, robot->_q);
		redis_client.getEigenMatrixDerivedString(JOINT_VELOCITIES_KEY, robot->_dq);

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
		robot->position(current_task_pos, "link6", pos_in_frame);
		robot->linearVelocity(current_task_vel, "link6", pos_in_frame);
		robot->angularVelocity(current_angl_vel, "link6");
		robot->taskInertiaMatrixWithPseudoInv(lambda_small, jv);
		robot->rotation(rotation_current, "link6");
		robot->Jv(jv, "link6", pos_in_frame);
		robot->Jw(jw, "link6");
		robot->nullspaceMatrix(N, jv);
		robot->gravityVector(g);

		/* Logging */
		for(int i = 0; i < 3; i++) {cout << current_task_pos(i) << ", ";}
		for(int i = 0; i < 3; i++) {cout << desired_task_pos(i) << ", ";}
		for(int i = 0; i < 7; i++) {cout << robot->_q(i) << ", ";}
		for(int i = 0; i < 3; i++) {cout << orient_error(i) << ", ";}
		for(int i = 0; i < 2; i++) {cout << current_task_vel(i) << ", ";}
		cout << current_task_vel(2) << endl;

		/* Controllers */
		diff_position = kp_pos * (desired_task_pos - current_task_pos) - kv_pos * current_task_vel;
		diff_joint = kp_joint * (desired_joint_pos - robot->_q) - kv_joint * robot->_dq;
		diff_joint2 = -kv_joint * robot->_dq;

		/* Command Torques */
		if(option == 1) {
		  command_torques = jv.transpose() * (lambda_small * diff_position) + N.transpose() * robot->_M * diff_joint2 + g;
		} else if(option == 2) {
		  cerr << rotation_current << endl;
		  orient_error << 0, 0, 0;
		  for(int i = 0; i < 3; i++) {
		    rotation_vec1 << rotation_current(0, i), rotation_current(1, i), rotation_current(2, i);
		    rotation_vec2 << rotation_desired(0, i), rotation_desired(1, i), rotation_desired(2, i);
		    orient_error = orient_error + rotation_vec1.cross(rotation_vec2);
		  }
		  j << jv, jw; orient_error = -orient_error/2;
		  robot->taskInertiaMatrixWithPseudoInv(lambda_big, j);
		  diff_rotation << diff_position, (-kp_ori*orient_error) + (-kv_ori*current_angl_vel);
		  command_torques = j.transpose() * lambda_big * diff_rotation + N.transpose() * robot->_M * diff_joint2 + g;
		} else if(option == 3) {
		  desired_task_vel = (kp_pos/kv_pos) * (desired_task_pos - current_task_pos);
		  if(0.05/(desired_task_vel.norm()) > 1) {
		    s = 1;
		  } else {
		    s = 0.05/(desired_task_vel.norm());
		  }
		  diff_pos2 = - kv_pos * (current_task_vel - s*desired_task_vel);
		  command_torques = jv.transpose() * (lambda_small * diff_pos2) + N.transpose() * robot->_M * diff_joint2 + g;
		}		
	
		/* Send Torques */
		redis_client.setEigenMatrixDerivedString(JOINT_TORQUES_COMMANDED_KEY, command_torques);
		controller_counter++;
	}

	command_torques.setZero();
	redis_client.setEigenMatrixDerivedString(JOINT_TORQUES_COMMANDED_KEY, command_torques);

	return 0;
}


//------------------------------------------------------------------------------
void parseCommandline(int argc, char** argv) {
	if (argc != 4) {
		cout << "Usage: hw2 <path-to-world.urdf> <path-to-robot.urdf> <robot-name>" << endl;
		exit(0);
	}
	// argument 0: executable name
	// argument 1: <path-to-world.urdf>
	world_file = string(argv[1]);
	// argument 2: <path-to-robot.urdf>
	robot_file = string(argv[2]);
	// argument 3: <robot-name>
	robot_name = string(argv[3]);
}
