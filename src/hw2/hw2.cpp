#include <model/ModelInterface.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

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
static std::string KP_JOINT_KEY = "";
static std::string KV_JOINT_KEY = "";
static std::string KP_JOINT_INIT_KEY = "";
static std::string KV_JOINT_INIT_KEY = "";

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
	KP_JOINT_KEY                = "cs225a::robot::" + robot_name + "::tasks::kp_joint";
	KV_JOINT_KEY                = "cs225a::robot::" + robot_name + "::tasks::kv_joint";
	KP_JOINT_INIT_KEY           = "cs225a::robot::" + robot_name + "::tasks::kp_joint_init";
	KV_JOINT_INIT_KEY           = "cs225a::robot::" + robot_name + "::tasks::kv_joint_init";

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
	double kp_joint_init = 400, kv_joint_init = 40;
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
	cout << "Initial Position: " << x_initial.transpose() << endl;
	
	/**
 	 * OPERATIONAL SPACE CONTROL
 	 * -------------------------
 	 * Controller to implement for HW2
 	 */
	
	
	// Set gains in Redis if not initialized
	double kp_pos   = 0, kv_pos   = 0; // For positioning task
	double kp_joint = 0, kv_joint = 0; // For nullspace damping
	if (!redis_client.getCommandIs(KP_POSITION_KEY)) {
		redis_buf = to_string(kp_pos);
		redis_client.setCommandIs(KP_POSITION_KEY, redis_buf);
	}
	if (!redis_client.getCommandIs(KV_POSITION_KEY)) {
		redis_buf = to_string(kv_pos);
		redis_client.setCommandIs(KV_POSITION_KEY, redis_buf);
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
	Eigen::Vector3d force;
	Eigen::MatrixXd lambda(3, 3);
	Eigen::MatrixXd N(7, 7);
	Eigen::VectorXd gravity_vec(dof);
	Eigen::MatrixXd jacobian(3, dof);
	Eigen::MatrixXd jac_inv(dof, 3);
	Eigen::Vector3d current_pos;
	Eigen::Vector3d current_vel;
	Eigen::Vector3d pos_in_frame;
	pos_in_frame.setZero();
	Eigen::Vector3d desired_pos;

	/* Note: One of the below is used for the fixed desired point cases. */
	
	desired_pos << 0.6, -0.1, 0.4;
	desired_pos << 0.5, 0.0, 0.8;
	
	// While window is open:
	double t = 0;
	while (runloop) {
	  
		// Wait for next scheduled loop (controller must run at precise rate)
		timer.waitForNextLoop();

		/* Note: The below is used for trajectory tracking. */
		
		desired_pos(0) = x_initial(0) - 0.1 + 0.2 * sin(2*M_PI*t); 
		desired_pos(1) = x_initial(1) + 0.2 * cos(2*M_PI*t);
		desired_pos(2) = x_initial(2) + 0.05;
		
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
		redis_client.getCommandIs(KV_JOINT_KEY, redis_buf);
		kv_joint = stoi(redis_buf);

		//------ Compute controller torques
		robot->position(current_pos, "link6", pos_in_frame);
		for(int i = 0; i < 3; i++) {cout << current_pos(i) << ", ";}
		for(int i = 0; i < 3; i++) {cout << desired_pos(i) << ", ";}
		for(int i = 0; i < 6; i++) {cout << robot->_q(i) << ", ";}
		cout << robot->_q(6) << endl;
		robot->linearVelocity(current_vel, "link6", pos_in_frame);
		robot->Jv(jacobian, "link6", pos_in_frame);
		robot->taskInertiaMatrixWithPseudoInv(lambda, jacobian);
		robot->gravityVector(gravity_vec);
		robot->nullspaceMatrix(N, jacobian);
		robot->dynConsistentInverseJacobian(jac_inv, jacobian);

		/* Note: We comment all but one of the below controllers depending on which subproblem we wish to solve. */
		
		/* With No Joint Space Damping */

		force = lambda * (kp_pos * (desired_pos - current_pos) - kv_pos * current_vel);
		command_torques = jacobian.transpose() * force + gravity_vec;
		
		/* With Joint Space Damping */
		
		force = lambda * (kp_pos * (desired_pos - current_pos) - kv_pos * current_vel);
		command_torques = jacobian.transpose() * force + gravity_vec - (kv_joint * robot->_dq);

		/* With Null Space Damping and Mass Matrix */
		force = lambda * (kp_pos * (desired_pos - current_pos) - kv_pos * current_vel);
		command_torques = jacobian.transpose() * force + gravity_vec - N*robot->_M*(kv_joint * robot->_dq);

		/* With Null Space Damping and no Mass Matrix */
		force = (kp_pos * (desired_pos - current_pos) - kv_pos * current_vel);
		command_torques = jacobian.transpose() * force + gravity_vec - N*robot->_M*(kv_joint * robot->_dq);
		
		/* With Operational Point Gravity */
		
		force = lambda * (kp_pos * (desired_pos - current_pos) - kv_pos * current_vel) + jac_inv.transpose() * gravity_vec;
		command_torques = jacobian.transpose() * force;
		
		//------ Send torques
		redis_client.setEigenMatrixDerivedString(JOINT_TORQUES_COMMANDED_KEY, command_torques);
		controller_counter++;
		t += 0.0001;
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
