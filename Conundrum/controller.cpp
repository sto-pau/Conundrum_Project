/**
 * @file controller.cpp
 * @brief Toro controller 
 * 
 */

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "Sai2Primitives.h"

#include <iostream>
#include <string>

#include <signal.h>

// state machine
#define HOME				0
#define FIRST_MOVING		1
#define FIRST_HITTING 		2
#define MOVING_DRUMSTICK      3
#define HITTING_DRUM		4
#define LIFTING_DRUMSTICK	5
#define WAIT_FOR_MEASURE      6

bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/toro.urdf";	

enum Control
{
	JOINT_CONTROLLER = 0, 
	POSORI_CONTROLLER
};

#include "redis_keys.h"

unsigned long long controller_counter = 0;

int main() {

	int state = JOINT_CONTROLLER;
	string controller_status = "1";

	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
	robot->updateModel();	

	// prepare controller
	int dof = robot->dof();
	cout << "DOF " << dof << "\n";
	VectorXd command_torques = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// pose task for right foot 
	string control_link = "RL_foot";
	Vector3d control_point = Vector3d(0, 0, 0);
	auto posori_task_right_foot = new Sai2Primitives::PositionTask(robot, control_link, control_point); //PosOriTask(robot, control_link, control_point);

	posori_task_right_foot->_use_interpolation_flag = true;
	posori_task_right_foot->_use_velocity_saturation_flag = true;
	
	VectorXd posori_task_torques_right_foot = VectorXd::Zero(dof);
	posori_task_right_foot->_kp = 400.0;
	posori_task_right_foot->_kv = 20.0;
	// posori_task_right_foot->_kp_ori = 400.0;
	// posori_task_right_foot->_kv_ori = 20.0;

	// set desired position and orientation to the initial configuration
	Vector3d x_pos;
	robot->positionInWorld(x_pos, control_link, control_point);
	Matrix3d x_ori;
	robot->rotationInWorld(x_ori, control_link);
	posori_task_right_foot->_desired_position = x_pos; //Vector3d(0.8, 0.002, -1.15); //x_pos;
	//posori_task_right_foot->_desired_orientation = x_ori; 

	// pose task for left foot 
	control_link = "LL_foot";
	control_point = Vector3d(0, 0, 0);
	auto posori_task_left_foot = new Sai2Primitives::PositionTask(robot, control_link, control_point); //PosOriTask(robot, control_link, control_point);

	posori_task_left_foot->_use_interpolation_flag = true;
	posori_task_left_foot->_use_velocity_saturation_flag = true;

	VectorXd posori_task_torques_left_foot = VectorXd::Zero(dof);
	posori_task_left_foot->_kp = 400.0;
	posori_task_left_foot->_kv = 20.0;
	// posori_task_left_foot->_kp_ori = 400.0;
	// posori_task_left_foot->_kv_ori = 20.0;

	// set desired position and orientation to the initial configuration
	robot->positionInWorld(x_pos, control_link, control_point);
	robot->rotationInWorld(x_ori, control_link);
	posori_task_left_foot->_desired_position = x_pos;
	//posori_task_left_foot->_desired_orientation = x_ori; 

	// pose task for right hand 
	control_link = "ra_end_effector"; //length is 0.214374
	control_point = Vector3d(0, 0.214374, 0);
	auto posori_task_right_hand = new Sai2Primitives::PositionTask(robot, control_link, control_point); //PosOriTask(robot, control_link, control_point);

	posori_task_right_hand->_use_interpolation_flag = true;
	posori_task_right_hand->_use_velocity_saturation_flag = true;
	
	VectorXd posori_task_torques_right_hand = VectorXd::Zero(dof);
	posori_task_right_hand->_kp = 200.0;
	posori_task_right_hand->_kv = 20.0;
	// posori_task_right_hand->_kp_ori = 200.0;
	// posori_task_right_hand->_kv_ori = 20.0;

	// set two goal positions/orientations 
	robot->positionInWorld(x_pos, control_link, control_point);
	robot->rotationInWorld(x_ori, control_link);
	posori_task_right_hand->_desired_position = x_pos + Vector3d(0.1, -0.05, 0.3);//Vector3d(0.5, -0.2, 0.8);
	//posori_task_right_hand->_desired_orientation = x_ori;// * AngleAxisd(-M_PI/4, Vector3d::UnitY()).toRotationMatrix() * AngleAxisd(0 * M_PI/4, Vector3d::UnitZ()).toRotationMatrix(); //AngleAxisd(-3*M_PI/4, Vector3d::UnitY()).toRotationMatrix() * AngleAxisd(0 * M_PI/4, Vector3d::UnitZ()).toRotationMatrix() * x_ori; 
	// posori_task_right_hand->_desired_orientation = AngleAxisd(M_PI/2, Vector3d::UnitX()).toRotationMatrix() * \
	// 											AngleAxisd(-M_PI/2, Vector3d::UnitY()).toRotationMatrix() * x_ori; 

	// pose task for left hand
	control_link = "la_end_effector";
	control_point = Vector3d(0, 0, -0.214); //length is 0.214374
	auto posori_task_left_hand = new Sai2Primitives::PosOriTask(robot, control_link, control_point); //PosOriTask(robot, control_link, control_point);
	//posori_task_left_hand->setDynamicDecouplingFull(); //only one with this William said to remove

	// set two goal positions/orientations 
	robot->positionInWorld(x_pos, control_link, control_point);
	robot->rotationInWorld(x_ori, control_link);
	posori_task_left_hand->_desired_position = x_pos;
	posori_task_left_hand->_desired_orientation = x_ori;

	posori_task_left_hand->_use_interpolation_flag = false;
	posori_task_left_hand->_use_velocity_saturation_flag = true;
	
	VectorXd posori_task_torques_left_hand = VectorXd::Zero(dof);
	posori_task_left_hand->_kp_pos = 200.0;
	posori_task_left_hand->_kv_pos = 20.0;
	posori_task_left_hand->_kp_ori = 200.0;
	posori_task_left_hand->_kv_ori = 20.0;

	// set two goal positions/orientations 
	robot->positionInWorld(x_pos, control_link, control_point);
	robot->rotationInWorld(x_ori, control_link);
	posori_task_left_hand->_desired_position = x_pos; //Vector3d(0.92103,0.18308,-0.41312); //x_pos + Vector3d(0.1, 0.05, 0.3);//Vector3d(0.5, -0.2, 0.8);
	posori_task_left_hand->_desired_orientation = x_ori; //* AngleAxisd(-M_PI/4, Vector3d::UnitY()).toRotationMatrix() * AngleAxisd(0 * M_PI/4, Vector3d::UnitZ()).toRotationMatrix(); //AngleAxisd(-3*M_PI/4, Vector3d::UnitY()).toRotationMatrix() * AngleAxisd(0 * M_PI/4, Vector3d::UnitZ()).toRotationMatrix() * x_ori; 
	// posori_task_right_hand->_desired_orientation = AngleAxisd(M_PI/2, Vector3d::UnitX()).toRotationMatrix() * \
	// 											AngleAxisd(-M_PI/2, Vector3d::UnitY()).toRotationMatrix() * x_ori; 

	// pose task for hip_base 
	control_link = "hip_base";
	control_point = Vector3d(0, 0, 0);
	auto posori_task_torso = new Sai2Primitives::PositionTask(robot, control_link, control_point); //PosOriTask(robot, control_link, control_point);

	posori_task_torso->_use_interpolation_flag = true;
	posori_task_torso->_use_velocity_saturation_flag = true;

	VectorXd posori_task_torques_torso = VectorXd::Zero(dof);
	posori_task_torso->_kp = 400.0;
	posori_task_torso->_kv = 20.0;
	// posori_task_torso->_kp_ori = 400.0;
	// posori_task_torso->_kv_ori = 20.0;

	// set desired position and orientation to the initial configuration
	robot->positionInWorld(x_pos, control_link, control_point);
	robot->rotationInWorld(x_ori, control_link);
	posori_task_torso->_desired_position = x_pos;
	//posori_task_torso->_desired_orientation = x_ori; 

	// joint task
	auto joint_task = new Sai2Primitives::JointTask(robot);

	joint_task->_use_interpolation_flag = true;
	joint_task->_use_velocity_saturation_flag = true;

	VectorXd joint_task_torques = VectorXd::Zero(dof);
	joint_task->_kp = 100.0;
	joint_task->_kv = 20.0;

	// set desired joint posture to be the initial robot configuration
	VectorXd q_init_desired = robot->_q;
	//q_init_desired(11) = 0; 
	//cout << "ANGLE " << q_init_desired(11);
	joint_task->_desired_position = q_init_desired;

	// setup redis callback
	redis_client.createReadCallback(0);
	redis_client.createWriteCallback(0);

	// add to read callback
	redis_client.addEigenToReadCallback(0, JOINT_ANGLES_KEY, robot->_q);
	redis_client.addEigenToReadCallback(0, JOINT_VELOCITIES_KEY, robot->_dq);

	// add to write callback
	redis_client.addStringToWriteCallback(0, CONTROLLER_RUNNING_KEY, controller_status);
	redis_client.addEigenToWriteCallback(0, JOINT_TORQUES_COMMANDED_KEY, command_torques);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;
	
	// for state machine
	int i;
	int bpm = 4;
	Vector3d pos_des; 
	Vector3d curr_pos;
	bool play = true;
	unsigned drum_state = HOME;
	double t_buffer = 0.1;
	double start;
	double dz = 0.2;
	double threshold = 0.01;
	double v_hit = 1.0;
	double v_travel = 0.3;
	double time_to_hit = dz/v_hit;
	
	VectorXd time_data(4), x_data(4), y_data(4), z_data(4); 
	Vector3d snare = Vector3d(0.36543, 0.32512, -0.50876);
	Vector3d tom1 = Vector3d(0.72103, 0.18308, -0.35312);
	Vector3d tom2 = Vector3d(0.74235, -0.18308, -0.26023);
	//data << 0,0.365,0.325,-0.53876, 15,0.74235,-0.18022,-0.30023, 30,0.72103,0.18308,-0.41312, 45,0.365,0.325,-0.53876;
	time_data << 0, 15, 30, 45;
	x_data << snare(0), tom1(0), tom2(0), snare(0);
	y_data << snare(1), tom1(1), tom2(1), snare(1);
	z_data << snare(2), tom1(2), tom2(2), snare(2);
	
	string left_arm_control_link = "la_end_effector";
	Vector3d left_arm_control_point = Vector3d(0, 0, -0.214); //length is 0.214374
	
	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// execute redis read callback
		redis_client.executeReadCallback(0);

		// update model
		robot->updateModel();
		
		robot->positionInWorld(curr_pos, left_arm_control_link, left_arm_control_point); //get curr pos
		
		//cout << "DRUM STATE: " << drum_state << "\n";
		
		switch(drum_state){
			case HOME:
				if (play = true){
					i = 0;
					pos_des << x_data(i), y_data(i), z_data(i);
					pos_des(2) += dz;  //at vtravel
					posori_task_left_hand->_linear_saturation_velocity = v_travel;
					cout << pos_des << "\n";
					drum_state = FIRST_MOVING;
					cout << "DRUM STATE: " << drum_state << "\n";
				}
				break;
			case FIRST_MOVING:
				if (time - start >= time_data(i)-time_to_hit-t_buffer){
					cout << time - start << "\n";
					pos_des << x_data(i), y_data(i), z_data(i); //at vhit
					posori_task_left_hand->_linear_saturation_velocity = v_hit;
					drum_state = HITTING_DRUM;
					cout << "DRUM STATE: " << drum_state << "\n";
				}
				break;
			case FIRST_HITTING:
				if (abs(curr_pos.norm()-pos_des.norm()) < threshold){
					pos_des << x_data(i), y_data(i), z_data(i); 
					pos_des(2) += dz;		//at vtravel
					posori_task_left_hand->_linear_saturation_velocity = v_travel;
					start = time;
					drum_state = LIFTING_DRUMSTICK;
					cout << "DRUM STATE: " << drum_state << "\n";
				}
				break;
			case LIFTING_DRUMSTICK:
				if (abs(curr_pos.norm()-pos_des.norm()) < threshold){
					i++;
					if (i%bpm == 0){
						i = 0;
						drum_state = WAIT_FOR_MEASURE;
					}
					else{
						pos_des << x_data(i), y_data(i), z_data(i);
						pos_des(2) += dz;      //at vtravel
						posori_task_left_hand->_linear_saturation_velocity = v_travel;
						drum_state = MOVING_DRUMSTICK;
						cout << "DRUM STATE: " << drum_state << "\n";
					}
				}
				break;
			case MOVING_DRUMSTICK:
				if (time - start >= time_data(i)-time_to_hit-t_buffer){
					cout << time - start << "\n";
					pos_des << x_data(i), y_data(i), z_data(i);   //at vhit
					posori_task_left_hand->_linear_saturation_velocity = v_hit;
					drum_state = HITTING_DRUM;
					cout << "DRUM STATE: " << drum_state << "\n";
				}
				break;
			case HITTING_DRUM:
				if (abs(curr_pos.norm()-pos_des.norm()) < threshold){
					pos_des << x_data(i), y_data(i), z_data(i);
					pos_des(2) += dz;      //at vtravel
					posori_task_left_hand->_linear_saturation_velocity = v_travel;
					drum_state = LIFTING_DRUMSTICK;
					cout << "DRUM STATE: " << drum_state << "\n";
				}
				break;
			case WAIT_FOR_MEASURE:
				if (time-start >= 60){
					start = time;
					drum_state = MOVING_DRUMSTICK;
				}
				break;
			default:
				break;
		}
		
		posori_task_left_hand->_desired_position = pos_des;  //set position desired
		
		// calculate torques to maintain hip_base posture
		N_prec.setIdentity();		
		posori_task_torso->updateTaskModel(N_prec);
		posori_task_torso->computeTorques(posori_task_torques_torso);
		
		// calculate torques to fix the feet 
		N_prec = posori_task_torso->_N;
		posori_task_right_foot->updateTaskModel(N_prec);
		posori_task_right_foot->computeTorques(posori_task_torques_right_foot);

		N_prec = posori_task_right_foot->_N;
		posori_task_left_foot->updateTaskModel(N_prec);
		posori_task_left_foot->computeTorques(posori_task_torques_left_foot);

		// calculate torques to move right hand
		N_prec = posori_task_left_foot->_N;
		posori_task_right_hand->updateTaskModel(N_prec);
		posori_task_right_hand->computeTorques(posori_task_torques_right_hand);

		// calculate torques to move left hand
		N_prec = posori_task_right_hand->_N;
		posori_task_left_hand->updateTaskModel(N_prec);
		posori_task_left_hand->computeTorques(posori_task_torques_left_hand);

		// calculate torques to maintain joint posture
		N_prec = posori_task_left_hand->_N;
		joint_task->updateTaskModel(N_prec);
		joint_task->computeTorques(joint_task_torques);

		// calculate torques 
		command_torques = posori_task_torques_right_foot + posori_task_torques_left_foot 
							+ posori_task_torques_right_hand + posori_task_torques_left_hand + posori_task_torques_torso + joint_task_torques;//posori_task_torques_torso + joint_task_torques;  // gravity compensation handled in sim

		// execute redis write callback
		redis_client.executeWriteCallback(0);	

		controller_counter++;
	}

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, 0 * command_torques);

	return 0;
}
