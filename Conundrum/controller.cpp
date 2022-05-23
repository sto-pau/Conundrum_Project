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
#include <fstream>
#include <signal.h>
using namespace std;
// state machine
#define HOME				0
#define FIRST_MOVING		1
#define FIRST_HITTING 		2
#define MOVING_DRUMSTICK      3
#define HITTING_DRUM		4
#define LIFTING_DRUMSTICK	5
#define WAIT_FOR_MEASURE      6
#define WAIT     			 7
#define STOMP     			 8

bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;

/*Module level function to read GUI output file and store time,x,y,z data*/

void readGUI(string filename, float s[4][10], int& no_tsteps);

const string robot_file = "./resources/toro.urdf";	

const std::string DRUM_KEY = "drum::key";

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
	control_point = Vector3d(0, 0, -0.214);
	auto posori_task_right_hand = new Sai2Primitives::PosOriTask(robot, control_link, control_point); //PosOriTask(robot, control_link, control_point);

	posori_task_right_hand->_use_interpolation_flag = true;
	posori_task_right_hand->_use_velocity_saturation_flag = true;
	
	VectorXd posori_task_torques_right_hand = VectorXd::Zero(dof);
	posori_task_right_hand->_kp_pos = 200.0;
	posori_task_right_hand->_kv_pos = 20.0;
	posori_task_right_hand->_kp_ori = 200.0;
	posori_task_right_hand->_kv_ori = 20.0;

	// set two goal positions/orientations 
	robot->positionInWorld(x_pos, control_link, control_point);
	robot->rotationInWorld(x_ori, control_link);
	posori_task_right_hand->_desired_position = x_pos; //Vector3d(0.93609, -0.05134, -0.4536);//Vector3d(0.5, -0.2, 0.8);
	posori_task_right_hand->_desired_orientation = x_ori;// * AngleAxisd(-M_PI/4, Vector3d::UnitY()).toRotationMatrix() * AngleAxisd(0 * M_PI/4, Vector3d::UnitZ()).toRotationMatrix(); //AngleAxisd(-3*M_PI/4, Vector3d::UnitY()).toRotationMatrix() * AngleAxisd(0 * M_PI/4, Vector3d::UnitZ()).toRotationMatrix() * x_ori; 
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

	//joint_task->_use_interpolation_flag = true;
	joint_task->_use_velocity_saturation_flag = true;
	Eigen::VectorXd w = M_PI/1.0*Eigen::VectorXd::Ones(dof);
	joint_task->_saturation_velocity = w;

	VectorXd joint_task_torques = VectorXd::Zero(dof);
	joint_task->_kp = 100.0;
	joint_task->_kv = 20.0;

	// set desired joint posture to be the initial robot configuration
	VectorXd q_init_desired = robot->_q;
	//q_init_desired(11) = 0; 
	//cout << "ANGLES " << robot->_q;
	VectorXd joint_desired = q_init_desired;
	joint_task->_desired_position = joint_desired;

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
	
	// for timing
	double unified_start_time = start_time + 5;
	bool startedPlaying = false;
	bool play = true;
	double start;

	//location of the instruments
	Vector3d snare = Vector3d(0.36543, 0.32512, -0.50876);
	Vector3d tom1 = Vector3d(0.72103, 0.18308, -0.35312);
	Vector3d tom2 = Vector3d(0.74235, -0.18308, -0.26023);

	//for both arm state machines
	double t_buffer = 0.7;	
	double dz = 0.2;
	double threshold = 0.01;
	double v_hit = 1.0;
	double v_travel = 0.3;
	double time_to_hit = dz/v_hit;
	
	//left arm state machine
	int i;
	Vector3d pos_des; 
	Vector3d curr_pos;	
	unsigned LH_state = HOME;	
	
	string left_arm_control_link = "la_end_effector";
	Vector3d left_arm_control_point = Vector3d(0, 0, -0.214); //length is 0.214374

	//for right arm state machine
	int index_ra;
	Vector3d pos_des_ra; 
	Vector3d curr_pos_ra;	
	unsigned RH_state = HOME;	
	
	string right_arm_control_link = "ra_end_effector";
	Vector3d right_arm_control_point = Vector3d(0, 0, -0.214); //length is 0.21437

	//for both leg state machines
	double thetaThreshold = 0.1; //apprx 5.73 deg
	double t_stomp_buffer = 0.5;
	double dTh = M_PI/12;
	double time_to_stomp = dTh / w[0]; //w declared above when JointTask sat_vel is declared
	
	//for left leg state machine (HiHat)
	int LF_joint = 17; //18th joint, indexed with 17
	unsigned LF_state = HOME;
	int index_LF;
	float ang_LF_des = q_init_desired[LF_joint]; //initialize with starting value to stay there at home (otherwise garbage angle)
	float curr_LF_ang;	
	double LF_lift = q_init_desired[LF_joint];
	double LF_stomp = LF_lift + dTh;

	//for right leg state machine (Bass)
	int RF_joint = 11; //12th joint, indexed with 11
	unsigned RF_state = HOME;
	int index_RF;
	float ang_RF_des = q_init_desired[RF_joint]; //initialize with starting value to stay there at home (otherwise garbage angle)
	float curr_RF_ang;	
	double RF_lift = q_init_desired[RF_joint];
	double RF_stomp = RF_lift + dTh;	
	
	//Wait for play button to be hit
	redis_client.set("gui::is_playing","0");
	while(!stoi(redis_client.get("gui::is_playing"))){}
	
	
	/**************START OF GUI FILE-READ******************/

	//Right hand
	int no_tsteps_rh; float rh[4][10];
	readGUI("right_hand.txt", rh, no_tsteps_rh);	
	cout << "TIME STEPS RH: " << no_tsteps_rh << "\n"; 
	VectorXd time_data_rh(no_tsteps_rh), x_data_rh(no_tsteps_rh), y_data_rh(no_tsteps_rh), z_data_rh(no_tsteps_rh);
	//Store time,x,y,z data
	for(int ct = 0; ct<no_tsteps_rh;ct++){

		time_data_rh(ct) = rh[0][ct];
		x_data_rh(ct) = rh[1][ct];
		y_data_rh(ct) = rh[2][ct];
		z_data_rh(ct) = rh[3][ct];
	
	}
	
	//Left hand
	int no_tsteps_lh; float lh[4][10];
	readGUI("left_hand.txt", lh, no_tsteps_lh);
	cout << "TIME STEPS LH: " << no_tsteps_lh << "\n"; 	
	VectorXd time_data_lh(no_tsteps_lh), x_data_lh(no_tsteps_lh), y_data_lh(no_tsteps_lh), z_data_lh(no_tsteps_lh);
	//Store time,x,y,z data
	for(int ct = 0; ct<no_tsteps_lh;ct++){

		time_data_lh(ct) = lh[0][ct];
		x_data_lh(ct) = lh[1][ct];
		y_data_lh(ct) = lh[2][ct];
		z_data_lh(ct) = lh[3][ct];

	}	

	//Right foot
	int no_tsteps_rf; float rf[4][10];
	readGUI("right_foot.txt", rf, no_tsteps_rf);
	cout << "TIME STEPS RF: " << no_tsteps_rf << "\n"; 	
	VectorXd time_data_rf(no_tsteps_rf), x_data_rf(no_tsteps_rf), y_data_rf(no_tsteps_rf), z_data_rf(no_tsteps_rf);
	//Store time,x,y,z data
	for(int ct = 0; ct<no_tsteps_rf;ct++){
		time_data_rf(ct) = rf[0][ct];
		x_data_rf(ct) = rf[1][ct];
		y_data_rf(ct) = rf[2][ct];
		z_data_rf(ct) = rf[3][ct];
	} 	

	//Left Foot
	int no_tsteps_lf; float lf[4][10];
	readGUI("left_foot.txt", lf, no_tsteps_lf);	
	cout << "TIME STEPS LF: " << no_tsteps_lf << "\n"; 
	VectorXd time_data_lf(no_tsteps_lf), x_data_lf(no_tsteps_lf), y_data_lf(no_tsteps_lf), z_data_lf(no_tsteps_lf);
	//Store time,x,y,z data
	for(int ct = 0; ct<no_tsteps_lf;ct++){
		time_data_lf(ct) = lf[0][ct];
		x_data_lf(ct) = lf[1][ct];
		y_data_lf(ct) = lf[2][ct];
		z_data_lf(ct) = lf[3][ct];
	}


/*******END OF GUI FILE-READ*********************/

/***START OF STATE MACHINE***************/

	while (runloop) {	
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		if( time >= unified_start_time && startedPlaying == false) { //synchronized start at unified start time
			start = time;
			startedPlaying = true;
		}

		// execute redis read callback
		redis_client.executeReadCallback(0);

		// update model
		robot->updateModel();
		
		robot->positionInWorld(curr_pos, left_arm_control_link, left_arm_control_point); //get curr pos left arm
		robot->positionInWorld(curr_pos_ra, right_arm_control_link, right_arm_control_point); //get curr pos right arm
		curr_LF_ang = robot->_q[LF_joint]; //get current left foot angle
		curr_RF_ang = robot->_q[RF_joint]; //get current right foot angle
		
		
		if (no_tsteps_lh != 0){
			switch(LH_state){
				case HOME:
					if (play == true){
						i = 0;
						pos_des << x_data_lh(i), y_data_lh(i), z_data_lh(i);
						pos_des(2) += dz;  //at vtravel
						posori_task_left_hand->_linear_saturation_velocity = v_travel;
						//cout << pos_des << "\n";
						LH_state = FIRST_MOVING;
						cout << "DRUM STATE: " << LH_state << "\n";
					} 
					break;
				case FIRST_MOVING:
					if (time >= unified_start_time + time_data_lh(i) - time_to_hit - t_buffer){ //move in anticipation to the synchronized start before 'start' has been set
						
						pos_des << x_data_lh(i), y_data_lh(i), z_data_lh(i); //at vhit
						posori_task_left_hand->_linear_saturation_velocity = v_hit;
						LH_state = HITTING_DRUM;
						cout << "DRUM STATE: " << LH_state << "\n";
					}
					break;
				case LIFTING_DRUMSTICK:
					if (abs(curr_pos.norm()-pos_des.norm()) < threshold){
						i++;
						if (i % no_tsteps_lh == 0){
							i = 0;
							LH_state = MOVING_DRUMSTICK;
							Eigen::VectorXd addTime = 60 * Eigen::VectorXd::Ones(no_tsteps_lh);
							time_data_lh = time_data_lh + addTime; //wrap time around by adding the length of one time measure (one song meter)

							//these desired pos assignments HAVE to come after setting i = 0!
							pos_des << x_data_lh(i), y_data_lh(i), z_data_lh(i);
							pos_des(2) += dz;  //at vtravel
							posori_task_left_hand->_linear_saturation_velocity = v_travel;

						}
						else{
							pos_des << x_data_lh(i), y_data_lh(i), z_data_lh(i);
							pos_des(2) += dz;      //at vtravel
							posori_task_left_hand->_linear_saturation_velocity = v_travel;
							LH_state = MOVING_DRUMSTICK;
							cout << "DRUM STATE: " << LH_state << "\n";
						}
					}
					break;
				case MOVING_DRUMSTICK:
					
					if (time - start >= time_data_lh(i)-time_to_hit-t_buffer){
						cout << "\nstarting hit ra " << time - start << "\n";
						pos_des << x_data_lh(i), y_data_lh(i), z_data_lh(i);   //at vhit
						posori_task_left_hand->_linear_saturation_velocity = v_hit;
						LH_state = HITTING_DRUM;
						cout << "DRUM STATE: " << LH_state << "\n";
					}
					break;
				case HITTING_DRUM:
					if (abs(curr_pos.norm()-pos_des.norm()) < threshold){
						if ((float)pos_des(0) == (float)snare(0)){
							redis_client.set(DRUM_KEY, "1");
							cout << "SNARE" << "\n";
						}
						if ((float)pos_des(0) == (float)tom1(0)){
							redis_client.set(DRUM_KEY, "2");
							cout << "TOM1" << "\n";
						}
						if ((float)pos_des(0) == (float)tom2(0)){
							redis_client.set(DRUM_KEY, "3");
							cout << "TOM2" << "\n";
						}
						cout  << "TIME AT DRUM HIT: " << time - start << "\n";
						pos_des << x_data_lh(i), y_data_lh(i), z_data_lh(i);
						pos_des(2) += dz;      //at vtravel
						posori_task_left_hand->_linear_saturation_velocity = v_travel;
						LH_state = LIFTING_DRUMSTICK;
						cout << "DRUM STATE: " << LH_state << "\n";
					}
					break;
				default:
					break;
			}
			
			posori_task_left_hand->_desired_position = pos_des;  //set position desired for left arm
		}

		if (no_tsteps_rh != 0){
			switch(RH_state){
				case HOME:
					if (play == true){
						index_ra = 0;
						pos_des_ra << x_data_rh(index_ra), y_data_rh(index_ra), z_data_rh(index_ra);
						pos_des_ra(2) += dz;  //at vtravel
						posori_task_right_hand->_linear_saturation_velocity = v_travel;
						//cout << pos_des_ra << "\n";
						RH_state = FIRST_MOVING;
						cout << "DRUM STATE RH: " << RH_state << "\n";
					} 
					break;
				case FIRST_MOVING:
					if (time >= unified_start_time + time_data_rh(index_ra) - time_to_hit - t_buffer){ //move in anticipation to the synchronized start before 'start' has been set
						
						pos_des_ra << x_data_rh(index_ra), y_data_rh(index_ra), z_data_rh(index_ra); //at vhit
						posori_task_right_hand->_linear_saturation_velocity = v_hit;
						RH_state = HITTING_DRUM;
						cout << "DRUM STATE RH: " << RH_state << "\n";
					}
					break;
				case LIFTING_DRUMSTICK:
					if (abs(curr_pos_ra.norm()-pos_des_ra.norm()) < threshold){
						index_ra++;
						if (index_ra % no_tsteps_rh == 0){
							index_ra = 0;
							RH_state = MOVING_DRUMSTICK;
							Eigen::VectorXd addTime = 60 * Eigen::VectorXd::Ones(no_tsteps_rh);
							time_data_rh = time_data_rh + addTime; //wrap time around by adding the length of one time measure (one song meter)

							//these desired pos assignments HAVE to come after setting i = 0!
							pos_des_ra << x_data_rh(index_ra), y_data_rh(index_ra), z_data_rh(index_ra);
							pos_des_ra(2) += dz;  //at vtravel
							posori_task_right_hand->_linear_saturation_velocity = v_travel;

						}
						else{
							pos_des_ra << x_data_rh(index_ra), y_data_rh(index_ra), z_data_rh(index_ra);
							pos_des_ra(2) += dz;      //at vtravel
							posori_task_right_hand->_linear_saturation_velocity = v_travel;
							RH_state = MOVING_DRUMSTICK;
							cout << "DRUM STATE RH: " << RH_state << "\n";
						}
					}
					break;
				case MOVING_DRUMSTICK:
					
					if (time - start >= time_data_rh(index_ra)-time_to_hit-t_buffer){
						cout << "\nstarting hit ra " << time - start << "\n";
						pos_des_ra << x_data_rh(index_ra), y_data_rh(index_ra), z_data_rh(index_ra);   //at vhit
						posori_task_right_hand->_linear_saturation_velocity = v_hit;
						RH_state = HITTING_DRUM;
						cout << "DRUM STATE RH: " << RH_state << "\n";
					}
					break;
				case HITTING_DRUM:
					if (abs(curr_pos_ra.norm()-pos_des_ra.norm()) < threshold){
						if ((float)pos_des_ra(0) == (float)snare(0)){
							redis_client.set(DRUM_KEY, "1");
							cout << "SNARE" << "\n";
						}
						if ((float)pos_des_ra(0) == (float)tom1(0)){
							redis_client.set(DRUM_KEY, "2");
							cout << "TOM1" << "\n";
						}
						if ((float)pos_des_ra(0) == (float)tom2(0)){
							redis_client.set(DRUM_KEY, "3");
							cout << "TOM2" << "\n";
						}
						cout  << "TIME AT DRUM HIT: " << time - start << "\n";
						pos_des_ra << x_data_rh(i), y_data_rh(i), z_data_rh(i);
						pos_des_ra(2) += dz;      //at vtravel
						posori_task_right_hand->_linear_saturation_velocity = v_travel;
						RH_state = LIFTING_DRUMSTICK;
						cout << "DRUM STATE RH: " << RH_state << "\n";
					}
					break;
				default:
					break;
			}
			
			posori_task_right_hand->_desired_position = pos_des_ra;  //set position desired for right arm
		}

		if (no_tsteps_lf != 0){
			switch(LF_state){ //left leg (hi-hat) state machine
				case HOME:
					if (play == true){
						index_LF = 0;
						LF_state = FIRST_MOVING;
						cout << "LEFT LEG STATE: " << LF_state << "\n";
					}
					break;
				case FIRST_MOVING:
					if (time >= unified_start_time + time_data_lf(index_LF) - time_to_stomp - t_stomp_buffer){
						ang_LF_des = LF_stomp;
						LF_state = STOMP;
						cout << "LEFT LEG  STATE: " << LF_state << "\n";
					}
					break;
				case WAIT:
					if (time - start >= time_data_lf(index_LF) - time_to_stomp - t_stomp_buffer){
						cout << "\nstarting stomp LL " << time - start << "\n";
						ang_LF_des = LF_stomp;
						LF_state = STOMP;
						cout << "LEFT LEG  STATE: " << LF_state << "\n";
					}
					break;
				case STOMP:
					if (abs(curr_LF_ang - ang_LF_des) < thetaThreshold){
						redis_client.set(DRUM_KEY, "4");
						index_LF++;
						ang_LF_des = LF_lift;
						LF_state = WAIT;
						cout << "TIME AT HIHAT HIT: " << time - start << "\n";
						if (index_LF % no_tsteps_lf == 0){
							index_LF = 0;
							Eigen::VectorXd addLLTime = 60 * Eigen::VectorXd::Ones(no_tsteps_lf);
							time_data_lf = time_data_lf + addLLTime;
						}
						cout << "LEFT LEG  STATE: " << LF_state << "\n";
					}
					break;
				default:
					break;
			}//end left leg (hi-hat) state machine
			
			joint_desired[LF_joint] = ang_LF_des; //set desired LL position
		}
		
		if (no_tsteps_rf != 0){
			switch(RF_state){ //right leg (bass) state machine
				case HOME:
					if (play == true){
						index_RF = 0;
						RF_state = FIRST_MOVING;
						cout << "RIGHT LEG  STATE: " << RF_state << "\n";
					}
					break;
				case FIRST_MOVING:
					if (time >= unified_start_time + time_data_rf(index_RF) - time_to_stomp - t_stomp_buffer){
						ang_RF_des = RF_stomp;
						RF_state = STOMP;
						cout << "RIGHT LEG  STATE: " << RF_state << "\n";
					}
					break;
				case WAIT:
					if (time - start >= time_data_rf(index_RF) - time_to_stomp - t_stomp_buffer){
						cout << "\nstarting stomp LL " << time - start << "\n";
						ang_RF_des = RF_stomp;
						RF_state = STOMP;
						cout << "RIGHT LEG  STATE: " << RF_state << "\n";
					}
					break;
				case STOMP:
					if (abs(curr_RF_ang - ang_RF_des) < thetaThreshold){
						redis_client.set(DRUM_KEY, "5");
						index_RF++;
						ang_RF_des = RF_lift;
						RF_state = WAIT;
						cout << "TIME AT BASS HIT: " << time - start << "\n";
						if (index_RF % no_tsteps_rf == 0){
							index_RF = 0;
							Eigen::VectorXd addLLTime = 60 * Eigen::VectorXd::Ones(no_tsteps_rf);
							time_data_rf = time_data_rf + addLLTime;
						}
						cout << "RIGHT LEG  STATE: " << RF_state << "\n";
					}
					break;
				default:
					break;
			}//end right leg (bass) state machine
			
			joint_desired[RF_joint] = ang_RF_des; //set desired LL position
		}

		joint_task->_desired_position = joint_desired;
		
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


/*Module level function to read GUI output file and store time,x,y,z data*/

void readGUI(string filename, float s[4][10], int& no_tsteps){

	//Set up file read from GUI
	ifstream myfile;
	myfile.open (filename, ios::in);
	
	//array to store initial read from file
	string line;
	// i counts number of timesteps
	// j iterates across time,x,y,z of each timestep
	int i = 0; int j=0;
	//Read elements from the file
	while ( getline (myfile,line) )
	{
		s[j][i] = stof(line);
		j++;
		//If we're done reading one vector
		if(j==4){
			//Move to next timestep
			j=0; i++;
		}

	}
	//Find number of timesteps
	no_tsteps = i;

	myfile.close();


}
