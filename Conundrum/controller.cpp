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
#define START     			 9
#define NODDING     		 10

bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;

/*Module level function to read GUI output file and store time,x,y,z data*/

void readGUI(string filename, float s[4][10], int& no_tsteps);

const string robot_file = "./resources/toro.urdf";	

const std::string DRUM_KEY = "drum::key";

const std::string BASS_KEY = "drum::bass";
const std::string HIHAT_KEY = "drum::hihat";
const std::string TOM1_KEY = "drum::tom1";
const std::string TOM2_KEY = "drum::tom2";
const std::string SNARE_KEY = "drum::snare";


enum Control
{
	JOINT_CONTROLLER = 0, 
	POSORI_CONTROLLER
};

#include "redis_keys.h"

unsigned long long controller_counter = 0;

int main() {

	ofstream data_file;
	data_file.open("hitTime.csv");

	ofstream data_filera;
	data_filera.open("hitTimera.csv");

	ofstream data_filelf;
	data_filelf.open("hitTimelf.csv");

	ofstream data_filerf;
	data_filerf.open("hitTimerf.csv");


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
	VectorXd command_torques = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// pose task for right hand 
	string control_link = "ra_end_effector"; //length is 0.214374
	Vector3d control_point = Vector3d(0, 0, -0.214374);
	auto posori_task_right_hand = new Sai2Primitives::PosOriTask(robot, control_link, control_point); //PosOriTask(robot, control_link, control_point);

	posori_task_right_hand->_use_interpolation_flag = true;
	posori_task_right_hand->_use_velocity_saturation_flag = false;
	
	VectorXd posori_task_torques_right_hand = VectorXd::Zero(dof);
	posori_task_right_hand->_kp_pos = 400.0;
	posori_task_right_hand->_kv_pos = 20.0;
	posori_task_right_hand->_kp_ori = 200.0;
	posori_task_right_hand->_kv_ori = 20.0;

	// set two goal positions/orientations 
	Vector3d x_pos;
	Matrix3d x_ori;
	Vector3d rh_init_pos;
	robot->positionInWorld(rh_init_pos, control_link, control_point);
	//rh_init_pos = x_pos;
	robot->rotationInWorld(x_ori, control_link);
	posori_task_right_hand->_desired_position = rh_init_pos; //Vector3d(0.93609, -0.05134, -0.4536);//Vector3d(0.5, -0.2, 0.8);
	posori_task_right_hand->_desired_orientation = x_ori;// * AngleAxisd(-M_PI/4, Vector3d::UnitY()).toRotationMatrix() * AngleAxisd(0 * M_PI/4, Vector3d::UnitZ()).toRotationMatrix(); //AngleAxisd(-3*M_PI/4, Vector3d::UnitY()).toRotationMatrix() * AngleAxisd(0 * M_PI/4, Vector3d::UnitZ()).toRotationMatrix() * x_ori; 
	// posori_task_right_hand->_desired_orientation = AngleAxisd(M_PI/2, Vector3d::UnitX()).toRotationMatrix() * \
	// 											AngleAxisd(-M_PI/2, Vector3d::UnitY()).toRotationMatrix() * x_ori; 

	posori_task_right_hand->setDynamicDecouplingFull();

	// pose task for left hand
	control_link = "la_end_effector";
	control_point = Vector3d(0, 0, -0.214374); //length is 0.214374
	auto posori_task_left_hand = new Sai2Primitives::PosOriTask(robot, control_link, control_point); //PosOriTask(robot, control_link, control_point);
	//posori_task_left_hand->setDynamicDecouplingFull(); //only one with this William said to remove

	// set two goal positions/orientations 
	//robot->positionInWorld(x_pos, control_link, control_point);
	//robot->rotationInWorld(x_ori, control_link);
	//posori_task_left_hand->_desired_position = x_pos;
	//posori_task_left_hand->_desired_orientation = x_ori;

	posori_task_left_hand->_use_interpolation_flag = true;
	posori_task_left_hand->_use_velocity_saturation_flag = false;
	
	VectorXd posori_task_torques_left_hand = VectorXd::Zero(dof);
	posori_task_left_hand->_kp_pos = 400.0;
	posori_task_left_hand->_kv_pos = 20.0;
	posori_task_left_hand->_kp_ori = 200.0;
	posori_task_left_hand->_kv_ori = 20.0;

	// set two goal positions/orientations 
	Vector3d lh_init_pos;
	robot->positionInWorld(lh_init_pos, control_link, control_point);
	//lh_init_pos = x_pos;
	robot->rotationInWorld(x_ori, control_link);
	posori_task_left_hand->_desired_position = lh_init_pos; //Vector3d(0.92103,0.18308,-0.41312); //x_pos + Vector3d(0.1, 0.05, 0.3);//Vector3d(0.5, -0.2, 0.8);
	posori_task_left_hand->_desired_orientation = x_ori; //* AngleAxisd(-M_PI/4, Vector3d::UnitY()).toRotationMatrix() * AngleAxisd(0 * M_PI/4, Vector3d::UnitZ()).toRotationMatrix(); //AngleAxisd(-3*M_PI/4, Vector3d::UnitY()).toRotationMatrix() * AngleAxisd(0 * M_PI/4, Vector3d::UnitZ()).toRotationMatrix() * x_ori; 
	// posori_task_right_hand->_desired_orientation = AngleAxisd(M_PI/2, Vector3d::UnitX()).toRotationMatrix() * \
	// 											AngleAxisd(-M_PI/2, Vector3d::UnitY()).toRotationMatrix() * x_ori; 

	posori_task_left_hand->setDynamicDecouplingFull();

	// pose task for hip_base 
	control_link = "hip_base";
	control_point = Vector3d(0, 0, 0);
	auto posori_task_torso = new Sai2Primitives::PositionTask(robot, control_link, control_point); //PosOriTask(robot, control_link, control_point);

	posori_task_torso->_use_interpolation_flag = true;
	posori_task_torso->_use_velocity_saturation_flag = true;

	VectorXd posori_task_torques_torso = VectorXd::Zero(dof);
	posori_task_torso->_kp = 400.0;
	posori_task_torso->_kv = 20.0;

	// set desired position and orientation to the initial configuration
	robot->positionInWorld(x_pos, control_link, control_point);
	robot->rotationInWorld(x_ori, control_link);
	posori_task_torso->_desired_position = x_pos;
	//posori_task_torso->_desired_orientation = x_ori; 

	//posori_task_torso->setDynamicDecouplingFull();

	// joint task
	auto joint_task = new Sai2Primitives::JointTask(robot);

	//joint_task->_use_interpolation_flag = true;
	joint_task->_use_velocity_saturation_flag = false;
	//Eigen::VectorXd w = M_PI/3.0*Eigen::VectorXd::Ones(dof);
	//joint_task->_saturation_velocity = w;
	Eigen::VectorXd w = M_PI/3.0*Eigen::VectorXd::Ones(dof);
	joint_task->_use_interpolation_flag = true;

	VectorXd joint_task_torques = VectorXd::Zero(dof);
	joint_task->_kp = 400.0;
	joint_task->_kv = 20.0;

	// set desired joint posture to be the initial robot configuration
	VectorXd q_init_desired = robot->_q;
	VectorXd joint_desired = q_init_desired;
	joint_task->_desired_position = joint_desired;

	//head joint task

	// set desired joint posture to be the initial robot configuration
	VectorXd head_joint_desired = q_init_desired;

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
	bool fTimerDidSleep = true;

	//location of the instruments
	Vector3d snare = Vector3d(0.36543, 0.32512, -0.50876);
	Vector3d tom1 = Vector3d(0.72103, 0.18308, -0.35312);
	Vector3d tom2 = Vector3d(0.74235, -0.18308, -0.26023);

	//for both arm state machines
	double t_buffer = 0.30;	
	double dz = 0.05;
	double threshold = 0.0001;
	double v_hit = 0.5;
	double v_travel = 0.75; 
	double time_to_hit = dz/v_hit;
	posori_task_left_hand->_otg->setMaxLinearAcceleration(2 * v_travel);
	posori_task_right_hand->_otg->setMaxLinearAcceleration(2 * v_travel);

	//_otg->setMaxLinearVelocity(0.3);
	//_otg->setMaxLinearAcceleration(1.0);
	//_otg->setMaxLinearJerk(3.0);
	
	//left arm state machine
	int i;
	Vector3d pos_des = lh_init_pos; 
	Vector3d curr_pos;	
	unsigned LH_state = HOME;	
	
	string left_arm_control_link = "la_end_effector";
	Vector3d left_arm_control_point = Vector3d(0, 0, -0.214374); //length is 0.214374

	//for right arm state machine
	int index_ra;
	Vector3d pos_des_ra = rh_init_pos; 
	Vector3d curr_pos_ra;	
	unsigned RH_state = HOME;	
	
	
	string right_arm_control_link = "ra_end_effector";
	Vector3d right_arm_control_point = Vector3d(0, 0, -0.214374); //length is 0.21437

	double dTh = M_PI/18;
	
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

	//for Head BoB control
	unsigned Head_state = START;
	int Head_joint = 34;
	float ang_Head_des = 0.0;
	float amplitudeBob = (60 * M_PI / 180) / 2;
	double t_bob_buffer = 0.5;
	double time_to_bob = (3 * M_PI) / w[0]; //want bottom of head motion to correspond to beat
	double start_nod_time;
	double nod_time;

	//for both leg state machines
	double thetaThreshold = 0.01; //apprx 5.73 deg
	double t_stomp_buffer = 0.372;
	Eigen::VectorXd alpha = M_PI*Eigen::VectorXd::Ones(dof);
	Eigen::VectorXd joint_jerk = 3*M_PI*Eigen::VectorXd::Ones(dof);
	w[LF_joint] = 3*M_PI;
	w[RF_joint] = 3*M_PI;
	alpha[LF_joint] = 6*M_PI;
	alpha[RF_joint] = 6*M_PI;
	joint_jerk[LF_joint] = 12*M_PI;
	joint_jerk[RF_joint] = 12*M_PI;
	joint_task->_otg->setMaxVelocity(w);
	joint_task->_otg->setMaxAcceleration(alpha);
	joint_task->_otg->setMaxJerk(joint_jerk);

	double time_to_stomp = (dTh - thetaThreshold) / w[RF_joint]; //w declared above when JointTask sat_vel is declared
	
	// cout << "time total buffer" << t_stomp_buffer + time_to_stomp <<"\n";
	
	//Wait for play button to be hit
	redis_client.set("gui::is_playing","0");
	
	double start_time = timer.elapsedTime(); //secs	
	// cout << "START_TIME: " << start_time << "\n";
	// for timing
	double unified_start_time = 5.0;
	bool startedPlaying = false;
	bool restart = true;
	double start = 0;

	//Variables for file read
	int no_tsteps_rh,no_tsteps_lh,no_tsteps_rf,no_tsteps_lf;
	double period, bpm, measureLength;
	VectorXd time_data_rh(10), x_data_rh(10), y_data_rh(10), z_data_rh(10);
	VectorXd time_data_lh(10), x_data_lh(10), y_data_lh(10), z_data_lh(10);
	VectorXd time_data_rf(10), x_data_rf(10), y_data_rf(10), z_data_rf(10);
	VectorXd time_data_lf(10), x_data_lf(10), y_data_lf(10), z_data_lf(10);

/***START OF STATE MACHINE***************/

	while (runloop) {	

		// update model
		robot->updateModel();
		
		robot->positionInWorld(curr_pos, left_arm_control_link, left_arm_control_point); //get curr pos left arm
		robot->positionInWorld(curr_pos_ra, right_arm_control_link, right_arm_control_point); //get curr pos right arm
		curr_LF_ang = robot->_q[LF_joint]; //get current left foot angle
		curr_RF_ang = robot->_q[RF_joint]; //get current right foot angle
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;
		if( time >= unified_start_time && start == 0 && restart == false) { //synchronized start at unified start time
			start = time;
		}
		
		// execute redis read callback
		redis_client.executeReadCallback(0);
		
		if(restart == false){
			if (stoi(redis_client.get("gui::is_playing")) == 0){   //check to see if stop button has been pressed
				startedPlaying = false;
				restart = true;
				//set all limbs to go to home state
				LH_state = HOME;		
				RH_state = HOME;
				LF_state = HOME;
				RF_state = HOME;
				Head_state = START;
				
				//set task des pos to initial positions
				pos_des = lh_init_pos;
				pos_des_ra = rh_init_pos;
				
				//set joint space to initial joint configuration
				joint_desired = q_init_desired;
				head_joint_desired = q_init_desired;
			}
		}

		if (restart == true){
			if (stoi(redis_client.get("gui::is_playing")) == 1){  //if start button has been pressed again
				restart = false;

				/**************START OF GUI FILE-READ******************/
				// Read BPM and looptime from redis f
				bpm = std::stod(redis_client.get(BPM_KEY));
				measureLength = std::stod(redis_client.get(LOOP_TIME_KEY));
				period = bpm / measureLength; // 2.0 * M_PI
				//Right hand
				float rh[4][10];
				readGUI("right_hand.txt", rh, no_tsteps_rh);	
				//Store time,x,y,z data
				for(int ct = 0; ct<no_tsteps_rh;ct++){

					time_data_rh(ct) = rh[0][ct];
					x_data_rh(ct) = rh[1][ct];
					y_data_rh(ct) = rh[2][ct];
					z_data_rh(ct) = rh[3][ct];
				
				}
				
				//Left hand
				float lh[4][10];
				readGUI("left_hand.txt", lh, no_tsteps_lh);
				//Store time,x,y,z data
				for(int ct = 0; ct<no_tsteps_lh;ct++){

					time_data_lh(ct) = lh[0][ct];
					x_data_lh(ct) = lh[1][ct];
					y_data_lh(ct) = lh[2][ct];
					z_data_lh(ct) = lh[3][ct];

				}	

				//Right foot
				float rf[4][10];
				readGUI("right_foot.txt", rf, no_tsteps_rf);
				//Store time,x,y,z data
				for(int ct = 0; ct<no_tsteps_rf;ct++){
					time_data_rf(ct) = rf[0][ct];
					x_data_rf(ct) = rf[1][ct];
					y_data_rf(ct) = rf[2][ct];
					z_data_rf(ct) = rf[3][ct];
				} 	

				//Left Foot
				float lf[4][10];
				readGUI("left_foot.txt", lf, no_tsteps_lf);	
				//Store time,x,y,z data
				for(int ct = 0; ct<no_tsteps_lf;ct++){
					time_data_lf(ct) = lf[0][ct];
					x_data_lf(ct) = lf[1][ct];
					y_data_lf(ct) = lf[2][ct];
					z_data_lf(ct) = lf[3][ct];
				}

				start_time = timer.elapsedTime();
				time = 0;
				startedPlaying = true;
				start = 0;

				
			/*******END OF GUI FILE-READ*********************/
			}
		}	
		
		//set desired joint sinusoidal circular motion
		if (time >= unified_start_time - time_to_bob - t_bob_buffer && (startedPlaying == true)){
			switch(Head_state){
				case START:
					start_nod_time = timer.elapsedTime();
					nod_time = 0;
					//ang_Head_des = 0 * M_PI / 180;
					ang_Head_des = q_init_desired[Head_joint] + amplitudeBob * sin( start_nod_time * (2 * M_PI * period) * 2 + M_PI);//time * 2 * M_PI * period is 1/2 head nod is one beat
					head_joint_desired[Head_joint] = ang_Head_des; //set desired head position
					Head_state = NODDING;

				case NODDING:
					nod_time = start_nod_time - timer.elapsedTime();
					ang_Head_des = amplitudeBob * sin( nod_time * (2 * M_PI * period) * 2 + M_PI);//time * 2 * M_PI * period is 1/2 head nod is one beat
					head_joint_desired[Head_joint] = ang_Head_des; //set desired head position
			}
		}		
		
		if (no_tsteps_lh != 0){
			switch(LH_state){
				case HOME:
					if (startedPlaying == true){
						i = 0;
						pos_des << x_data_lh(i), y_data_lh(i), z_data_lh(i);
						pos_des(2) += dz;  //at vtravel
						posori_task_left_hand->_otg->setMaxLinearVelocity(v_travel);
						LH_state = FIRST_MOVING;
						// cout << "DRUM STATE: " << LH_state << "\n";
					} 
					break;
				case FIRST_MOVING:
					if (time >= unified_start_time + time_data_lh(i) - time_to_hit - t_buffer){ //move in anticipation to the synchronized start before 'start' has been set
						
						pos_des << x_data_lh(i), y_data_lh(i), z_data_lh(i); //at vhit
						posori_task_left_hand->_otg->setMaxLinearVelocity(v_hit);
						LH_state = HITTING_DRUM;
						// cout << "DRUM STATE: " << LH_state << "\n";
					}
					break;
				case LIFTING_DRUMSTICK:
					// QUESTION: (curr_pos - pos_desired).norm() < threshold?
					if (abs(curr_pos.norm()-pos_des.norm()) < threshold){ 
						i++;
						if (i % no_tsteps_lh == 0){
							i = 0;
							LH_state = MOVING_DRUMSTICK;
							// Eigen::VectorXd addTime = 60 * Eigen::VectorXd::Ones(no_tsteps_lh); // CHANGED TO BELOW
							Eigen::VectorXd addTime = measureLength * Eigen::VectorXd::Ones(10);
							time_data_lh = time_data_lh + addTime; //wrap time around by adding the length of one time measure (one song meter)

							//these desired pos assignments HAVE to come after setting i = 0!
							pos_des << x_data_lh(i), y_data_lh(i), z_data_lh(i);
							pos_des(2) += dz;  //at vtravel
							// posori_task_left_hand->_linear_saturation_velocity = v_travel; 
							posori_task_left_hand->_otg->setMaxLinearVelocity(v_travel);
						}
						else{
							pos_des << x_data_lh(i), y_data_lh(i), z_data_lh(i);
							pos_des(2) += dz;      //at vtravel
							posori_task_left_hand->_otg->setMaxLinearVelocity(v_travel);
							LH_state = MOVING_DRUMSTICK;
							// cout << "DRUM STATE: " << LH_state << "\n";

						}
					}
					break;
				case MOVING_DRUMSTICK:				
					if (time - start >= time_data_lh(i) - time_to_hit - t_buffer){
						//cout << "\nstarting hit ra " << time - start << "\n";
						pos_des << x_data_lh(i), y_data_lh(i), z_data_lh(i);   //at vhit
						// posori_task_left_hand->_linear_saturation_velocity = v_hit; 
						posori_task_left_hand->_otg->setMaxLinearVelocity(v_hit);
						LH_state = HITTING_DRUM;
						// cout << "DRUM STATE: " << LH_state << "\n";			
						// data_file << "starting" << " ";
						// data_file << curr_pos.transpose() << " ";
						// data_file << time - start << "\n";
					}
					break;
				case HITTING_DRUM:
					if (abs(curr_pos.norm()-pos_des.norm()) < threshold){
						if ((float)pos_des(0) == (float)snare(0)){
							redis_client.set(SNARE_KEY, "1");
							cout << "SNARE" << "\n";
						}
						if ((float)pos_des(0) == (float)tom1(0)){
							redis_client.set(TOM1_KEY, "1");
							cout << "TOM1" << "\n";
						}
						if ((float)pos_des(0) == (float)tom2(0)){
							redis_client.set(TOM2_KEY, "1");
							cout << "TOM2" << "\n";
						}
						cout  << "TIME AT DRUM HIT: " << time - start << "\n";
						pos_des << x_data_lh(i), y_data_lh(i), z_data_lh(i);
						pos_des(2) += dz;      //at vtravel
						// posori_task_left_hand->_linear_saturation_velocity = v_travel; 
						posori_task_left_hand->_otg->setMaxLinearVelocity(v_travel);
						LH_state = LIFTING_DRUMSTICK;
						// cout << "DRUM STATE: " << LH_state << "\n";
						// data_file << "detected" << " ";
						// data_file << curr_pos.transpose() << " ";
						// data_file << time - start << "\n";
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
					if (startedPlaying == true){
						index_ra = 0;
						pos_des_ra << x_data_rh(index_ra), y_data_rh(index_ra), z_data_rh(index_ra);
						// cout << "INITIAL_POS_DES_RA: " << pos_des_ra << "\n";
						pos_des_ra(2) += dz;  //at vtravel
						posori_task_right_hand->_otg->setMaxLinearVelocity(v_travel);
						RH_state = FIRST_MOVING;
						// cout << "DRUM STATE RH: " << RH_state << "\n";
					} 
					break;
				case FIRST_MOVING:
					if (time >= unified_start_time + time_data_rh(index_ra) - time_to_hit - t_buffer){ //move in anticipation to the synchronized start before 'start' has been set
						
						pos_des_ra << x_data_rh(index_ra), y_data_rh(index_ra), z_data_rh(index_ra); //at vhit
						posori_task_right_hand->_otg->setMaxLinearVelocity(v_hit);
						RH_state = HITTING_DRUM;
						// cout << "DRUM STATE RH: " << RH_state << "\n";
					}
					break;
				case LIFTING_DRUMSTICK:
					if (abs(curr_pos_ra.norm()-pos_des_ra.norm()) < threshold){
						index_ra++;
						if (index_ra % no_tsteps_rh == 0){
							index_ra = 0;
							RH_state = MOVING_DRUMSTICK;
							// Eigen::VectorXd addTime = 60 * Eigen::VectorXd::Ones(no_tsteps_rh);
							Eigen::VectorXd addTime = measureLength * Eigen::VectorXd::Ones(10);
							time_data_rh = time_data_rh + addTime; //wrap time around by adding the length of one time measure (one song meter)

							//these desired pos assignments HAVE to come after setting i = 0!
							pos_des_ra << x_data_rh(index_ra), y_data_rh(index_ra), z_data_rh(index_ra);
							pos_des_ra(2) += dz;  //at vtravel
							posori_task_right_hand->_otg->setMaxLinearVelocity(v_travel);
						}
						else{
							pos_des_ra << x_data_rh(index_ra), y_data_rh(index_ra), z_data_rh(index_ra);
							pos_des_ra(2) += dz;      //at vtravel
							posori_task_right_hand->_otg->setMaxLinearVelocity(v_travel);
							RH_state = MOVING_DRUMSTICK;
							// cout << "DRUM STATE RH: " << RH_state << "\n";
						}
					}
					break;
				case MOVING_DRUMSTICK:
					
					if (time - start >= time_data_rh(index_ra)-time_to_hit-t_buffer){
						// cout << "\nstarting hit ra " << time - start << "\n";
						pos_des_ra << x_data_rh(index_ra), y_data_rh(index_ra), z_data_rh(index_ra);   //at vhit
						//posori_task_right_hand->_linear_saturation_velocity = v_hit;
						posori_task_right_hand->_otg->setMaxLinearVelocity(v_hit);
						RH_state = HITTING_DRUM;
						// cout << "DRUM STATE RH: " << RH_state << "\n";
						// data_filera << "starting" << " ";
						// data_filera << curr_pos_ra.transpose() << " ";
						// data_filera<< time - start << "\n";
					}
					break;
				case HITTING_DRUM:
					if (abs(curr_pos_ra.norm()-pos_des_ra.norm()) < threshold){
						if ((float)pos_des_ra(0) == (float)snare(0)){
							redis_client.set(SNARE_KEY, "1");
							cout << "SNARE" << "\n";
						}
						if ((float)pos_des_ra(0) == (float)tom1(0)){
							redis_client.set(TOM1_KEY, "1" );
							cout << "TOM1" << "\n";
						}
						if ((float)pos_des_ra(0) == (float)tom2(0)){
							redis_client.set(TOM2_KEY, "1");
							cout << "TOM2" << "\n";
						}
						cout  << "TIME AT DRUM HIT: " << time - start << "\n";
						pos_des_ra << x_data_rh(index_ra), y_data_rh(index_ra), z_data_rh(index_ra);
						pos_des_ra(2) += dz;      //at vtravel
						posori_task_right_hand->_otg->setMaxLinearVelocity(v_travel);
						RH_state = LIFTING_DRUMSTICK;
						// cout << "DRUM STATE RH: " << RH_state << "\n";
						data_filera << "detected" << " ";
						data_filera << curr_pos_ra.transpose() << " ";
						data_filera << time - start << "\n";
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
					if (startedPlaying == true){
						index_LF = 0;
						LF_state = FIRST_MOVING;
						// cout << "LEFT LEG STATE: " << LF_state << "\n";
					}
					break;
				case FIRST_MOVING:
					if (time >= unified_start_time + time_data_lf(index_LF) - time_to_stomp - t_stomp_buffer){
						// cout << "!![0]!!" << unified_start_time + time_data_lf(index_LF) - time_to_stomp - t_stomp_buffer << endl;
						// cout << time << endl;
						// cout << unified_start_time << endl;
						// cout << start << endl;
						// cout << restart << endl;

						
						ang_LF_des = LF_stomp;
						LF_state = STOMP;
						// cout << "LEFT LEG  STATE: " << LF_state << "\n";
					}
					break;
				case WAIT:
					if (time - start >= time_data_lf(index_LF) - time_to_stomp - t_stomp_buffer){
						//cout << "\nstarting stomp LL " << time - start << "\n";
						ang_LF_des = LF_stomp;
						LF_state = STOMP;
						// cout << "LEFT LEG  STATE: " << LF_state << "\n";
						
						// data_filelf << "starting" << " ";
						// data_filelf << curr_LF_ang << " ";
						// data_filelf << time - start << "\n";
					}
					break;
				case STOMP:
					if (abs(curr_LF_ang - ang_LF_des) < thetaThreshold){
						redis_client.set(HIHAT_KEY, "1");
						index_LF++;
						ang_LF_des = LF_lift;
						LF_state = WAIT;
						cout << "TIME AT HIHAT HIT: " << time - start << "\n";
						if (index_LF % no_tsteps_lf == 0){
							index_LF = 0;
							// Eigen::VectorXd addLLTime = 60 * Eigen::VectorXd::Ones(no_tsteps_lf);
							Eigen::VectorXd addLLTime = measureLength * Eigen::VectorXd::Ones(10);
							time_data_lf = time_data_lf + addLLTime;
						}

						// cout << unified_start_time << endl;
						// cout << start << endl;
						// cout << restart << endl;

						// cout << "LEFT LEG  STATE: " << LF_state << "\n";
						// data_filelf << "detected" << " ";
						// data_filelf << curr_LF_ang << " ";
						// data_filelf << time - start << "\n";
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
					if (startedPlaying == true){
						index_RF = 0;
						RF_state = FIRST_MOVING;
						// cout << "RIGHT LEG  STATE: " << RF_state << "\n";
					}
					break;
				case FIRST_MOVING:
					if (time >= unified_start_time + time_data_rf(index_RF) - time_to_stomp - t_stomp_buffer){
						ang_RF_des = RF_stomp;
						RF_state = STOMP;
						// cout << "RIGHT LEG  STATE: " << RF_state << "\n";
					}
					break;
				case WAIT:
					if (time - start >= time_data_rf(index_RF) - time_to_stomp - t_stomp_buffer){
						// cout << "\nstarting stomp LL " << time - start << "\n";
						ang_RF_des = RF_stomp;
						RF_state = STOMP;
						// cout << "RIGHT LEG  STATE: " << RF_state << "\n";
						
						// data_filerf << "starting" << " ";
						// data_filerf << curr_RF_ang << " ";
						// data_filerf << time - start << "\n";
					}
					break;
				case STOMP:
					if (abs(curr_RF_ang - ang_RF_des) < thetaThreshold){
						redis_client.set(BASS_KEY, "1");
						index_RF++;
						ang_RF_des = RF_lift;
						RF_state = WAIT;
						cout << "TIME AT BASS HIT: " << time - start << "\n";
						if (index_RF % no_tsteps_rf == 0){
							index_RF = 0;
							// Eigen::VectorXd addLLTime = 60 * Eigen::VectorXd::Ones(no_tsteps_rf);
							Eigen::VectorXd addLLTime = measureLength * Eigen::VectorXd::Ones(10);
							time_data_rf = time_data_rf + addLLTime;
						}
						// cout << "RIGHT LEG  STATE: " << RF_state << "\n";
						// data_filerf << "detected" << " ";
						// data_filerf << curr_RF_ang << " ";
						// data_filerf << time - start << "\n";
					}
					break;
				default:
					break;
			}//end right leg (bass) state machine
			
			joint_desired[RF_joint] = ang_RF_des; //set desired LL position
		}

		// calculate torques to maintain hip_base posture
		N_prec.setIdentity();		
		posori_task_torso->updateTaskModel(N_prec);
		posori_task_torso->computeTorques(posori_task_torques_torso);		

		// calculate torques to move right hand
		N_prec = posori_task_torso->_N;
		posori_task_right_hand->updateTaskModel(N_prec);
		posori_task_right_hand->computeTorques(posori_task_torques_right_hand);

		// calculate torques to move left hand
		N_prec = posori_task_right_hand->_N;
		posori_task_left_hand->updateTaskModel(N_prec);
		posori_task_left_hand->computeTorques(posori_task_torques_left_hand);

		// calculate torques to maintain joint posture
		N_prec = posori_task_left_hand->_N;

		double kpj = 400;
		double kvj = 20;

		VectorXd feet_task_torques = VectorXd::Zero(2);
		VectorXd full_feet_task_torques = VectorXd::Zero(dof);

		MatrixXd J_feet = MatrixXd::Zero(2, dof);
		J_feet(0,LF_joint) = 1;
		J_feet(1,RF_joint) = 1;
		
		full_feet_task_torques = robot->_M * (-kpj * (robot->_q - joint_desired) - kvj * (robot->_dq));
		feet_task_torques(0) = full_feet_task_torques(LF_joint);
		feet_task_torques(1) = full_feet_task_torques(RF_joint);

		feet_task_torques = N_prec.transpose() * J_feet.transpose() * feet_task_torques;

		MatrixXd N_feet = MatrixXd::Identity(dof, dof);
		robot->nullspaceMatrix(N_feet, J_feet, N_prec);

		//////////////////////////////////////////////////

		VectorXd head_task_torques = VectorXd::Zero(1);
		VectorXd full_head_task_torques = VectorXd::Zero(dof);

		MatrixXd J_head = MatrixXd::Zero(1 , dof);
		J_head(0 , Head_joint) = 1;
		
		full_head_task_torques = robot->_M * (-kpj * (robot->_q - head_joint_desired) - kvj * (robot->_dq));
		head_task_torques(0) = full_head_task_torques(Head_joint);

		head_task_torques = N_feet.transpose() * J_head.transpose() * head_task_torques;

		MatrixXd N_head = MatrixXd::Identity(dof, dof);
		robot->nullspaceMatrix(N_head, J_head, N_feet);	

		joint_task->_desired_position = q_init_desired;
		joint_task->updateTaskModel(N_head);
		joint_task->computeTorques(joint_task_torques);

		/////////////////////////////////////////////////////////

		// calculate torques 
		command_torques = posori_task_torques_right_hand + posori_task_torques_left_hand + posori_task_torques_torso + feet_task_torques + head_task_torques + joint_task_torques;// + head_joint_task_torques;//posori_task_torques_torso + joint_task_torques;  // gravity compensation handled in sim

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

	data_file.close();
	data_filera.close();
	data_filelf.close();
	data_filerf.close();

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
