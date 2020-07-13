#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <fstream>
#include <ctime>
#include "gnss_main.h"
#include "HgDataParser.h"
#include "Serial.h"
#include "navconst.h"
#include "signa.h"
#include "config.h"
#include <unistd.h>
#include <fcntl.h>
#include <assert.h>
#include <termio.h>
#include <iostream>
#include <time.h>
//#include "filter_functions.h"
#include "LCEKF.h"
#include <algorithm>

#define BUFFER_SIZE 2048
//Select the IMU used
//#define HG1120
#define HG4930

extern NavParameter NavParam;
extern NavSystem NavSys;

using namespace std;

int step = 1;
int clear = 1;

int no_const = 1; // 1 : GPS, 2 : GPS + GLONASS, 3 : GPS + GLONASS + GALILEO
int no_state = 15;

double pos_NE_SD = 1;
double pos_D_SD = 2;
double vel_NE_SD = 0.1;
double vel_D_SD = 0.2;

double dualheading;

int main(int argc, char **argv) {
	remove("/home/namgihun/gnss_INS_fusion_loosely_demoz_ECEF/acc_info.txt");
	remove("/home/namgihun/gnss_INS_fusion_loosely_demoz_ECEF/Ground.gps");
	remove("/home/namgihun/gnss_INS_fusion_loosely_demoz_ECEF/Output_imu.txt");
	remove("/home/namgihun/gnss_INS_fusion_loosely_demoz_ECEF/State.txt");
	remove("/home/namgihun/gnss_INS_fusion_loosely_demoz_ECEF/True_Position.txt");
	remove("/home/namgihun/gnss_INS_fusion_loosely_demoz_ECEF/Covariance.txt");
	double state[15]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; //attitude[3],velocity[3],position[3],IMU bias[6],clock bias and clock drift

	double C_b_e[3][3]={
			{-0.7766,0.0141,-0.6299},
			{0.0175,0.9998,0.0009},
			{0.6298,-0.0103,-0.7767}
	};

	int i,j;

	double **P= new double*[no_state]; //define a row of covariance double pointer

	for(int i=1; i<no_state+1; i++){
		P[i-1]=new double[no_state]; //define column for each row - P[no_state][no_state]
		memset(P[i-1], 0, sizeof(int)*no_state); //initialize the memory space as zero
	}

	for(int i=1; i<4; i++){
		P[i-1][i-1]=3.0462e-04;
		P[i+2][i+2]=0.01;
		P[i+5][i+5]=100;
		P[i+8][i+8]=9.6170e-05;
		P[i+11][i+11]=2.3504e-09;
	}

	char comPort[32] = "/dev/ttyUSB0";
	int baudrate = 1000000;

	int status=0;
	//Connect to defined serial port
	int SerialHandle;
	status = serial_init(&SerialHandle, comPort);
	if (status!=0)
{
		printf("failed to open Com port:%s\n",comPort);
		return status;
}
	status = serial_configure(SerialHandle, baudrate, SER_PARITY_NO, 1, 8);
	if (status!=0)
{
		printf("failed to configure Com port:%s\tstatus:%d\n",comPort,status);
		return status;
}

	printf("Input COM:\t%s\t%d\n",comPort,baudrate);

	//Allocate the read buffer
	UINT8 ReadBuffer[BUFFER_SIZE*2] = {0};
	memset(ReadBuffer, 0, BUFFER_SIZE*2);
	UINT8 *ReadBufferAct = ReadBuffer;
	UINT8 *ReadBufferEnd = ReadBuffer;

	int BytesRead = 0;
	//UINT x = 0;
	status = 2;

#ifdef HG4930
	#define INERTIAL_MESSAGE_LEN 44
#else
	#define INERTIAL_MESSAGE_LEN 50
#endif

#ifdef HG4930
	HgDataParser::HG4930InertialMessage Message;
#else
	//Data structure for HG1120 data defined in .dll
	HgDataParser::HG1120InertialMessage Message;
#endif

	bool IMU_health;
	bool Gyro_health;
	bool Gyro_health2;

	Message.ZeroMessage();
	///////////////////////////////////////////////////////// IMU Reading Part

///*
	Reading_GNSS(); // Start Reading GNSS Data

	double tor_gnss=0.5; //time interval
	int no_meas;

	while (state[6]==0){
		state[6]=NavSys.RefPos_LLA[0]; // 처음 gps 데이터 수신할떄까지 멈춤.
		state[7]=NavSys.RefPos_LLA[1];
		state[8]=NavSys.RefPos_LLA[2];
	}

	//state[6]=0;
//*/
	LCEKF myClass;



	double LS_position[3];
	for (int i=0;i<3;i++)
		LS_position[i]=NavSys.RefPos_ECEF[i];

	double tor = 0.01;
	double IMU[6];
	double IMU_data[6]={0,0,0,0,0,0};
	double xyz[3];
	double lla[3];

	int NumData_prev=0;
	int num_gnss=0;
	int num_ins=0;
	double end_prev;
	Matrix<double,15,15> P_mat;

	clock_t start, end;

	int nums = 1;
	double best_state[6];
	double best_state_SD[6];
	while (1) {

		fstream imudataFile;
		fstream covFile;
		fstream stateFile;

		string buffer;
		imudataFile.open("Output_imu.txt",ios::app);
		covFile.open("Covariance.txt",ios::app);
		stateFile.open("State.txt",ios::app);
///*
		memcpy(ReadBuffer, ReadBufferAct, (ReadBufferEnd - ReadBufferAct));
		ReadBufferEnd = ReadBuffer + (ReadBufferEnd - ReadBufferAct);
		ReadBufferAct = ReadBuffer;

		if (!serial_read(SerialHandle, (char*)ReadBufferEnd, BUFFER_SIZE, &BytesRead))
		{
			printf("\nFailed to read Data!\n");
		}
		else
		{
			// Move the pointer to the end of the Read buffer
			ReadBufferEnd += (int)BytesRead;

			while (ReadBufferAct <= ReadBufferEnd - INERTIAL_MESSAGE_LEN)
			{
				if (*(UINT8*)ReadBufferAct == 0x0E)
				{
					// Switch Message Code - Call appropriate .dll functions
					switch (*(UINT8*)(ReadBufferAct + 1))
					{
					#ifdef HG4930
					case 0x01:
					{status = HgDataParser::GetHG4930X01ControlMessage(ReadBufferAct, 0, &Message.ControlMessage); ReadBufferAct += 19;

					break; }
					case 0x02:
					{status = HgDataParser::GetHG4930X02InertialMessage(ReadBufferAct, 0, &Message); ReadBufferAct += 43;

					break; }
					#else
					case 0x04:
					{status = HgDataParser::GetHG1120X04ControlMessage(ReadBufferAct, 0, &Message.ControlMessage); ReadBufferAct += 25; break; }
					case 0x05:
					{status = HgDataParser::GetHG1120X05InertialMessage(ReadBufferAct, 0, &Message); ReadBufferAct += 49; break; }
					case 0x0C:
					{status = HgDataParser::GetHG1120X0CControlMessage(ReadBufferAct, 0, &Message.ControlMessage); ReadBufferAct += 25; break; }
					case 0x0D:
					{status = HgDataParser::GetHG1120X0DInertialMessage(ReadBufferAct, 0, &Message); ReadBufferAct += 49; break; }
					#endif
					default:
					{ status = 2; }
					}

					IMU[0]=-Message.ControlMessage.AngularRate[1]; ////////////////// 1 2 0 - + -
					IMU[1]=Message.ControlMessage.AngularRate[2];
					IMU[2]=-Message.ControlMessage.AngularRate[0];
					IMU[3]=-Message.ControlMessage.LinearAcceleration[1]; ///// cahnge need?????
					IMU[4]=Message.ControlMessage.LinearAcceleration[2];
					IMU[5]=-Message.ControlMessage.LinearAcceleration[0];
					IMU_health=Message.ControlMessage.StatusWord.AccelerometerHealth;
					Gyro_health=Message.ControlMessage.StatusWord.GyroHealth;
					Gyro_health2=Message.ControlMessage.StatusWord.GyroHealth2;

					if (!IMU_health && !Gyro_health && !Gyro_health2){
						num_ins++;

						IMU_data[0] = IMU[0];
						IMU_data[1] = IMU[1];
						IMU_data[2] = IMU[2];
						IMU_data[3] = IMU[3];
						IMU_data[4] = IMU[4];
						IMU_data[5] = IMU[5];
						Vector3d w = Vector3d(IMU_data[0],IMU_data[1],IMU_data[2]);
						Vector3d a = Vector3d(IMU_data[3],IMU_data[4],IMU_data[5]);
						Vector3d pos = Vector3d(NavSys.RefPos_ECEF[0], NavSys.RefPos_ECEF[1], NavSys.RefPos_ECEF[2]);
						Vector3d vel = Vector3d(NavSys.RefVel_ECEF[0], NavSys.RefVel_ECEF[1], NavSys.RefVel_ECEF[2]);
						double dualheading;

						for (i=0;i<3;i++){
							best_state[i] = NavSys.RefPos_ECEF[i];
							best_state[i+3] = NavSys.RefVel_ECEF[i];
							best_state_SD[i] = NavSys.RefPos_ECEF_SD[i];
							best_state_SD[i+3] = NavSys.RefVel_ECEF_SD[i];
							//best_state_SD[i+3] = 0.1;
						}
						Vector3d pos_SD = Vector3d(best_state_SD[0],best_state_SD[1],best_state_SD[2]);
						Vector3d vel_SD = Vector3d(best_state_SD[3],best_state_SD[4],best_state_SD[5]);

						tor=0.00000001;

						if (num_gnss == 2 && nums ==1){
							nums ++;
							printf("\n bestpos, bestvel : %f %f %f %f %f %f \n",pos(0),pos(1),pos(2),vel(0),vel(1),vel(2));
							myClass.Configure(pos_SD,vel_SD);
							myClass.Initialization(w, a, pos, vel, pos_SD, vel_SD , dualheading);
						}
						if (num_ins > 1 && num_gnss > 2){
							tor=0.00166666666666666666666666666666;
							myClass.Update(NavSys.NumData, w, a, pos, vel, pos_SD, vel_SD, dualheading);
							state[6] = myClass.Est_r_eb_e_(0);
							state[7] = myClass.Est_r_eb_e_(1);
							state[8] = myClass.Est_r_eb_e_(2);
							state[3] = myClass.Est_v_eb_e_(0);
							state[4] = myClass.Est_v_eb_e_(1);
							state[5] = myClass.Est_v_eb_e_(2);
							state[0] = myClass.Est_att_euler_(0)/M_PI*180;
							state[1] = myClass.Est_att_euler_(1)/M_PI*180;
							state[2] = myClass.Est_att_euler_(2)/M_PI*180;
							state[9] = myClass.Est_IMUaBias_(0);
							state[10]= myClass.Est_IMUaBias_(1);
							state[11]= myClass.Est_IMUaBias_(2);
							state[12]= myClass.Est_IMUwBias_(0);
							state[13]= myClass.Est_IMUwBias_(1);
							state[14]= myClass.Est_IMUwBias_(2);

							P_mat = myClass.P_matrix_;
							//state[0] = myClass.quat_BL_.w();
							//state[1] = myClass.quat_BL_.x();
							//state[2] = myClass.quat_BL_.y();
							//state[3] = myClass.quat_BL_.z();
							//printf("\n %f \n",myClass.pEst_D_rrm_(2));
						///////////////////////
						//if (tor >0.0099){
							//tor =0.016666666;
						//	num_ins++;
							//System_Update(IMU_data, state, P, C_b_e, tor, dualheading);
						//}


						for (i=0; i<6; i++)  {
							imudataFile << IMU[i];
							imudataFile << ",";
						}

						covFile << "IMU";
						for (i=0; i<15; i++){
							covFile << ",";
							covFile << setprecision(4) << P_mat(i,i);
						}

						stateFile << "IMU,";
						for (i=0; i<15; i++){
							if (i>5 && i<9){
								stateFile << setprecision(12) << state[i];
								stateFile << ",";
							} else if (i<3){
								stateFile << setprecision(6) << state[i];
								stateFile << ",";
							} else {
								stateFile << setprecision(4) << state[i];
								stateFile << ",";
							}
						}
						for (i=0; i<3; i++){
							stateFile << setprecision(4) << C_b_e[i][i];
							stateFile << ",";
						}
						stateFile << tor;
						stateFile << ",";

						for (i=0;i<3;i++)
							xyz[i]=state[i+6];
						ConvertECEF2LLA(xyz,lla);

						for (i=0; i<3; i++){
							stateFile << setprecision(12) << lla[i];
							stateFile << ",";
						}

						stateFile << setprecision(12) << NavSys.sTimeCurrent;

						imudataFile << "\n";
						covFile << "\n";
						stateFile << "\n";
						}
						//printf("\n Rawdata : %f %f %f %f %f %f %d %f",IMU[0],IMU[1],IMU[2],IMU[3],IMU[4],IMU[5],num_ins,tor);
						//printf("\n IMU_x : %f",IMU[3]);
						//printf("\n IMUdata : %f %f %f %f %f %f %d %f",IMU_data[0],IMU_data[1],IMU_data[2],IMU_data[3],IMU_data[4],IMU_data[5],num_ins,tor);
						//printf("\n system : Velocity, Position : %f  %f  %f  %f  %f  %f",state[3],state[4],state[5],state[6],state[7],state[8]);
						//printf("\n %f  %f  %f  %f  %f  %f %d %f\n",state[9],state[10],state[11],state[12],state[13],state[14],num_ins,tor);
					}
				}

				//Move the read buffer pointer
				ReadBufferAct++;
			} // !ReadFile

			//Sleep(1);
		}

		imudataFile.close();
		covFile.close();
		stateFile.close();

//EKFNav::Update(uint64_t t_us, unsigned long timeWeek, Vector3f wMeas_B_rps, Vector3f aMeas_B_mps2, Vector3d pMeas_D_rrm, Vector3f vMeas_NED_mps, double Bestpos_NE_SD, double Bestpos_D_SD, double Bestvel_NE_SD, double Bestvel_D_SD, double dualheading)
//void Configure(double pos_NE_SD, double pos_D_SD, double vel_NE_SD, double vel_D_SD);
//void Initialize(Vector3f wMeas_B_rps, Vector3f aMeas_B_mps2, Vector3d pMeas_D_rrm, Vector3f vMeas_NED_mps, double Bestpos_NE_SD, double Bestpos_D_SD, double Bestvel_NE_SD, double Bestvel_D_SD, double dualheading);
		//myClass.Update(t, 0, w, a, pos, vel, pos_NE_SD, pos_D_SD, vel_NE_SD,vel_D_SD,dualheading);
//*/
///*
	if(NavSys.NumData>num_gnss){
		num_gnss++;
	if(num_gnss > 2){
	fstream stateFile;
	fstream covFile;
	fstream truePFile;

	stateFile.open("State.txt",ios::app);
	covFile.open("Covariance.txt",ios::app);
	truePFile.open("True_Position.txt",ios::app);


	double est_pos[4];
	double est_lla[3];
	//while (dualheading ==0 && num_gnss<2)
		//printf("\n dualheading is zero");

	state[6] = myClass.Est_r_eb_e_(0);
	state[7] = myClass.Est_r_eb_e_(1);
	state[8] = myClass.Est_r_eb_e_(2);
	state[3] = myClass.Est_v_eb_e_(0);
	state[4] = myClass.Est_v_eb_e_(1);
	state[5] = myClass.Est_v_eb_e_(2);
	state[0] = myClass.Est_att_euler_(0)/M_PI*180;
	state[1] = myClass.Est_att_euler_(1)/M_PI*180;
	state[2] = myClass.Est_att_euler_(2)/M_PI*180;
	state[9] = myClass.Est_IMUaBias_(0);
	state[10]= myClass.Est_IMUaBias_(1);
	state[11]= myClass.Est_IMUaBias_(2);
	state[12]= myClass.Est_IMUwBias_(0);
	state[13]= myClass.Est_IMUwBias_(1);
	state[14]= myClass.Est_IMUwBias_(2);

	P_mat = myClass.P_matrix_;

	covFile << "GNSS";
	for (i=0; i<15; i++){
		covFile << ",";
		covFile << setprecision(4) << P_mat(i,i);
	}
	stateFile << "GNSS,";
	for (i=0; i<15; i++){
		if (i>5 && i<9){
			stateFile << setprecision(12) << state[i];
			stateFile << ",";
		} else if (i<3){
			stateFile << setprecision(6) << state[i];
			stateFile << ",";
		} else {
			stateFile << setprecision(4) << state[i];
			stateFile << ",";
		}
	}
	for (i=0; i<3; i++){
		stateFile << setprecision(4) << C_b_e[i][i];
		stateFile << ",";
	}
	stateFile << tor_gnss;
	stateFile << ",";

	for (i=0;i<3;i++)
		xyz[i]=state[i+6];
	ConvertECEF2LLA(xyz,lla);

	for (i=0; i<3; i++){
		stateFile << setprecision(12) << lla[i];
		stateFile << ",";
	}

	stateFile << setprecision(12) << NavSys.sTimeCurrent;

	for (i=0;i<3;i++){
		truePFile << setprecision(12) << NavSys.RefPos_ECEF[i];
		truePFile << ",";
	}
	for (i=0;i<3;i++){
		truePFile << setprecision(12) << NavSys.RefPos_LLA[i];
		truePFile << ",";
	}

	for (i=0;i<3;i++){
		truePFile << setprecision(12) << NavSys.RefVel_ECEF[i];
		truePFile << ",";
	}

	for (i=0;i<6;i++){
		truePFile << setprecision(12) << best_state_SD[i];
		truePFile << ",";
	}

	covFile << "\n";
	stateFile << "\n";
	truePFile << "\n";

	stateFile.close();
	covFile.close();
	truePFile.close();

	printf("\n Real position : %f  %f  %f %f %f %f \n",NavSys.RefPos_ECEF[0],NavSys.RefPos_ECEF[1],NavSys.RefPos_ECEF[2],NavSys.RefPos_LLA[0]/M_PI*180,NavSys.RefPos_LLA[1]/M_PI*180,NavSys.RefPos_LLA[2]);
	printf("\n NumData : %d ,Real Velocity : %f %f %f",num_gnss,NavSys.RefVel_ECEF[0],NavSys.RefVel_ECEF[1],NavSys.RefVel_ECEF[2]);
	printf("\n Estimate Position : %f %f %f Estimated Velocity : %f %f %f",state[6],state[7],state[8],state[3],state[4],state[5]);

	}
	} // gnss if문
//*/
} // while문

}
