/*************************************************************************/
/* File:        MotorUnits.c                                             */
/* Description:                                                          */
/* Date:        01-29-2011                                               */
/*************************************************************************/

#include <math.h>
#include <stdio.h>
#include "Xkw/Xkw.h"

#include "Roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"

double Kp_arm = KP_ARM;
double Kd_arm = KD_ARM;
double Kp_eye = KP_EYE;
double Kd_eye = KD_EYE;

#define ARM  1
#define EYE  2
#define BASE 3

// some flags and variables for PD loop tuning
int tune_PD = 0; // flag to set which controller to tune
FILE * writefile;
double last_q1, last_q2;
int new_test = 0; // set to one at the beginning of each test input
int test_num = 0; // counter resets at each setpoint
long int t = 0; // ms time variable
char filename[32];


/* ===== the simulator allows you to control Roger by executing ===== */
/* ===== control_roger() once every millisecond ===================== */
/* ==================== DO NOT alter this method ==================== */
control_roger(roger, time)
Robot * roger;
double time;
{
  update_setpoints(roger);

  // turn setpoint references into torques
  PDController_base(roger, time);
  PDController_arms(roger, time);
  PDController_eyes(roger, time);
  t++;
}



/* === Need to be done in Project 1 === */
/* Arms PD controller, reference value is given in config space*/
PDController_arms(roger, time)
Robot * roger;
double time;

{
  int i,j;
  double theta_error[2], vel[2], M[2][2], V[2], G[2];
  double Kp0, Kd0,Kp1, Kd1;
  double max_error;

  Kp0 = Kp_arm;
	Kd0 = Kd_arm;
	Kp1 = Kp_arm;
	Kd1 = Kd_arm;

 	// check if there's a new right arm setpoint and initialize file i/o
 	if(tune_PD == ARM && (roger->arm_setpoint[RIGHT][0] != last_q1
 			|| roger->arm_setpoint[RIGHT][1] != last_q2)){
 		if(writefile != NULL){// test if the current output file is open
			// close the open file
			fclose(writefile);
			writefile = NULL;
		}
		t = 0; // reset clock
		// create new filename
		sprintf(filename, "outfile_%d.txt", test_num);
		// open a new output file
		writefile = fopen(filename, "w");
		last_q1 = roger->arm_setpoint[RIGHT][0];
		last_q2 = roger->arm_setpoint[RIGHT][1];
		fprintf(writefile, "time\tq1\tq2\n"); // write header to file
		test_num++; // increment test counter
 }

for(i = 0; i < 2; i++){
    // PDcontrol - desired accelerations
    theta_error[0] = roger->arm_setpoint[i][0] - roger->arm_theta[i][0];
	  theta_error[1] = roger->arm_setpoint[i][1] - roger->arm_theta[i][1];
		vel[0] = roger->arm_theta_dot[i][0];
		vel[1] = roger->arm_theta_dot[i][1];

		for (j = 0; j < 2; j++){
			if (theta_error[j] > M_PI)       // a HACK to bound error
      	theta_error[j] -= 2.0 * M_PI;  // -pi < error < +pi
    	if (theta_error[j] < -M_PI)
      	theta_error[j] += 2.0 * M_PI;
		}
		// PD control for joint 0 (shoulder)
    roger->arm_torque[i][0] = Kp0 * theta_error[0] - Kd0 * vel[0];
		// PD control for joint 1 (elbow)
	 	roger->arm_torque[i][1] = Kp1 * theta_error[1] - Kd1 * vel[1];
}
	// print system output every 10 ms
if(tune_PD == ARM){
	if (t % 10 == 0 && writefile != NULL){
		if (t == 0)
			fprintf(writefile, "\t\t\tKp:%lf\tKd:%lf\n", Kp1, Kd1);
		fprintf(writefile, "%ld\t%lf\t%lf\n", t, theta_error[0], theta_error[1]);
		// print time and angle errors for PD tuning
	}
	t++; // increment ms counter
	double max_error;
	// close current output file when steady state is reached
	max_error = (fabs(theta_error[0]) > fabs(theta_error[1]) ? theta_error[0] :
			theta_error[1]);
	
	if (fabs(max_error) < 0.0001 && writefile != NULL && t > 2500){
			fclose(writefile);
			writefile = NULL;
			printf("test %d complete. t = %ld error = %lf\n", test_num, t, max_error);
	}
}
}
/* === Need to be done in Project 1 === */
/* Eyes PD controller, reference value is given in config space */
PDController_eyes(roger, time)
Robot * roger;
double time;
{
  int i;
	long int t = (int)(time * 1000);
  double theta_error[2], theta_ddot_des;

 	// check if there's a new right eye setpoint and initialize file i/o
 	if(tune_PD == EYE && (roger->eyes_setpoint[RIGHT] != last_q1)){
 		if(writefile != NULL){// test if the current output file is open
			// close the open file
			fclose(writefile);
			writefile = NULL;
		}
		t = 0; // reset clock
		// create new filename
		sprintf(filename, "outfile_%d.txt", test_num);
		// open a new output file
		writefile = fopen(filename, "w");
		last_q1 = roger->eyes_setpoint[RIGHT];
		fprintf(writefile, "time\tq1\n"); // write header to file
		test_num++; // increment test counter
 }

  // gains KP_EYE and KD_EYE constants defined in control.h
  for (i = 0; i < NEYES; i++) {
    theta_error[i] = roger->eyes_setpoint[i] - roger->eye_theta[i];
		// adjust eye setpoint to be within gaze bounds
		if (roger->eyes_setpoint[i] > M_PI/2.0)
			roger->eyes_setpoint[i] = M_PI/2.0;
		else if (roger->eyes_setpoint[i] < -M_PI/2.0)
			roger->eyes_setpoint[i] = -M_PI/2.0;
    if (theta_error[i] > M_PI)       // a HACK to bound error
      theta_error[i] -= 2.0 * M_PI;  // -pi < error < +pi
    if (theta_error[i] < -M_PI)
      theta_error[i] += 2.0 * M_PI;

		roger->eye_torque[i] = Kp_eye * theta_error[i] - Kd_eye
				* roger->eye_theta_dot[i];
}
  if (tune_PD == EYE){
		if (t % 10 == 0)
			printf("%ld\t%lf\n", t, theta_error[RIGHT]);
			// print time, theta, and reference angle

		// print system output every 10 ms
		if (t % 10 == 0 && writefile != NULL){
			if (t == 0)
				fprintf(writefile, "\t\t\tKp:%lf\tKd:%lf\n", Kp_eye, Kd_eye);
			fprintf(writefile, "%ld\t%lf\n", t, theta_error[RIGHT]);
			// print time and angle errors for PD tuning
		}
		t++; // increment ms counter

		// close current output file when steady state is reached
		if (fabs(theta_error[RIGHT]) < 0.0001 && writefile != NULL && t > 2500){
				fclose(writefile);
				writefile = NULL;
				printf("test %d complete. t = %ld error = %lf\n", test_num, t,
						theta_error[RIGHT]);
		}
  }
}

