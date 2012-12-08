/*************************************************************************/
/* File:        project2.c                                               */
/* Description: Cartesian control for arms and base                      */
/* Date:        01-29-2011                                               */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include "Xkw/Xkw.h"

#include "Roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"

// PD tuning parameters for base
double Kp_base_Fx = KP_BASE_FX;
double Kd_base_Fx = KD_BASE_FX;
double Kp_base_M = KP_BASE_M;
double Kd_base_M = KD_BASE_M;

// for data collection to files
extern FILE * writefile;
double last_setX, last_setY;
double initialHeading;
extern int new_test; // set to one at the beginning of each test input
extern int test_num; // counter resets at each setpoint
extern long int t; // ms time variable
extern char filename[32];
extern int tune_PD;
int user_set = 0;
double user_xy[2];

Robot* theRobot;

/* === Need to be done in Project 2 === */
/* Inverse Kinematics function */
int inv_kinematics(roger, limb, x, y, theta1, theta2)
Robot* roger;
int limb;
double x, y;
double *theta1, *theta2;
{
  double r2, c2, s2_plus, s2_minus, theta2_plus, theta2_minus;
  double k1, k2_plus, k2_minus, alpha_plus, alpha_minus;
  double theta1_plus, theta1_minus;

  if (limb==LEFT) y -= ARM_OFFSET;
  else y += ARM_OFFSET;
  r2 = x*x + y*y;
  c2 = (r2 - LARM_1*LARM_1 - LARM_2*LARM_2)/(2.0*LARM_1*LARM_2);
  if(fabs(c2) <= 1.0){
  	s2_plus = sqrt(1.0-(c2*c2));
  	s2_minus = -s2_plus;
  	theta2_plus = atan2(s2_plus, c2);
  	theta2_minus = atan2(s2_minus, c2);
  	k1 = LARM_1 + LARM_2 * c2;
  	k2_plus = LARM_2*s2_plus;
  	k2_minus = LARM_2*s2_minus;
		alpha_plus = atan2(k2_plus, k1);
		alpha_minus = atan2(k2_minus, k1);
		theta1_plus = atan2(y,x) - alpha_plus;
		theta1_minus = atan2(y,x) - alpha_minus;

//    // calculate total joint swing and pick minimum
//		double dist_plus = fabs(fmod(theta1_plus-roger->arm_theta[limb][0],M_PI))
//				+ fabs(fmod(theta2_plus-roger->arm_theta[limb][1],M_PI));
//		double dist_minus = fabs(fmod(theta1_minus-roger->arm_theta[limb][0],M_PI))
//						+ fabs(fmod(theta2_minus-roger->arm_theta[limb][1],M_PI));
//		if (dist_plus < dist_minus){
//			*theta1 = theta1_plus;
//			*theta2 = theta2_plus;
//		}
//		else if (dist_minus < dist_plus){
//			*theta1 = theta1_minus;
//			*theta2 = theta2_minus;
//		}
//		else{
//			// bend forward if goal is in front of roger, backward if behind
//			if((x >= 0 && limb == RIGHT) || (x < 0 && limb == LEFT)){
//				*theta1 = theta1_plus;
//				*theta2 = theta2_plus;
//			}
//			else{
//				*theta1 = theta1_minus;
//				*theta2 = theta2_minus;
//			}
//		}
//
		// bend forward if goal is in front of roger, backward if behind
//		if((x >= 0 && limb == RIGHT) || (x < 0 && limb == LEFT)){
//			*theta1 = theta1_plus;
//			*theta2 = theta2_plus;
//		}
//		else{
//			*theta1 = theta1_minus;
//			*theta2 = theta2_minus;
//		}

// use awkward elbows-in posture.
		if((x >= 0 && limb == LEFT) || (x < 0 && limb == RIGHT)){
			*theta1 = theta1_plus;
			*theta2 = theta2_plus;
		}
		else{
			*theta1 = theta1_minus;
			*theta2 = theta2_minus;
		}


    

  	return(1);
  }
  else
  	return(0);
}

/* === Need to be done in Project 2 === */
/* Base PD controller, reference is given in cartesian space */
PDController_base(roger, time) 
Robot * roger;
double time;
{
  int i, j;
  double wTb[4][4], bTw[4][4], e[2], e_dot[2];
  double ref_b[4], ref_w[4], v_b[4], v_w[4];
  double Fx, M;
  double usershift_b[4], usershift_w[4];

  // construct the homogeneous transform from the world to the mobile base
  construct_wTb(roger->base_position, wTb);

  // position feedback
  inv_transform(wTb, bTw);

  if (user_set){ // implementation of the user-initiated experiment

  	usershift_b[X] = user_xy[X];
  	usershift_b[Y] = user_xy[Y];
  	usershift_b[2] = 0;
  	usershift_b[3] = 1.0;

    matXvec(wTb, usershift_b, usershift_w);

    roger->base_setpoint[X] += usershift_w[X];
    roger->base_setpoint[Y] += usershift_w[Y];

  	user_set = 0;
  }

  ref_w[0] = roger->base_setpoint[X];
  ref_w[1] = roger->base_setpoint[Y];
  ref_w[2] = 0.0;
  ref_w[3] = 1.0;

  matXvec(bTw, ref_w, ref_b);

  // "control yoke" - the control offset of the mobile base in base coordinates
  e[X] = ref_b[X] - BASE_CONTROL_OFFSET;
  e[Y] = ref_b[Y];

  v_w[0] = roger->base_velocity[X];
  v_w[1] = roger->base_velocity[Y];
  v_w[2] = 0.0;
  v_w[3] = 0.0; // not a position vector
  matXvec(bTw, v_w, v_b);

  // add influence of angular velocity of body
  v_b[Y] = v_b[Y] + roger->base_velocity[2] * BASE_CONTROL_OFFSET;

  // calculate desired force for X direction
  Fx = Kp_base_Fx * e[X] - Kd_base_Fx * v_b[X];

  // calculate desired moment about base z-axis
  M = BASE_CONTROL_OFFSET * (Kp_base_M * e[Y] - Kd_base_M * (v_b[Y]));

  roger->wheel_torque[LEFT]  = 0.5 * (Fx - M/0.1);
  roger->wheel_torque[RIGHT] = 0.5 * (Fx + M/0.1);
  // check if there's a base setpoint and initialize file i/o
  if(tune_PD && (roger->base_setpoint[X] != last_setX
  			|| roger->base_setpoint[Y] != last_setY)){
  		if(writefile != NULL){// test if the current output file is open
  		// close the open file
  		fclose(writefile);
  		writefile = NULL;
  	}
  	t = 0; // reset clock
  	initialHeading = roger->base_position[2];
  	// create new filename
  	sprintf(filename, "outfile_%d.txt", test_num);
  	// open a new output file
  	writefile = fopen(filename, "w");
  	last_setX = roger->base_setpoint[X];
  	last_setY = roger->base_setpoint[Y];
  	fprintf(writefile, "time\tx error\ty error\ttheta\n"); // write header to file
  	test_num++; // increment test counter
  }

	// print system output every 10 ms
if(tune_PD){
	if (t % 10 == 0 && writefile != NULL){
		if (t == 0){
			fprintf(writefile, "\t\t\t\tKp_Fx:%.2lf\tKd_Fx:%.2lf\n", Kp_base_Fx, Kd_base_Fx);
			fprintf(writefile, "\t\t\t\tKp_M:%.2lf\tKd_M:%.2lf\n", Kp_base_M, Kd_base_M);
		}

		fprintf(writefile, "%ld\t%lf\t%lf\t%lf\n", t, e[X],e[Y],
				roger->base_position[2] - initialHeading);
		// print time, x and y errors, and theta
	}
//	t++; // increment ms counter
	double max_error;
	// close current output file when steady state is reached
	max_error = (fabs(e[X]) > fabs(e[Y]) ? e[X] :	e[Y]);

	if (fabs(max_error) < 0.0001 && writefile != NULL && t > 2500){
			fclose(writefile);
			writefile = NULL;
			printf("test %d complete. t = %ld error = %lf\n", test_num, t, max_error);
	}
}


}

/* === Need to be done in Project 2 === */
//called on cartesian input in arm goal input mode
//use cartesian space input from mouse including mouse button info
Teleoperation_cartesian_input_arms(roger, x, y, button)
Robot* roger;
double x;		//x value
double y;		//y value
int button;		//mouse button
{	
//	theRobot = roger;
  double wTb[4][4], bTw[4][4], ref_b[4], ref_w[4], theta0, theta1;
  int body_side = 0;

  printf("Arm goal input - x: %4.3f, y: %4.3f - button: %d\n", x, y, button);

  //COMPLETE FOR PROJECT #2
  body_side = 0;
  if (button == LEFT_BUTTON) //Left mouse button
    body_side = LEFT;
  else if (button == RIGHT_BUTTON) //Right mouse button
    body_side = RIGHT;
  else
    return;

//  printf("x=%6.4lf y=%6.4lf\n", x, y);
  
  // transform from world to base coords

  // construct the homogeneous transform from the world to the mobile base
  construct_wTb(roger->base_position, wTb);

  // position feedback
  inv_transform(wTb, bTw);

  ref_w[X] = x;
  ref_w[Y] = y;
  ref_w[2] = 0;
  ref_w[3] = 1.0;

  // transform reference coordinates base frame
  matXvec(bTw, ref_w, ref_b);

  if(inv_kinematics(roger, body_side, ref_b[X], ref_b[Y], &theta0, &theta1)){
//  	printf("\narm: %d\ttheta0: %lf\ttheta1: %lf", body_side, theta0, theta1);
  	roger->arm_setpoint[body_side][0] = theta0;
  	roger->arm_setpoint[body_side][1] = theta1;
  	printf("%s arm. theta0: %.3f theta1: %.3f\n", body_side == 0 ? "left" : 
  			"right", theta0, theta1);
	}
  // otherwise, make no changes to the arm setpoints




}

// mouse input is written in base_setpoint goals for the mobile base
/* ==================== DO NOT alter this method =========================== */
Teleoperation_cartesian_input_base(roger, x, y, button)
Robot* roger;
double x;		//x value
double y;		//y value
int button;		//mouse button
{	
	roger->base_setpoint[X] = x;
	roger->base_setpoint[Y] = y;
	printf("Base goal: x=%6.4lf y=%6.4lf\n", x, y);

}

// a homogeneous transform mapping homogeneous position vectors written in
// the base frame into homogeneous vectors in world frame 
/* ==================== DO NOT alter this method =========================== */
construct_wTb(base_pos, wTb)
double base_pos[3]; // (x,y,theta)
double wTb[4][4];
{
  double s0, c0;
  s0 = sin(base_pos[2]);
  c0 = cos(base_pos[2]);

  wTb[0][0] = c0;  wTb[0][1] = -s0; wTb[0][2] = 0.0; wTb[0][3] = base_pos[X];
  wTb[1][0] = s0;  wTb[1][1] = c0;  wTb[1][2] = 0.0; wTb[1][3] = base_pos[Y];
  wTb[2][0] = 0.0; wTb[2][1] = 0.0; wTb[2][2] = 1.0; wTb[2][3] = 0.0;
  wTb[3][0] = 0.0; wTb[3][1] = 0.0; wTb[3][2] = 0.0; wTb[3][3] = 1.0;
}


arm_jacobian(theta1,theta2, Jacobian)
double theta1,theta2;
double Jacobian[2][2];
{
  
}



