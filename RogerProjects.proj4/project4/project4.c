/*************************************************************************/
/* File:        project4.c                                               */
/* Description: User project #4                                          */
/* Date:        11-13-2012                                               */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include "Xkw/Xkw.h"

#include "Roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"

#define RAD_PER_PIXEL ((M_PI/2.0)/(double)NPIXELS) // radians per pixel

// states returned by controllers
enum {
	NO_REFERENCE = 0,
	DONT_CARE, 
	UNCONVERGED,
	CONVERGED
};

extern long unsigned int t;

int randomize_ball_position = FALSE;

project4_control(roger)
Robot* roger;
{
	static int state = 0;
	
	//comment in to test eye_triangulation in combination with left mouse click in user input mode!

//	double ball_x, ball_y;
//	int ur, ul;
//	if ( t % 50 == 0 && compute_average_red_pixel(roger, &ur, &ul) == TRUE)
//	{
//		eye_triangulate(roger, ur, ul, &ball_x, &ball_y);
//		printf("Ball location: %4.3f, %4.3f \n", ball_x, ball_y);
//	}

//	state = macro0(roger);

	//comment in to test primitive controller 2
	//state = primitive2(roger);

	//comment in to test primitive controller 3
	//state = primitive3(roger);

	//execute the search track controller
	state = macro1(roger);

	//printf("Current controller state: %d \n", state);

}

/*
/ Use inputs 'ul' and 'ur' together with eye angles to triangulate the 
/ red ball. The calculated position is in base frame and has to be converted 
/ to world frame and written into 'x' and 'y'.
*/
eye_triangulate(roger, ur, ul, x, y)
Robot* roger;
int ur, ul;
double *x, *y;
{
	
	double wTb[4][4], bTw[4][4], ref_b[4], ref_w[4];
//	double ball_b[2];
//	double bb, Ltheta0, Ltheta1, Rtheta0, Rtheta1;
//	double avg_theta, delta_y, gamma_l, gamma_r;
	double error_eye[2];

	// construct the homogeneous transform from the world to the mobile base
  construct_wTb(roger->base_position, wTb);
  inv_transform(wTb, bTw);
 // PROJECT4: Complete this method. Use inputs 'ul' and 'ur' together with eye angles to triangulate the 
 // red ball. The calculated position is in base frame and has to be converted to world frame and written 
 // into 'x' and 'y'.

  //calculate errors for eyes
	if((ul == 63 || ul == 64) && (ur == 63 || ur == 64)) {
		error_eye[LEFT] = error_eye[RIGHT] = 0;
	}
	else{
	  error_eye[LEFT] = (NPIXELS/2 - ul) * RAD_PER_PIXEL;
	  error_eye[RIGHT]= (NPIXELS/2 - ur) * RAD_PER_PIXEL;
	}

	// use eye angles to triangulate on ball
	double slope_L, slope_R;
	double angle = roger->eye_theta[LEFT]  - error_eye[LEFT];

	slope_L = tan(roger->eye_theta[LEFT]  - error_eye[LEFT]);
	slope_R  = tan(roger->eye_theta[RIGHT] - error_eye[RIGHT]);

	// calculate coordinates of red ball in base frame
	if(fabs(slope_R - slope_L) > 0.0) // avoid divide by zero
		ref_b[0] = (2.0*BASELINE)/(slope_R-slope_L);
	else ref_b[0] = 100;
	ref_b[1] = slope_L * ref_b[LEFT] + BASELINE;
	ref_b[2] = 0;
	ref_b[3] = 1;

	// transform coordinates to world frame
  matXvec(wTb, ref_b, ref_w);
	*x = ref_w[0];
	*y = ref_w[1];



 //PROJECT4 end
 //---------------------	
}

double arm_home_predator[2][2] = {{(11.0*M_PI/24.0), -(5.0*M_PI/6.0)},
			 {-(11.0*M_PI/24.0), (5.0*M_PI/6.0)}};

/*
/ APPROACH - primitive controller p2 will use base to approach the ball while eyes keep tracking the ball
*/
int primitive2(roger)
Robot* roger;
{
	//static double search_heading;
	//double heading_error_base;
	double ref_b[4], wTb[4][4], bTw[4][4], ref_w[4];
	double ball_x, ball_y;
	int ul, ur;
	double error_eye[2];

	//state to be returned to outside
	static int state = NO_REFERENCE;


	//check if ball is in view
	if (compute_average_red_pixel(roger, &ur, &ul) == TRUE)
	{
		state = UNCONVERGED;
	
     //------------------------------
	  //PROJECT4: Being inside this if statement means that the ball is visible in both eyes at 'ul' and 'ur'
     // Use function eye_triangulate() to triangulate the ball position and use the position as target for the 
     // base. Also keep the eyes tracking the ball. HINT: This is VERY similar to primitive1().
	  // Use 'define_base_setpoint(roger, x, y)' to set the base setpoint. It clamps the given position to the 
	  // inside of the map and including a margin to the border.

		eye_triangulate(roger, ur, ul, &ball_x, &ball_y);

		define_base_setpoint(roger, ball_x, ball_y);

		// compute base error and set converged if error's small enough
		if(fabs(ball_x - roger->base_position[X]) < 3.0*R_OBJ
				&& fabs(ball_y - roger->base_position[Y]) < 3.0*R_OBJ)
			state = CONVERGED;

		// keep the eyes tracking the ball
		if((ul == 63 || ul == 64) && (ur == 63 || ur == 64)) {
			error_eye[LEFT] = error_eye[RIGHT] = 0;
		}
		else{
		  error_eye[LEFT] = (NPIXELS/2 - ul) * RAD_PER_PIXEL;
		  error_eye[RIGHT]= (NPIXELS/2 - ur) * RAD_PER_PIXEL;
		}

		// assign eye setpoints
	  roger->eyes_setpoint[LEFT]  = roger->eye_theta[LEFT]  - error_eye[LEFT];
	  roger->eyes_setpoint[RIGHT] = roger->eye_theta[RIGHT] - error_eye[RIGHT];

     //PROJECT4 end
	  //---------------------	
	}
	else
	{
		state = NO_REFERENCE;
	}
	
	return state;
}

/*
/ PUNCH - primitive controller p3 will use arms to punch the ball
*/
int primitive3(roger)
Robot* roger;
{
	double ref_b[4], wTb[4][4], bTw[4][4], ref_w[4];
	double ball_x, ball_y;
	int ul, ur;
	double error_eye[2];
	double arm_force[2];
	//double theta_L0, theta_L1, theta_R0, theta_R1;
	double theta0, theta1;
	double dist_L, dist_R; // distance from hand to ball
	int hand_force[2];
	double fx, fy;
	static int punch_limb = LEFT; // arm to use for punching

	static double punch_vector[2][2] = {{1.5*LARM_1, -LARM_1}, {1.5*LARM_1, LARM_1}};
	static double home_vector[2][2] = {{0.25, 0.2},{0.25,-0.2}};
	double punch_vector_w[4];
	static int isPunching  = 0;
	static int punch_time = 0;
	double punch_duration = 100.0; // duration in ms of punch
	//state to be returned to outside
	static int state = NO_REFERENCE;
	
	state = NO_REFERENCE;

	//check if ball is in view
	if (compute_average_red_pixel(roger, &ur, &ul) == TRUE)
	{		
	  //------------------------------
	  //PROJECT4: 
	  // Being inside this if statement means that the ball is visible in both eyes at 'ul' and 'ur'
     // Use function eye_triangulate() to triangulate the ball position and if in range, touch/punch it.
	  // You know how to do that from project2. Hint: Only punch the ball if it is in front of the robot.
	  // Change 'state' to reflect the appropriate situation. The controller is UNCONVERGED if the ball is in reach.
	  // The controller is CONVERGED if you see an external force at either hand. 
	  // (Function hand_ext_forces(roger, limb, &fx, &fy) will return external
     // forces fx and fy at hand 'limb' (LEFT or RIGHT). It returns TRUE if no self-contact or hand-obstacle
     // contact is occuring, otherwise FALSE.

		//triangulate the ball location
		eye_triangulate(roger, ur, ul, &ball_x, &ball_y);		
	
		// continue chasing ball with base
		define_base_setpoint(roger, ball_x, ball_y);

		// calculate eye-error
		if((ul == 63 || ul == 64) && (ur == 63 || ur == 64)) {
			error_eye[LEFT] = error_eye[RIGHT] = 0;
		}
		else{
		  error_eye[LEFT] = (NPIXELS/2 - ul) * RAD_PER_PIXEL;
		  error_eye[RIGHT]= (NPIXELS/2 - ur) * RAD_PER_PIXEL;
		}

		// assign eye setpoints
	  roger->eyes_setpoint[LEFT]  = roger->eye_theta[LEFT]  - error_eye[LEFT];
	  roger->eyes_setpoint[RIGHT] = roger->eye_theta[RIGHT] - error_eye[RIGHT];

		// construct the homogeneous transform from the world to the mobile base
	  construct_wTb(roger->base_position, wTb);
	  // position feedback
	  inv_transform(wTb, bTw);

	  // ball coordinates in world frame
	  ref_w[0] = ball_x;
	  ref_w[1] = ball_y;
	  ref_w[2] = 0.0;
	  ref_w[3] = 1.0;

		// convert ball coordinates to base coordinates
	  matXvec(bTw, ref_w, ref_b);

//	  if(t%50 == 0)
//	  	printf("ballX_w: %f, ballY_w: %f\nballX_b: %f, ballY_b: %f\n", ref_w[X], ref_w[Y],ref_b[X], ref_b[Y]);

		// check if ball has entered punch zone
	  if(!isPunching && fabs((BASE_CONTROL_OFFSET + 2.0*R_OBJ) - ref_b[X]) <= 2.0*R_OBJ
	  		&& fabs(ref_b[Y]) < 0.5*R_OBJ){
	  	// initialize a new punch
	  	punch_limb = !punch_limb; // alternate punching arms
	  	punch_vector[punch_limb][X] = (ref_b[X] - home_vector[punch_limb][X])*1.5;
	  	punch_vector[punch_limb][Y] = (ref_b[Y] - home_vector[punch_limb][Y])*1.5;
	  	isPunching = 1;
	  	punch_time = 1;
	  }
	  if(isPunching){
		  	// calculate inverse kinematics for next step of punch trajectory
	  	if(inv_kinematics(roger, punch_limb,
	  			home_vector[punch_limb][X] + punch_vector[punch_limb][X] * punch_time/punch_duration,
	  			home_vector[punch_limb][Y] + punch_vector[punch_limb][Y] * punch_time/punch_duration,
	  			&theta0, &theta1)){
	  		// set the arm setpoints
			  roger->arm_setpoint[punch_limb][0] = theta0;
		  	roger->arm_setpoint[punch_limb][1] = theta1;
	  	}

	  	punch_time++;
	  	state = UNCONVERGED;
	  	// check if punch is done
	  	if((double)punch_time > punch_duration){
	  		// reset arm to home position
				roger->arm_setpoint[punch_limb][0] = arm_home_predator[punch_limb][0];
				roger->arm_setpoint[punch_limb][1] = arm_home_predator[punch_limb][1];
				// reset punch_time
				punch_time = 0;
				// stop punching
				isPunching = 0;
				state = CONVERGED;
	  	}
	  }

	  //check if in reach (inv_kinematics will return TRUE)
//	  int left_OK = 0;
//	  int right_OK = 0;
//	  if(ref_b[X] > BASE_CONTROL_OFFSET){
//			left_OK = inv_kinematics(roger, LEFT, ref_b[X]-0.5*R_OBJ,ref_b[Y]+1.25*R_TACTILE,&theta_L0, &theta_L1);
//			right_OK = inv_kinematics(roger, RIGHT, ref_b[X]-0.5*R_OBJ,ref_b[Y]-1.25*R_TACTILE,&theta_R0, &theta_R1);
//	  }
//		if(left_OK){ // is ball within reach of left hand?
//			// calculate distance from hand to ball
////	  	if(t % 50 == 0)
////	  	printf("left within reach\n");
//		  roger->arm_setpoint[LEFT][0] = theta_L0;
//	  	roger->arm_setpoint[LEFT][1] = theta_L1;
//		}
//		else{ // left arm out of reach
//  		// bring (or keep) left arm home
////			if(t % 50 == 0)
////			printf("left not in reach\n");
//			roger->arm_setpoint[LEFT][0] = arm_home_predator[LEFT][0];
//			roger->arm_setpoint[LEFT][1] = arm_home_predator[LEFT][1];
//		}
//
//	  if(right_OK){ // is ball within reach of right hand?
//	  	// calculate distance from hand to ball
////	  	if(t % 50 == 0)
////	  	printf("right within reach\n");
//	  	roger->arm_setpoint[RIGHT][0] = theta_R0;
//	  	roger->arm_setpoint[RIGHT][1] = theta_R1;
//	  }
//	  else{
//  		// bring (or keep) right arm home
////	  	if(t % 50 == 0)
////	  	printf("right not in reach\n");
//	  	roger->arm_setpoint[RIGHT][0] = arm_home_predator[RIGHT][0];
//	  	roger->arm_setpoint[RIGHT][1] = arm_home_predator[RIGHT][1];
//	  }
//
//	  if(!left_OK && !right_OK)
//	  	state = NO_REFERENCE; // arm is out of reach
//	  else
//			state = UNCONVERGED;
// by default, whichever hand(s) can reach the ball will bop it.
		


//		//calculate arm endpoint force vector length
//		hand_force[LEFT] = hand_ext_forces(roger, LEFT, &fx, &fy);
//		hand_force[RIGHT] = hand_ext_forces(roger, RIGHT, &fx, &fy);
//
//		if(!hand_force[LEFT] || !hand_force[RIGHT])
//			state = CONVERGED;

	  //PROJECT4 end
	  //---------------------	
	}
	else{ // return arms to home
		roger->arm_setpoint[LEFT][0] = arm_home_predator[LEFT][0];
		roger->arm_setpoint[LEFT][1] = arm_home_predator[LEFT][1];
		roger->arm_setpoint[RIGHT][0] = arm_home_predator[RIGHT][0];
		roger->arm_setpoint[RIGHT][1] = arm_home_predator[RIGHT][1];
	}
	
	return state;
}

//complete 'predator' for the red ball using primitive controllers 2 and 3, and macro0
int macro1(roger)
Robot* roger;
{
	int state = NO_REFERENCE;	
	const int num_children = 3;
	static int init = TRUE;
	static int* child_states;
	int i;	

	if (init)
	{
		child_states = (int*) malloc(num_children*sizeof(int));
		for (i=0; i<num_children; i++) 
		{
			child_states[i] = DONT_CARE;
		}
		init = FALSE;
	}
	//printf("Macro state: %d (Search-Track: %d / Approach: %d / Punch: %d)\n", state, child_states[0], child_states[1], child_states[2]);


//------------------------
//PROJECT4: You have complete macro1 to search/track the red ball, then approach and punch it. To do this you have to use macro0, 
//primitive2 and primitive3 appropriately. As the number of available controllers is increased to 3, we'd have to
//consider a total of 64 cases. In project3, you had to complete the case statement in macro0() consisting 16 different cases.
//As you hopefully noticed, many of those states could be aggregated into one. Therefore, here you are free to choose on your own
//which child_states[i] you should base your state on. Make sure that each time macro1() is executed, all 3 child_states[i] are updated
//either through assigning DONT_CARE or running the appropriate controller-- just like in project3. Likewise, you also need to update 'state'  

	
// ...
	if(child_states[2] >= UNCONVERGED){
		child_states[0] = DONT_CARE;
		child_states[1] = DONT_CARE;
		child_states[2] = primitive3(roger);
	}
	else{
		if((child_states[0] = macro0(roger)) >= UNCONVERGED){ // ball is being tracked, but maybe not locked in
			if((child_states[1] = primitive2(roger)) >= UNCONVERGED) // base is upon ball
				child_states[2] = primitive3(roger); // try to punch the ball
			else
				child_states[2] = DONT_CARE; // don't try to punch the ball
		}
		else{ // ball is not in sight
			child_states[1] = DONT_CARE;
			child_states[2] = DONT_CARE;
		}
	}



//PROJECT4 end
//-------------------------------



	
	return state;
}

/*
/ Define a setpoint for the base passed in as x and y. 
/ Setpoints are clamped to the inside of the cartesian map including a margin of 0.4m.
*/
define_base_setpoint(roger, x, y)
Robot * roger;
double x, y;
{
  if (x>(MAX_X - 0.4)) x=(MAX_X - 0.4);
  if (x<(MIN_X + 0.4)) x=(MIN_X + 0.4);
  if (y>(MAX_Y - 0.4)) y=(MAX_Y - 0.4);
  if (y<(MIN_Y + 0.4)) y=(MIN_Y + 0.4);
  roger->base_setpoint[X] = x;
  roger->base_setpoint[Y] = y;
}


project4_reset(roger)
Robot* roger;
{

}

//use cartesian space input from mouse including mouse button info
project4_cartesian_input(roger, x, y, button)
Robot* roger;
double x;		//x value
double y;		//y value
int button;		//mouse button
{
	printf("Project 4 input - x: %4.3f, y: %4.3f - button: %d\n", x, y, button);
	if (button == LEFT_BUTTON)
	{
		place_object(x, y);
	}	
	else
	{
		place_object_random();
	}
}

//use configuration space input from mouse including mouse button info and interface side
project4_configuration_input(roger, q1, q2, side, button)
Robot* roger;	
double q1;		//q1 value
double q2;		//q2 value
int side;		//left or right side interface
int button;		//mouse button 
{

	printf("Project 4 input - q1: %4.3f, q2: %4.3f - side: %s, button: %d\n", q1, q2, (side == LEFT ? "left": "right"), button);

}

// this procedure can be used to prompt for and read user customized input values
project4_enter_params() 
{
	printf("Project 4 enter_params called. \n");
}


/* do not alter */
//function called when the 'visualize' button on the gui is pressed
project4_visualize(roger)
Robot* roger;
{

}


/* 
/ DO NOT ALTER!
/ check if current external forces at the selected hand are due to own body collision 
/ or collision with obstacles. Returns FALSE if the hand touches the other hand, the base, or
/ environment obstacles.
*/
int hand_ext_forces(roger, limb, fx, fy)
Robot* roger;
int limb;
double *fx, *fy;
{
	double x, y, ox, oy;
	double dist[2];
	int row, col;	
 
	//printf("hand ext forces: %d \n", limb);

	*fx = roger->ext_force[limb][X];
	*fy = roger->ext_force[limb][Y];
	
	if (*fx == 0.0 && *fy == 0.0)
	{
		return TRUE;
	}


	//calculate hand endpoint position
	endpoint_pos(roger, limb, &x, &y);

	//check if hand is close to obstacle
	
	//loop over map and check if element is obstacle		
	for (row = 0; row < NBINS; row++)
	{	
		for (col = 0; col < NBINS; col++) 
		{ 
			if (roger->world_map.occupancy_map[row][col] == OBSTACLE)
			{
				//cell center location
				dist[X] = (MIN_X + (col + 1) * XDELTA) - x;
				dist[Y] = (MIN_Y + (row + 1) * YDELTA) - y;

				if ( fabs(dist[X]) < R_TACTILE + XDELTA && fabs(dist[Y]) < R_TACTILE + YDELTA )
				{
					//printf("Touching obstacle. \n");
					return FALSE;
				}
			}
		}
	}

	//check if hand is close to body
	dist[X] = roger->base_position[X] - x;
	dist[Y] = roger->base_position[Y] - y;
	
	if (sqrt(SQR(dist[X]) + SQR(dist[Y])) < R_BASE + R_TACTILE)
	{
		if(t%50 == 0)
		printf("Touching robot base: \n");
		return FALSE;
	}

	//check if hand is close to other hand
	if (limb == LEFT)	{
		endpoint_pos(roger, RIGHT, &ox, &oy);	
	} else {
		endpoint_pos(roger, LEFT, &ox, &oy);	
	}

	dist[X] = ox - x;
	dist[Y] = oy - y;
	
	if (sqrt(SQR(dist[X]) + SQR(dist[Y])) < R_TACTILE + R_TACTILE)
	{
		printf("Touching hands. \n");	
		return FALSE;
	}

	return TRUE;
}

/*
/ DO NOT ALTER!
/ Calculate the cartesian position of the selected hand in world frame. 
/ Coordinates are written into x and y
*/
endpoint_pos(roger, limb, x, y)
Robot* roger;
int limb;
double *x, *y;
{
	double hand_b[4], hand_w[4];
	double wTb[4][4];
	
	if (limb != LEFT && limb != RIGHT)
		return;

	hand_b[X] = cos(roger->arm_theta[limb][0]) * LARM_1 + cos(roger->arm_theta[limb][0] + roger->arm_theta[limb][1]) * LARM_2;
	hand_b[Y] = sin(roger->arm_theta[limb][0]) * LARM_1 + sin(roger->arm_theta[limb][0] + roger->arm_theta[limb][1]) * LARM_2;
	hand_b[2] = 0.0;
	hand_b[3] = 1.0;  
   
	if (limb == LEFT)
		hand_b[Y] += ARM_OFFSET;
	else
		hand_b[Y] -= ARM_OFFSET;
		
	//convert to world frame
	construct_wTb(roger->base_position, wTb);	  
	matXvec(wTb, hand_b, hand_w);

	
	*x = hand_w[X];
	*y = hand_w[Y];	
}




