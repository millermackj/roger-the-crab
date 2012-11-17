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

// states returned by controllers
enum {
	NO_REFERENCE = 0,
	DONT_CARE, 
	UNCONVERGED,
	CONVERGED
};

int randomize_ball_position = FALSE;

project4_control(roger)
Robot* roger;
{
	static int state = 0;
	
	//comment in to test eye_triangulation in combination with left mouse click in user input mode!
	/*
	double ball_x, ball_y;
	int ur, ul;
	if (compute_average_red_pixel(roger, &ur, &ul) == TRUE)
	{
		eye_triangulate(roger, ur, ul, &ball_x, &ball_y);
		printf("Ball location: %4.3f, %4.3f \n", ball_x, ball_y);
	}
	*/
	//state = macro0(roger);

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
	
	double wTb[4][4], bTw[4][4], ref_b[4], ref_w[4], ball_b[2];
	double bb, Ltheta0, Ltheta1, Rtheta0, Rtheta1;
	double avg_theta, delta_y, gamma_l, gamma_r;

 // PROJECT4: Complete this method. Use inputs 'ul' and 'ur' together with eye angles to triangulate the 
 // red ball. The calculated position is in base frame and has to be converted to world frame and written 
 // into 'x' and 'y'.



	*x = 0.0;
	*y = 0.0;

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

	   //...




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
	double theta0, theta1;
	int hand_force[2];
	double fx, fy;
	static int count = 0; 

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
	
		//...
	

		//check if in reach (inv_kinematics will return TRUE)
		
		
		//calculate arm endpoint force vector length			
		//hand_force[LEFT] = hand_ext_forces(roger, LEFT, &fx, &fy);
		//...
			
		

	  //PROJECT4 end
	  //---------------------	
	}

	if (state == NO_REFERENCE)
	{
		//nothing to hit -> send arms to ready position
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
	printf("Macro state: %d (Search-Track: %d / Approach: %d / Punch: %d)\n", state, child_states[0], child_states[1], child_states[2]);


//------------------------
//PROJECT4: You have complete macro1 to search/track the red ball, then approach and punch it. To do this you have to use macro0, 
//primitive2 and primitive3 appropriately. As the number of available controllers is increased to 3, we'd have to
//consider a total of 64 cases. In project3, you had to complete the case statement in macro0() consisting 16 different cases.
//As you hopefully noticed, many of those states could be aggregated into one. Therefore, here you are free to choose on your own
//which child_states[i] you should base your state on. Make sure that each time macro1() is executed, all 3 child_states[i] are updated
//either through assigning DONT_CARE or running the appropriate controller-- just like in project3. Likewise, you also need to update 'state'  

	
// ...



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
					printf("Touching obstacle. \n");
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
		printf("Touching robot base. \n");
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




