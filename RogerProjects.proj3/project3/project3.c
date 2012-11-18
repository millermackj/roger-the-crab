/*************************************************************************/
/* File:        project3.c                                               */
/* Description: User project #3                                          */
/* Date:        01-29-2011                                               */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "Xkw/Xkw.h"

#include "Roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"

extern long int t;
// states returned by controllers
enum {
	NO_REFERENCE = 0,
	DONT_CARE, 
	UNCONVERGED,
	CONVERGED
};
#define RAD_PER_PIXEL ((M_PI/2.0)/(double)NPIXELS) // radians per pixel

char* statenames[4] = {"no reference", "don't care", "unconverged", "converged"};
// for data collection to files
extern FILE * writefile;
double last_setX, last_setY;
double sHeading;
extern int new_test; // set to one at the beginning of each test input
extern int test_num; // counter resets at each setpoint
extern long int t; // ms time variable
extern char filename[32];
int firstRun = TRUE;




project3_control(roger)
Robot* roger;
{
	static int state = 0;
	static int prev_state = 0;


//-------------------------------
//PROJECT 3: You can comment in/out one of the primitive controllers below to developping /debugging them. 

	// create a new output file on first loop
	if(firstRun==TRUE){
		firstRun = FALSE;
		if(writefile != NULL){// test if the current output file is open
			// close the open file
			fclose(writefile);
			writefile = NULL;
		}
		// create new filename
		sprintf(filename, "outfile_%d.txt", test_num);
		// open a new output file
		writefile = fopen(filename, "w");

		fprintf(writefile, "Time\tBase Heading\tR_Eye\tL_Eye\tSearch Heading\n"); // write header to file
		test_num++; // increment test counter
	}
	
	prev_state = state;
	//comment in to test primitive controller 0
	//state = primitive0(roger);

	//comment in to test primitive controller 1
	//state = primitive1(roger);

	//execute the combined search track controller
	state = macro0(roger);

	if (t % 5 == 0 && writefile != NULL){
		fprintf(writefile, "%.3lf\t%lf\t%lf\t%lf\t%lf\n", t/1000.0, roger->base_position[2],
				roger->eye_theta[0], roger->eye_theta[1], sHeading);
		// print time, base heading, and eye angles
	}


	//printf("Current controller state: %d, %s \n", state, statenames[state]);




//PROJECT3 end
//-------------------------------


}


// SEARCH - primitive controller p0 will use base and eyes to SEARCH for the red ball while remaining stationary
int primitive0(roger)
Robot* roger;
{
	//internal states
	enum {
		SAMPLE = 0, 
		MOVE
	};
	//internal controller state
	static int controller_state = SAMPLE;

	static double search_heading;
	double current_heading;
	double heading_error_base;
	double arc_length;
	double ref_b[4], wTb[4][4], bTw[4][4], ref_w[4];
	double base_orientation_w[4];

	//state to be returned to outside
	static int state = NO_REFERENCE;
	
	switch (controller_state)
	{
		// get a new search location (heading)
		case (SAMPLE):

			//sample a new location from a distribution (currently uniform)
			if (sample_heading(&search_heading) == FALSE)
			{	
				printf("No more search locations!");
				state = NO_REFERENCE;
				break;
			}
			printf("Picked random heading: %4.3f \n ", search_heading);
			sHeading = search_heading;
			// go to next state
			controller_state = MOVE;
			break;

		// orient to search location using both eyes and base
		case (MOVE):

			state = UNCONVERGED;

		//-------------------------------
		//PROJECT3 - Fill in the control code to orient base and eyes to the desired heading in variable 'search_heading'

	  // construct the homogeneous transform from the world to the mobile base
	  construct_wTb(roger->base_position, wTb);
	  // position feedback
	  //inv_transform(wTb, bTw);

	  //calculate heading error to desired heading 'search_heading'

	  // start with a vector pointing one unit in x-direction of base
//	  ref_b[0] = 1.0;
//	  ref_b[1] = 0;
//	  ref_b[2] = 0;
//	  ref_b[3] = 1.0;
//
//	  // transform to world coordinates
//	  matXvec(wTb, ref_b, ref_w);
//
//	  // translate position vector to world origin
//	  ref_w[0] -= roger->base_position[0];
//	  ref_w[1] -= roger->base_position[1];
//
//	  // calculate heading
//	  current_heading = atan2(ref_w[1], ref_w[0]);
	  current_heading = roger->base_position[2];
	  heading_error_base = search_heading - current_heading;

	  // ensure -pi < error < +pi
	  if(heading_error_base > M_PI)
	  	heading_error_base -= 2.0*M_PI;
	  else if(heading_error_base < -M_PI)
	  	heading_error_base += 2.0*M_PI;

	  // convert heading error into arc-length error
	  arc_length = heading_error_base * 0.7;

	  // determine next setpoint for base

	  // target position in base coordinates is only in the y direction
	  ref_b[0] = BASE_CONTROL_OFFSET;
	  ref_b[1] = arc_length;
	  ref_b[2] = 0;
	  ref_b[3] = 1.0;

	  // transform target coordinate to world frame
	  matXvec(wTb, ref_b, ref_w);


	  //turn base to minimize heading error
	  roger->base_setpoint[0] = ref_w[0];
	  roger->base_setpoint[1] = ref_w[1];

	  //adjust setpoints for eyes
	  roger->eyes_setpoint[LEFT] = roger->eyes_setpoint[RIGHT] = heading_error_base/2.0;

	  if (t %10 == 0)
	  	printf("time:%.3f\tsearch_heading: %f\theading: %f\theading error: %f\n", t/1000.0,search_heading, current_heading, heading_error_base);
	  //if arrived at the desired heading +- some delta, switch back to 'controller_state' 'SAMPLE'
	  //likewise set 'state' to 'CONVERGED' once heading is reached
	  if (heading_error_base > -0.01 && heading_error_base < 0.01){
	  	controller_state = SAMPLE;
	  	state = CONVERGED;
	  }

	  //PROJECT3 end
	  //-------------------------------

			break;

		default:
			break;

	}	
	return state;
}


// find the center of red pixels in both eyes and return TRUE if red was found in both otherwise FALSE
// write center pixel into ul and ur for the respective eye
int compute_average_red_pixel(roger, ur, ul)
Robot * roger;
int *ur, *ul;
{
	int i, j, n[2];
	n[0] = n[1] = *ul = *ur = 0;



//-------------------------------	
	// PROJECT 3: 
	// determine the center of all red pixels in each image view
	// ... 
	// pixel i is red if (roger->image[LEFT][i] == RED)

	for(j = 0;j < 2; j++){
		int start_px = -1;
		int end_px = -1;
		for(i = 0; i < 128; i++){
			// scan until we find a red pixel
			if(roger->image[j][i] == RED){
				if (start_px == -1) // if this is the first red pixel we see,
					start_px = i;			// mark it as the beginning
				end_px = i;
			}
		}
		if(end_px > -1){ // if we've seen a red pixel
			n[j] = (start_px + end_px) / 2; // calculate average red pixel
		}
		else
			return FALSE; // return false if either eye has no red pixels
	}
	*ul = n[LEFT];
	*ur = n[RIGHT];
	return TRUE;
	// PROJECT 3 end	
//-------------------------------
}

// TRACK - primitive controller p1 will TRACK the red ball with base and eyes
int primitive1(roger)
Robot* roger;
{
	int ul, ur;
	double error_eye[2];
	double error_base, arc_length;
	double ref_b[4], wTb[4][4], bTw[4][4], ref_w[4];
	
	//state to be returned to outside
	static int state = NO_REFERENCE;

	//check if ball is in view for both eyes
	if (compute_average_red_pixel(roger, &ur, &ul) == TRUE)
	{
		//first iteration, just switch to unconverged and return
		if (state == NO_REFERENCE)
		{
			state = UNCONVERGED;
			return state;
		}			
		state = UNCONVERGED;



	//-------------------------------
	//PROJECT3: 

		//calculate errors for eyes and base
		if((ul == 63 || ul == 64) && (ur == 63 || ur == 64)) {
			error_eye[LEFT] = error_eye[RIGHT] = 0;
		}
		else{
		  error_eye[LEFT] = (NPIXELS/2 - ul) * RAD_PER_PIXEL;
		  error_eye[RIGHT]= (NPIXELS/2 - ur) * RAD_PER_PIXEL;
		}
//	if(t%10 == 0){
//		printf("err_L: %lf\t err_R: %lf\n", error_eye[LEFT], error_eye[RIGHT]);
//	}


		//adjust eye angles
		  roger->eyes_setpoint[LEFT]  = roger->eye_theta[LEFT]  - error_eye[LEFT];
		  roger->eyes_setpoint[RIGHT] = roger->eye_theta[RIGHT] - error_eye[RIGHT];

		  //error_base = -(roger->eye_theta[LEFT]-error_eye[LEFT]+roger->eye_theta[RIGHT]- error_eye[RIGHT])/2.0;
error_base  =0;

	  	// ensure -pi < error < +pi
		  if(error_base > M_PI)
	  		error_base -= 2.0*M_PI;
	  	else if(error_base < -M_PI)
	  		error_base += 2.0*M_PI;



		  if(fabs(error_base) < 0.05)
			  	error_base = 0;
		  {
		  	// construct the homogeneous transform from the world to the mobile base
		  	construct_wTb(roger->base_position, wTb);
		  	// position feedback
		  	//inv_transform(wTb, bTw);

		  	// convert heading error into arc-length error
		  	arc_length = error_base * 0.6;

		  	// determine next setpoint for base

		  	// target position in base coordinates is only in the y direction
		  	ref_b[0] = BASE_CONTROL_OFFSET;
		  	ref_b[1] = -arc_length;
		  	ref_b[2] = 0;
		  	ref_b[3] = 1.0;

		  	// transform target coordinate to world frame
		  	matXvec(wTb, ref_b, ref_w);

		  	//turn base to minimize heading error
		  	roger->base_setpoint[0] = ref_w[0];
		  	roger->base_setpoint[1] = ref_w[1];
		  }

		//change 'state' once eyes and base are centered on the ball
		if(error_eye[LEFT] == 0 && error_eye[RIGHT] == 0 && error_base == 0)
			state = CONVERGED;


	//PROJECT3 end
	//-------------------------------




	}
	else
	{
		//No ball in view -> no reference
		state = NO_REFERENCE;
	}
	return state;
}




//complete search and track of the red ball using primitive controllers
int macro0(roger)
Robot* roger;
{
	static int state = NO_REFERENCE;
	const int num_children = 2;
	static int init = TRUE;
	static int* child_states;
	int i;	

	if (init)
	{
		child_states = (int*) malloc(num_children*sizeof(int));
		for (i=0; i<num_children; i++) 
		{
			child_states[i] = NO_REFERENCE;
		}
		init = FALSE;
	}

//------------------------
//PROJECT3: Complete the case statement below to form the full finite state machine.
//case 0 and case 15 are filled in as examples. 
//Each case statement represents a pair of states variables for both primitive controllers.
//Make sure you understand in which order they are named in the comments.
//For each case, you need to assign both child_states[i] either by getting a state from 
//running the corresponding controller or by directly assigning it. Additionally, you need 
//to set 'state' for each case. 

	//					      TRACK             SEARCH
	switch (states_to_int(child_states, num_children))
	{
		case 0: // NO_REFERENCE, NO_REFERENCE

			//Nothing more to be found
			child_states[0] = primitive0(roger);
			child_states[1] = primitive1(roger);
			state = NO_REFERENCE;
			break;

		case 1: // NO_REFERENCE, DONT_CARE
			child_states[0] = DONT_CARE;
			child_states[1] = primitive1(roger);
			state = NO_REFERENCE;
			break;
			
		case 2: // NO_REFERENCE, UNCONVERGED
		case 3: // NO_REFERENCE, CONVERGED
			child_states[0] = primitive0(roger);
			child_states[1] = primitive1(roger);
			if(child_states[1] == UNCONVERGED)
				printf("time: %lf Macro state: %s (track: %s / search: %s)\n", t/1000.0, statenames[state], statenames[child_states[1]],statenames[child_states[0]]);

			state = NO_REFERENCE;
			break;
		case 4: // DONT_CARE, NO_REFERENCE
		case 5: // DONT_CARE, DONT_CARE
		case 6: // DONT_CARE, UNCONVERGED
		case 7: // DONT_CARE, CONVERGED
			state = DONT_CARE;
			break;
		case 8: // UNCONVERGED, NO_REFERENCE
		case 9: // UNCONVERGED, DONT_CARE
		case 10: // UNCONVERGED, UNCONVERGED
		case 11: // UNCONVERGED, CONVERGED
			child_states[0] = DONT_CARE;
			child_states[1] = primitive1(roger);
			state = UNCONVERGED;
			break;
		case 12: // CONVERGED, NO_REFERENCE
		case 13: // CONVERGED, DONT_CARE
		case 14: // CONVERGED, UNCONVERGED
		case 15: // CONVERGED, CONVERGED
			child_states[0] = DONT_CARE;
			child_states[1] = primitive1(roger);
			state = CONVERGED;
			break;

		default:
			break;


//PROJECT3 end
//-------------------------------


	}
	return state;
}





project3_reset(roger)
Robot* roger;
{

}

//use cartesian space input from mouse including mouse button info
project3_cartesian_input(roger, x, y, button)
Robot* roger;
double x;		//x value
double y;		//y value
int button;		//mouse button
{

	printf("Project 3 input - x: %4.3f, y: %4.3f - button: %d\n", x, y, button);

}

//use configuration space input from mouse including mouse button info and interface side
project3_configuration_input(roger, q1, q2, side, button)
Robot* roger;	
double q1;		//q1 value
double q2;		//q2 value
int side;		//left or right side interface
int button;		//mouse button 
{

	printf("Project 3 input - q1: %4.3f, q2: %4.3f - side: %s, button: %d\n", q1, q2, (side == LEFT ? "left": "right"), button);

}

// this procedure can be used to prompt for and read user customized input values
project3_enter_params() 
{
	printf("Project 3 enter_params called. \n");
}


/* do not alter */
//function called when the 'visualize' button on the gui is pressed
project3_visualize(roger)
Robot* roger;
{

}


//convert the array of child states into one integer value. Each child state (4 possible states) 
//is using 2 bits in the resulting integer (or 1 digit in base 4)
int states_to_int(states, num_states)
int* states;
int num_states;
{
	int i;
	int mult = 1;
	int composite_state = 0;

	for (i=0; i<num_states; i++)
	{
		composite_state += states[i] * mult;
		mult *= 4;		
	}

	return composite_state;
}
