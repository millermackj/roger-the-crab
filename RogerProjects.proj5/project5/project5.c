/*************************************************************************/
/* File:        project5.c                                               */
/* Description: User project #5                                          */
/* Date:        11-20-2012                                               */
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
//char* statenames[4] = {"no reference", "don't care", "unconverged", "converged"};
extern char* statenames[4];
int project5_initialized = FALSE;

project5_control(roger)
Robot* roger;
{
	static int state = 0;

	//make sure walls, dilation, and goals are inserted	
	if (project5_initialized == FALSE)
	{
		project5_init(roger);
		project5_initialized = TRUE;
	}

	//relax harmonic function
	sor(roger);
	
	//execute harmonic function path planner
	//state = primitive4(roger);

	//execute the multi room punch controller
	state = macro2(roger);

	//printf("Current controller state: %d \n", state);
	

}




//----------------------------------------------------------------------------------------------------------------------------------------------
//-----------------Drive configuration and obstacle dilation------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------------



//------------------------
//PROJECT 5: Adjust the arm configuration for driving. The smaller the size of roger, the less you have to dilate the obstacles in the map.
// It might also be good to adjust the ready pose of the arms used in project 4 ("arm_home_predator") as well as choosing the more 
// awkward elbows-in solution for the inverse kinematics.

double arm_home_drive[2][2] = {{(-.941), (-2.349)}, {(.941), (2.349)}};

//PROJECT 5 end
//------------------------


/*
/ DO NOT ALTER
/ Put the robot into a drive configuration which wraps the arms around the body.
*/
drive_configuration(roger)
Robot* roger;
{
	//wrap arms around the body 
   roger->arm_setpoint[LEFT][0] = arm_home_drive[LEFT][0];
	roger->arm_setpoint[LEFT][1] = arm_home_drive[LEFT][1];
	roger->arm_setpoint[RIGHT][0] = arm_home_drive[RIGHT][0];
	roger->arm_setpoint[RIGHT][1] = arm_home_drive[RIGHT][1];

	//point eyes straight ahead
	roger->eyes_setpoint[LEFT] = 0.0;
	roger->eyes_setpoint[RIGHT] = 0.0;
}



/*
/ Given two grid coordinates (indices) in the occupancy grid, calculate and return the distance between the closest points of both grid cells.
/ The size of each cell is XDELTA x YDELTA. 
*/
double cell_distance(xbin1, ybin1, xbin2, ybin2)
int xbin1, ybin1, xbin2, ybin2;
{
	double dist[2] = {0.0, 0.0};	
	
//-------------------------------
//PROJECT 5: Complete the distance calculation in the x and y direction between both cells

	//calculate distance in x direction   
	
	dist[X] = abs(xbin2 - xbin1)*XDELTA; // center distance
	if(dist[X] > 0)
		dist[X] -= XDELTA;
	
	//calculate distance in y direction
	dist[Y] = abs(ybin2 - ybin1)*YDELTA; // center distance
	if(dist[Y] > 0)
		dist[Y] -= YDELTA;
	

//PROJECT 5 end
//-------------------------------

	return sqrt( SQR(dist[X]) + SQR(dist[Y]) );
		
}


#define ROBOT_DILATE_RADIUS (R_TACTILE + R_BASE + 0.01)



dilate_obstacles(roger)
Robot* roger;
{
	int w, i, j;
	int xbin, ybin;
	double x,y, dist;

	int num_obs = 0;
	int num_dilate = 0;

	printf("Dilating obstacles...\n");


	//remove previous dilated obstacles		
	for (i = 0; i < NBINS; ++i) {   // rows
		for (j = 0; j < NBINS; ++j) {   // cols
			if (roger->world_map.occupancy_map[i][j] == DILATED_OBSTACLE) {
				roger->world_map.occupancy_map[i][j] = FREESPACE;
				roger->world_map.potential_map[i][j] = 1.0;
				roger->world_map.color_map[i][j] = 0;
			}			
		}
	}

//-------------------------------
//PROJECT 5: Iterate over the occupancy grid. Whenever you encounter an OBSTACLE, marks all FREESPACE cell in 
//distance < ROBOT_DILATE_RADIUS as DILATED_OBSTACLE with potential = 1.0 and color = LIGHTYELLOW (see example below  
//You need to complete function "cell_distance(xbin1,ybin1, xbin2, ybin2)" first which will return the distance between the closest 
//points of two grid cells. Adjust constant ROBOT_DILATE_RADIUS above to match the circle size that matches your robot.

   //create dilated obstacles		

//	printf("!!!  Obstacle dilation needs to be implemented here!  !!!!\n");


	for (i = 0; i < NBINS; ++i) {   // rows
		for (j = 0; j < NBINS; ++j) {   // cols
			if (roger->world_map.occupancy_map[i][j] == OBSTACLE) {
				
				// we've come upon an obstacle! so start iterating backwards along x direction
				num_obs++;
				
				// iterate backwards in x direction
				for(xbin = i; xbin >= 0 && xbin >= i - ROBOT_DILATE_RADIUS/XDELTA - 1; xbin--){
					// first look in negative y direction for freespace to mark
					for(ybin = j; ybin >= 0 && ybin >= j - ROBOT_DILATE_RADIUS/YDELTA - 1; ybin--){
						if(cell_distance(i, j, xbin, ybin) <= ROBOT_DILATE_RADIUS 
								&& roger->world_map.occupancy_map[xbin][ybin] == FREESPACE){
							num_dilate++;
							roger->world_map.occupancy_map[xbin][ybin] = DILATED_OBSTACLE;
							roger->world_map.potential_map[xbin][ybin] = 1.0;
							roger->world_map.color_map[xbin][ybin] = LIGHTYELLOW;
						}
					}
					// then look in positive direction for freespace to mark
					for(ybin = j; ybin < NBINS && ybin <= j + ROBOT_DILATE_RADIUS/YDELTA + 1; ybin++){
						if(cell_distance(i, j, xbin, ybin) <= ROBOT_DILATE_RADIUS 
								&& roger->world_map.occupancy_map[xbin][ybin] == FREESPACE){
							num_dilate++;
							roger->world_map.occupancy_map[xbin][ybin] = DILATED_OBSTACLE;
							roger->world_map.potential_map[xbin][ybin] = 1.0;
							roger->world_map.color_map[xbin][ybin] = LIGHTYELLOW;
						}
					}
				}
				
				// now look forward in x direction and dilate back and forward in y direction
				for(xbin = i; xbin < NBINS && xbin <= i + ROBOT_DILATE_RADIUS/XDELTA + 1; xbin++){
					// iterate backwards in y direction
					for(ybin = j; ybin >= 0 && ybin >= j - ROBOT_DILATE_RADIUS/YDELTA - 1; ybin--){
						if(cell_distance(i, j, xbin, ybin) <= ROBOT_DILATE_RADIUS 
								&& roger->world_map.occupancy_map[xbin][ybin] == FREESPACE){
							num_dilate++;
							roger->world_map.occupancy_map[xbin][ybin] = DILATED_OBSTACLE;
							roger->world_map.potential_map[xbin][ybin] = 1.0;
							roger->world_map.color_map[xbin][ybin] = LIGHTYELLOW;
						}
					}
					// iterate forwards in y direction
					for(ybin = j; ybin < NBINS && ybin <= j + ROBOT_DILATE_RADIUS/YDELTA + 1; ybin++){
						if(cell_distance(i, j, xbin, ybin) <= ROBOT_DILATE_RADIUS 
								&& roger->world_map.occupancy_map[xbin][ybin] == FREESPACE){
							num_dilate++;
							roger->world_map.occupancy_map[xbin][ybin] = DILATED_OBSTACLE;
							roger->world_map.potential_map[xbin][ybin] = 1.0;
							roger->world_map.color_map[xbin][ybin] = LIGHTYELLOW;
						}
					}
				}
			}			
		}
	}



	printf("Total: %d obstacles, %d freespaces marked as dilated obstacles.\n", num_obs, num_dilate);

//PROJECT 5 end
//-------------------------------

}

//----------------------------------------------------------------------------------------------------------------------------------------------
//----------------Search locations--------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------------


//-------------------------------
//PROJECT 5: Adjust the list of search locations to fit your desired strategy
// by calling "init_search_locations(roger)" you can set/renew these harmonic map goals

	//rows are search locations with x, y in the columns

	#define NUM_SEARCH_LOCATIONS 4
	double search_locations[NUM_SEARCH_LOCATIONS][2] = {{0.0, 1.25}, {1.25, 1.25},{-1.25, 1.25},{-1.25,-1.25}};

//PROJECT 5 5end
//-------------------------------


/*
/ Initializes a set a search locations in the occupancy grid / potentential map
/ Can be called whenever you want to set/renew the search goals
*/
init_search_locations(roger)
Robot* roger;
{
	int i;
	int xbin ,ybin;

	for (i=0; i<NUM_SEARCH_LOCATIONS; i++)
	{
		ybin = round((MAX_Y - search_locations[i][Y])/YDELTA);
		xbin = round((search_locations[i][X] - MIN_X)/XDELTA);

		printf("%d : %4.3f, %4.3f -> %d, %d \n", i, search_locations[i][X], search_locations[i][Y], xbin, ybin );
		roger->world_map.occupancy_map[ybin][xbin] = GOAL;
		roger->world_map.potential_map[ybin][xbin] = 0.0;
		roger->world_map.color_map[ybin][xbin] = GREEN;

	}
}


//----------------------------------------------------------------------------------------------------------------------------------------------
//---------------Harmonic function relaxation and gradient calculation--------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------------


/*
/ DO NOT ALTER
/ Run SOR numerical relaxation of the harmonic map
*/
sor(roger)
Robot * roger; 
{
	int i, j, sor_count=0, converged = FALSE;
	double sor_once();

	while (!converged && (sor_count < 5000)) {
		++sor_count;
		if (sor_once(roger) < THRESHOLD) 
			converged = TRUE;
	}
	if (sor_count > 1){
		printf("completed harmonic function --- %d iterations\n", sor_count);
		int j = 0;
		int i = 0;

		FILE * writefile;
		writefile = fopen("potential_map.tab", "w");
		for(i = 0; i < NBINS; i++){
			fprintf(writefile, "\t%d", i);
		}
		for(j = 0; j < NBINS; j++){
			fprintf(writefile, "\n%d",j);
			for(i = 0; i < NBINS; i++){
				fprintf(writefile, "\t%f", roger->world_map.potential_map[j][i]);
			}
		}
		fclose(writefile);
	}
}


/*
/ one complete backup, only dirichlet boundary conditions
*/
double sor_once(roger)
Robot * roger; 
{
	int i, j, ipos, ineg, jpos, jneg;
	double residual, max, front, back, up, down;

	max = 0.0;

//----------------------
//PROJECT 5:
// iterate over entire map once
// return the  maximum change in the potential value over the entire
// occupancy map as a means of determining convergence
// 
// 0.0 <= roger->world_map.potential_map[ybin][xbin] <= 1.0
	for (j = 0; j < NBINS; j++) {   // cols
		// pad y boundary cells by repeating the boundary edge
		if(j == 0){
			jneg = 0;
			jpos = j + 1;
		}
		else if(j == NBINS - 1){
			jneg = j - 1;
			jpos = j;
		}
		else{
			jneg = j - 1;
			jpos = j + 1;
		}

		for (i = 0; i < NBINS; i++) {   // rows
			if(roger->world_map.occupancy_map[j][i] == FREESPACE){
				if(i == 0){
					ineg = i;
					ipos = i + 1;
				}
				else if(i == NBINS - 1){
					ineg = i - 1;
					ipos = i;
				}
				else{
					ineg = i - 1;
					ipos = i + 1;
				}

				up = roger->world_map.potential_map[jpos][i];
				down = roger->world_map.potential_map[jneg][i];
				back = roger->world_map.potential_map[j][ineg];
				front = roger->world_map.potential_map[j][ipos];

				residual = (back + front + up + down - 4.0*roger->world_map.potential_map[j][i]);

				roger->world_map.potential_map[j][i] += (RELAXATION_FACTOR/4.0)*residual;

				if(fabs(residual) > max)
					max = fabs(residual);
			}
		}
	}
	
//PROJECT 5 end
//----------------------

	return max;
}



/*
/ DO NOT ALTER
/ Given a x,y location, it will use the potential map to compute a gradient returned in 'grad'
/
*/
double compute_gradient(x, y, roger, grad)
double x, y;
Robot * roger;
double grad[2]; // grad = [ d(phi)/dx  d(phi)/dy ] ~ [ d(phi)/dj  -d(phi)/di ]
{
	int i0,i1,j0,j1;
	double mag, dphi_di, dphi_dj, del_x, del_y;

	j0 = (int) ((x-MIN_X)/XDELTA);
	j1 = (j0+1);
	i1 = NBINS - (int) ((y - MIN_Y)/YDELTA); 
	i0 = (i1-1);

	del_x = (x-MIN_X)/XDELTA - j0;
	del_y = (NBINS - (y - MIN_Y)/YDELTA) - i0;

	dphi_dj = ((1.0-del_y)*(roger->world_map.potential_map[i0][j1] -
			roger->world_map.potential_map[i0][j0] ) +
			(del_y)*(roger->world_map.potential_map[i1][j1] -
			roger->world_map.potential_map[i1][j0]  ) );
	dphi_di = ((1.0-del_x)*(roger->world_map.potential_map[i1][j0] -
			roger->world_map.potential_map[i0][j0] ) +
		 	(del_x)*(roger->world_map.potential_map[i1][j1] -
			roger->world_map.potential_map[i0][j1]  ) );

	grad[0] = dphi_dj; grad[1] = -dphi_di;

	mag = sqrt(SQR(grad[0])+SQR(grad[1]));

	if (mag>THRESHOLD) {
		grad[0] /= mag; grad[1] /= mag;
	}
	else {
		grad[0] = grad[1] = 0;
	}
	return(mag);
}


//----------------------------------------------------------------------------------------------------------------------------------------------
//--------------Primitive4 - follow gradient of harmonic function-------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------------

#define CONTROL_STEP 0.75

/*
/ Harmonic function path planner / follower. It will follow a gradient in the potential map 
/ to a goal. Once reached, it will delete the goal and return CONVERGED. If no gradient is 
/ available (no goals or gradient too shallow) it will return NO_REFERENCE. In all other cases
/ it will return UNCONVERGED.
*/

int primitive4(roger)
Robot* roger;
{
	int xbin, ybin;
	double x, y, bb, mag, grad[2]; //, compute_gradient();
	int state;
	double wTb[4][4], bTw[4][4];
	static double ref_b[4] = {BASE_CONTROL_OFFSET, 0, 0, 1.0};
	double ref_w[4];

	state = NO_REFERENCE;
	

//-----------------------
//PROJECT 5: To complete this controller, get the current location of the robot point you are controlling.
// Find out if the corresponding grid cell is a GOAL. If so, stop and delete goal (see code example below) and
// switch state to CONVERGED. Otherwise calculate the gradient of the potential map for the current location 
// and use it to calculate a new setpoint for the base that is CONTROL_STEP in direction of the gradient (UNCONVERGED).
// If there is no gradient, then stop and return set state to NO_REFERENCE.
// Be aware that the occupancy and potential maps are indexed with y value first (occupancy_map[ybin][xbin]).



	 x = roger->base_position[X];
	 y = roger->base_position[Y];
	 ybin = round((MAX_Y - y)/YDELTA);
	 xbin = round((x - MIN_X)/XDELTA);

	 construct_wTb(roger->base_position,wTb);
	 //inv_transform(wTb, bTw);
	 // find world coordinates of base control point
   matXvec(wTb, ref_b, ref_w);

//	 printf("xbin: %d, ybin: %d\n", xbin, ybin);
	 if(roger->world_map.occupancy_map[ybin][xbin] == GOAL){
		//delete current goal from harmonic map
		 roger->world_map.occupancy_map[ybin][xbin] = FREESPACE;
		 roger->world_map.potential_map[ybin][xbin] = 1.0;
		 // relax map again to account for changes
		 sor(roger);
		 state = CONVERGED;
		 printf("goal deleted\n");
		 roger->base_setpoint[X] = ref_w[X];
		 roger->base_setpoint[Y] = ref_w[Y];
	 }
	 else{
		 mag = compute_gradient(x, y, roger, grad);
		 if(mag != 0.0){
			 roger->base_setpoint[X] = x - CONTROL_STEP*(grad[X]) + (ref_w[X]-x);
			 roger->base_setpoint[Y] = y - CONTROL_STEP*(grad[Y]) + (ref_w[Y]-y);
			 state = UNCONVERGED;
		 }
		 else{ // if gradient is zero (below threshold)
			 roger->base_setpoint[X] = ref_w[X];
			 roger->base_setpoint[Y] = ref_w[Y];
		 }
	 }
	
//PROJECT5 end
//-----------------------------

	return state;
}


//----------------------------------------------------------------------------------------------------------------------------------------------
//---------------Macro2 - multi room ball predator ---------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------------

/*
/ Complete 'multi-room-ball-predator' for the red ball
*/
int macro2(roger)
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
			child_states[i] = DONT_CARE;
		}
		init = FALSE;
	}
//	printf("Macro2 state: %s \n", statenames[state]);
//printf("Macro2 state: %s (macro1: %s, primitive4: %s)\n", statenames[state], statenames[child_states[0]], statenames[child_states[1]]);


//------------------------
//PROJECT5: You have complete macro2 to search/track/punch the red ball in the given multi room environment. 
// You can use any of our previous controllers. Depending how many controllers you want to use, you need to adjust 
// the size of child_states above. 
// The sampling method for random headings has been altered to sample 5 headings and then give up. If you are using a controller that uses
// primitive0(), please execute 'reset_sampling_count();' once you hit the ball.
// Likewise, if you are using primitive4(), please execute 'init_search_locations(roger);' once you hit the ball. Executing those two functions
// will reset heading and x/y search locations.

// child states:
//	0		macro1			search headings, approach, punch
// 	1		primitive4	gradient descent to search goals


child_states[0] = macro1(roger); // start by trying search headings from current position

// if we've exhausted search headings
if(child_states[0] == NO_REFERENCE){
	child_states[1] = primitive4(roger); // go to next search goal
	if(child_states[1] == CONVERGED){
		reset_sampling_count(); // trigger heading search again
		printf("sampling count has been reset!\n");
		child_states[0] = macro1(roger);
		printf("macro1 state: %s\n", statenames[child_states[0]]);
		child_states[1] == DONT_CARE;
	}
}
if(child_states[0] == CONVERGED){
	state = CONVERGED;
	init_search_locations(roger); // prepare all search locations for another round
}
if(child_states[0] == UNCONVERGED || child_states[1] == UNCONVERGED)
	state = UNCONVERGED;
//if(child_states[1] == NO_REFERENCE)
//	init_search_locations();

//PROJECT5 end
//-----------------------------
	
	return state;
}






//----------------------------------------------------------------------------------------------------------------------------------------------
// Project 5 functions that do NOT need to be altered
//----------------------------------------------------------------------------------------------------------------------------------------------

/*
/ Clean environment from project 5 specific things.
*/ 
project5_reset(roger)
Robot* roger;
{
	int i, j;

	//clear walls, dilated obstacles, goals
	for (i = 1; i < NBINS-1; ++i) {   // rows
		for (j = 1; j < NBINS-1; ++j) {   // cols
			roger->world_map.occupancy_map[i][j] = FREESPACE;
			roger->world_map.potential_map[i][j] = 1.0;
				
		}
	}
	project5_initialized = FALSE;
}

/*
/ Setup environment for project 5
*/
project5_init(roger)
Robot* roger;
{
	//insert the walls
	draw_room(roger);
	
	//dilate the obstacles
	dilate_obstacles(roger);
	
	//init the search locations
	init_search_locations(roger);

	//change Roger into drive configuration (arms wrapped around his body for small footprint)
	drive_configuration(roger);

	//reset sampling count
	reset_sampling_count();	
}


/*
/ draws the rooms and init potential map
*/
draw_room(roger)
Robot *roger;
{
	int w, i, j;
	int xbin, ybin;
	double x,y, dist;

	//remove dilated obstacles		
	for (i = 0; i < NBINS; ++i) {   // rows
		for (j = 0; j < NBINS; ++j) {   // cols
			if (roger->world_map.occupancy_map[i][j] == DILATED_OBSTACLE) {
				roger->world_map.occupancy_map[i][j] = FREESPACE;
				roger->world_map.potential_map[i][j] = 1.0;
			}			
		}
	}

	//insert walls
	//wall segments: {start_x, start_y, stop_x, stop_y}
	double walls[3][4] = {{0.7, -2.0, 0.0, 2.8}, {-0.7, -0.5, 1.4, 0.0}, {-0.7, -0.5, 0, 0.8 }};

	for (w=0; w<3; w++) 
	{
		if (walls[w][2] != 0.0)
		{
			y = walls[w][1];	
			for (x=walls[w][0]; x<walls[w][0] + walls[w][2]; x+=XDELTA/4.0)
			{
				ybin = (int)((MAX_Y - y)/YDELTA);
				xbin = (int)((x - MIN_X)/XDELTA);
				roger->world_map.occupancy_map[ybin][xbin] = OBSTACLE;
				roger->world_map.potential_map[ybin][xbin] = 1.0;
			}
		}
		else
		{	
			x = walls[w][0];	
			for (y=walls[w][1]; y<walls[w][1] + walls[w][3]; y+=YDELTA/4.0)
			{
				ybin = (int)((MAX_Y - y)/YDELTA);
				xbin = (int)((x - MIN_X)/XDELTA);
				roger->world_map.occupancy_map[ybin][xbin] = OBSTACLE;
				roger->world_map.potential_map[ybin][xbin] = 1.0;
			}
		}
	}

	//make sure potential map is initialized everywhere
	for (i = 0; i < NBINS; ++i) {   // rows
		for (j = 0; j < NBINS; ++j) {   // cols
			switch (roger->world_map.occupancy_map[i][j]) {
				case FREESPACE: 
					roger->world_map.potential_map[i][j] = 1.0;
					break;
				case OBSTACLE:
					roger->world_map.potential_map[i][j] = 1.0;
					break;
				case GOAL:
					roger->world_map.potential_map[i][j] = 0.0;
					break;
				default:
					break;
			}			
		}
	}
}

//use cartesian space input from mouse including mouse button info
project5_cartesian_input(roger, x, y, button)
Robot* roger;
double x;		//x value
double y;		//y value
int button;		//mouse button
{

	printf("Project 5 input - x: %4.3f, y: %4.3f - button: %d\n", x, y, button);	

}

//use configuration space input from mouse including mouse button info and interface side
project5_configuration_input(roger, q1, q2, side, button)
Robot* roger;	
double q1;		//q1 value
double q2;		//q2 value
int side;		//left or right side interface
int button;		//mouse button 
{

	printf("Project 5 input - q1: %4.3f, q2: %4.3f - side: %s, button: %d\n", q1, q2, (side == LEFT  ? "left": "right"), button);

}

// this procedure can be used to prompt for and read user customized input values
project5_enter_params() 
{
	printf("Project 5 enter_params called. \n");

}

#define STEP         0.01

/* do not alter */
//function called when the 'visualize' button on the gui is pressed
project5_visualize(roger)
Robot* roger;
{
	int i, j, xbin, ybin, already_used[NBINS][NBINS];
	double compute_gradient(), mag, grad[2], x, y;
	//void draw_roger(), draw_object(), draw_frames(), mark_used(), draw_history();
	
	printf("Project 5 visualize called. \n");

	// make sure it converged
	sor(roger);
	
	// initialize auxilliary structure for controlling the
	// density of streamlines rendered
	for (j=0; j<NBINS; ++j) {
		for (i=0; i<NBINS; ++i) {
			already_used[i][j] = FALSE;
		}
	}
	
		
	// If [row,col] is FREESPACE and at least one of its neighbors
	// is OBSTACLE, then draw a streamline
	for (i=1;i<(NBINS-1);i+=1) {
		for (j=1;j<(NBINS-1);j+=1) {
			if ((roger->world_map.occupancy_map[i][j] == FREESPACE) &&
				((roger->world_map.occupancy_map[i-1][j-1] == OBSTACLE) ||
				 (roger->world_map.occupancy_map[i-1][j] == OBSTACLE)   ||
				 (roger->world_map.occupancy_map[i-1][j+1] == OBSTACLE) ||
				 (roger->world_map.occupancy_map[i][j-1] == OBSTACLE)   ||
				 (roger->world_map.occupancy_map[i][j+1] == OBSTACLE)   ||
				 (roger->world_map.occupancy_map[i+1][j-1] == OBSTACLE) ||
				 (roger->world_map.occupancy_map[i+1][j] == OBSTACLE)   ||
				 (roger->world_map.occupancy_map[i+1][j+1] == OBSTACLE) || 
				 
				 (roger->world_map.occupancy_map[i-1][j-1] == DILATED_OBSTACLE) ||
				 (roger->world_map.occupancy_map[i-1][j] == DILATED_OBSTACLE)   ||
				 (roger->world_map.occupancy_map[i-1][j+1] == DILATED_OBSTACLE) ||
				 (roger->world_map.occupancy_map[i][j-1] == DILATED_OBSTACLE)   ||
				 (roger->world_map.occupancy_map[i][j+1] == DILATED_OBSTACLE)   ||
				 (roger->world_map.occupancy_map[i+1][j-1] == DILATED_OBSTACLE) ||
				 (roger->world_map.occupancy_map[i+1][j] == DILATED_OBSTACLE)   ||
				 (roger->world_map.occupancy_map[i+1][j+1] == DILATED_OBSTACLE) ) ) 
			{
				
				// follow a stream line
				x = MIN_X + (j+0.5)*XDELTA;
				y = MAX_Y - (i+0.5)*YDELTA;
				ybin = i; xbin = j;
				
				if (!already_used[ybin][xbin]) {
					int loops = 0;
					while ((roger->world_map.occupancy_map[ybin][xbin] != GOAL) &&
						   (loops++ < 1000)) {
						mag = compute_gradient(x, y, roger, grad);
						if (mag < THRESHOLD) {
							//TODO: Prevent uninitialized harmonic map to try to print stream
							//printf("gradient magnitude is too small %6.4lf\n", mag);
						}
						else {
							// printf("gradientmag: %f gx:%f gy:%f stream:%d x:%d y:%d\n", 
							//	   mag, grad[0], grad[1], streamIdx, bin_ti, bin_tj);
							
							x_draw_line(GOAL_COLOR, x, y, x-STEP*grad[0], y-STEP*grad[1]);

							x -= STEP*grad[0];
							y -= STEP*grad[1];
							
							ybin = (int)((MAX_Y-y)/YDELTA);
							xbin = (int)((x-MIN_X)/XDELTA);
						}
					}
					mark_used((i+1), (j+1), already_used);
				}
			}
		}
	}
}
