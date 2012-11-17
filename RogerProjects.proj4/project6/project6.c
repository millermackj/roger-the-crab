/*************************************************************************/
/* File:        project6.c                                               */
/* Description: User project #6                                          */
/* Date:        01-29-2011                                               */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include "Xkw/Xkw.h"

#include "Roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"


project6_control(roger)
Robot* roger;
{

}

project6_reset(roger)
Robot* roger;
{

}

//use cartesian space input from mouse including mouse button info
project6_cartesian_input(roger, x, y, button)
Robot* roger;
double x;		//x value
double y;		//y value
int button;		//mouse button
{

	printf("Project 6 input - x: %4.3f, y: %4.3f - button: %d\n", x, y, button);

}

//use configuration space input from mouse including mouse button info and interface side
project6_configuration_input(roger, q1, q2, side, button)
Robot* roger;	
double q1;		//q1 value
double q2;		//q2 value
int side;		//left or right side interface
int button;		//mouse button 
{

	printf("Project 6 input - q1: %4.3f, q2: %4.3f - side: %s, button: %d\n", q1, q2, (side == LEFT ? "left": "right"), button);

}

// this procedure can be used to prompt for and read user customized input values
project6_enter_params() 
{
	printf("Project 6 enter_params called. \n");
}


/* do not alter */
//function called when the 'visualize' button on the gui is pressed
project6_visualize(roger)
Robot* roger;
{

}

