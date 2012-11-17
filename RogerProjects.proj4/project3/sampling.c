/*************************************************************************/
/* File:        sampling.c                                               */
/* Description: Methods for sampling from distributions                  */
/* Date:        10-30-2012                                               */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "Roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"

int init_random_seed = TRUE;


sample_heading(heading)
double *heading;
{
	//only initialize random number generator with seed once
	if (init_random_seed == TRUE)
	{
		printf("------------------------------\n");
		srand(time(NULL));
		init_random_seed = FALSE;
	}

	//uniformly sample from -PI to PI
	*heading = (double) rand() / (double) RAND_MAX * M_PI * 2.0 - M_PI;
	return TRUE;
}

