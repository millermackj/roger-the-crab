/*************************************************************************/
/* File:        UserIO.c                                                 */
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

int init_control_flag = TRUE;

extern double Kp_arm, Kd_arm, Kp_eye, Kd_eye, Kp_base_Fx, Kd_base_Fx,
	Kp_base_M, Kd_base_M;
extern double user_xy[2];
extern int user_set;

// this procedure can be used to prompt for and read user customized
// input values
enter_params()
{
		printf("Kp_base_Fx: %.2lf Kd_base_Fx: %.2lf\nEnter Controller Params "
				"for base PD (Kp_base_Fx  Kd_base_Fx):\n", Kp_base_Fx, Kd_base_Fx);
		scanf("%lf %lf", &Kp_base_Fx, &Kd_base_Fx);

		printf("Kp_base_M: %.2lf Kd_base_M: %.2lf\nEnter Controller Params "
				"for base PD (Kp_base_M  Kd_base_M):\n", Kp_base_M, Kd_base_M);
		scanf("%lf %lf", &Kp_base_M, &Kd_base_M);

		printf("Run test: Translate (t) or Rotate (r)?");

		while(getchar() != '\n');
		char input = getchar();
		while(getchar() != '\n');

		if (input == 't'){
			user_set = 1;
			user_xy[X] = 0.75;
			user_xy[Y] = 0;
		}
		else if (input == 'r'){
			user_set = 1;
			user_xy[X] = 0;
			user_xy[Y] = 0.75;
		}

//	printf("Kp_arm: %lf Kd_arm: %lf\nEnter Controller Params "
//			"for arm joints (Kp_arm  Kd_arm):\n", Kp_arm, Kd_arm);
//	scanf("%lf %lf", &Kp_arm, &Kd_arm);
//	printf("Kp_eye: %lf Kd_eye: %lf\nEnter controller params "
//			"for eyes (Kp_eye Kd_eye):\n", Kp_eye, Kd_eye);
//	scanf("%lf %lf", &Kp_eye, &Kd_eye);
	printf("%c: Done with user input.\n", input);
}

// handles mode button in the GUI - cycles through your control modes
// you shouldn't ever have to change any of this - messing with it may cause
// strange errors
int change_input_mode() 
{
  static int input_mode;

  input_mode = (input_mode + 1) % N_INPUT_MODES;
  //  printf("input_mode=%d\n", input_mode);
  return (input_mode);
}

int change_control_mode() 
{
  static int control_mode;

  control_mode = (control_mode + 1) % N_CONTROL_MODES;
  //  printf("control_mode=%d\n", control_mode);
  init_control_flag = TRUE;
  return (control_mode);
}






