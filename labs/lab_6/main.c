// main.c
// Test program for matrix manipulation functions
// Author: Matthew Blanchard

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <matrix.h>
#include "/usr/local/include/labvolt/inc/labvolt.h"

int main (int argc, char *argv[])
{
	float theta_i[5] = {0, 90, 90, 0, 0};	// Starting angles
	float pos_a[3] = {25, 25, 0};		// Starting cylinder position
	float pos_b[3] = {-25, -25, 0};		// Ending cylinder position
	int i = 0;
	float pos_dest[3] = {0, 0, 0};

	// Start the robot
	printf("Initializing ...\n)");

        init();
        zero();
	//robot_set_joint_angles(theta_i);

        printf("Initialized!\n");

	/*
	while(1) {
		for (i = 0; i < 3; i++) {
			scanf("%f", &pos_dest[i]);
		}
		robot_move_point(pos_dest);

	}
	*/
	
	
	// Move over 10.5cm above position A
	pos_a[2] = 21;			// Adjust height
	robot_move_point(pos_a);	        // Move above object

	gripper_set(0);				// Open gripper
	pos_a[2] = 10.5;				// Adjust height
	robot_move_point(pos_a);	// Move into object
	gripper_set(1);				// Close gripper

	pos_a[2] = 21;			// Adjust height
	robot_move_point(pos_a);	// Move up

	pos_b[2] = 21;			// Adjust height
	robot_move_point(pos_b);	// Move above drop zone
	
	pos_b[2] = 10.5;				// Adjust height
	robot_move_point(pos_b);	// Lower object
	gripper_set(0);				// Release object

	pos_b[2] = 21;			// Adjust height
	robot_move_point(pos_b);	// Raise arm

	nest();					// Return home
	

	return 0;
};
