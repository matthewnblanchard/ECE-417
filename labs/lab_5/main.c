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
        float theta_cur[5] = {0, 135, 135, 0, 0};   // Current angles
        float theta_tar[5];                         // Target angles
        int i = 0;                                  // Loop index

        init();
        zero();

        printf("Welcome to the joint angle control tester.\n");
        printf("Please enter values for theta.\n");

        while(1) {

                // Get joint angles
                for (i = 0; i < 5; i++) {
                        printf("Please enter theta %d:\r\n", i);
                        scanf("%f", &theta_tar[i]);
			printf("\r\nMoving...\r\n");
                }

		// Move the robot        
                robot_set_joint_angles(theta_cur, theta_tar);

		// Save the new joint angles
		for (i = 0; i < 5; i++) {
			theta_cur[i] = theta_tar[i];
		}
		
		printf("Done!\r\n\r\n");
        }

}
