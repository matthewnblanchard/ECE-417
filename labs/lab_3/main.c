// main.c
// Test program for matrix manipulation functions
// Author: Matthew Blanchard

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <matrix.h>

int main (int argc, char *argv[])
{
        float theta[5];         // Joint angles
        int i = 0;              // Loop index
        float matrix_0_1[4][4]; // Matrix from 0 to 1
        float matrix_1_2[4][4]; // Matrix from 1 to 2
        float matrix_2_3[4][4]; // Matrix from 2 to 3
        float matrix_3_4[4][4]; // Matrix from 3 to 4
        float matrix_4_5[4][4]; // Matrix from 4 to 5
        float matrix_0_2[4][4]; // Matrix from 0 to 2
        float matrix_0_3[4][4]; // matrix from 0 to 3
        float matrix_0_4[4][4]; // Matrix from 0 to 4
        float matrix_0_5[4][4]; // Matrix from 0 to 5
                
        printf("Welcome to the forward kinematics subroutine tester.\n");
        printf("Please enter values for theta.\n");

        // Get joint angles
        for (i = 0; i < 5; i++) {
                printf("Please enter theta %d:\r\n", i);
                scanf("%f", &theta[i]);
        }

        // Calculate matrices
        param_matrix_fwd(matrix_0_1, param_d[0], param_a[0], param_alpha[0], theta[0] + param_theta[0]);
        param_matrix_fwd(matrix_1_2, param_d[1], param_a[1], param_alpha[1], theta[1] + param_theta[1]);
        param_matrix_fwd(matrix_2_3, param_d[2], param_a[2], param_alpha[2], theta[2] + param_theta[2]);
        param_matrix_fwd(matrix_3_4, param_d[3], param_a[3], param_alpha[3], theta[3] + param_theta[3]);
        param_matrix_fwd(matrix_4_5, param_d[4], param_a[4], param_alpha[4], theta[4] + param_theta[4]);

        // Multiply matrices
        h_matrix_mult(matrix_0_1, matrix_1_2, matrix_0_2);
        h_matrix_mult(matrix_0_2, matrix_2_3, matrix_0_3);
        h_matrix_mult(matrix_0_3, matrix_3_4, matrix_0_4);
        h_matrix_mult(matrix_0_4, matrix_4_5, matrix_0_5);

        // Print result
        printf("Transformation matrix from frame 0 to 5:\r\n");
        print_matrix(matrix_0_5);

        return 0;        
}
