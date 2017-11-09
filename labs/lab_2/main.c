// main.c
// Test program for matrix manipulation functions
// Author: Matthew Blanchard

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <matrix.h>

int main (int argc, char *argv[])
{
        float a[4][4];    // Matrix A
        float b[4][4];    // Matrix B
        float c[4][4];    // Matrix C (resultant)
        float p_a[3];     // Point A
        float p_b[3];     // Point B

        printf("Welcome to matrix subroutine tester.\n");
        printf("Please enter matrix A. Separate columns with a space, and rows with a newline\n");

        read_matrix(a);
        
        printf("Please enter matrix B\n");

        read_matrix(b);

        printf("\nYou have entered the following matrices:\n");
        printf("\nA:\n");
        print_matrix(a);
        printf("\nB:\n");
        print_matrix(b);

        printf("\nThe product of these matrices C = AB:\n");
        h_matrix_mult(a, b, c);
        printf("\nC:\n");
        print_matrix(c);

        printf("\nThe inverse of A = A^-1:\n");
        h_matrix_inv(a, c);
        printf("\nA^-1\n");
        print_matrix(c);

        printf("\nPlease enter a point to be transformed by matrix A:\n");
        read_point(p_b);

        printf("\nYou have enetered the following point:\n");
        printf("\nP_B:\n");
        print_point(p_b);

        printf("\nP_A = A * P_B:\n");
        h_matrix_tform(a, p_b, p_a);
        print_point(p_a);

        printf("\nP_B = A^-1 * P_A:\n");
        h_matrix_tform(c, p_a, p_b);
        print_point(p_b);

        return 0;        
}
