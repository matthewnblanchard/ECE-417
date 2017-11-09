// matrix.h
// Various matrix manipulation functions for use with the Lab Volt robots
// Author: Matthew Blanchard

#ifndef _MATRIX_H
#define _MATRIX_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <math.h>

// Trig conversions
#define PI 3.14159265
#define RAD(X) ((X) * PI / 180)
#define DEG(X) ((X) * 180 / PI)

// Lab-Volt Parameters
extern float param_d[5];
extern float param_theta[5];
extern float param_a[5];
extern float param_alpha[5];

// void h_matrix_mult(float a[4][4], float b[4][4], float c[4][4])
// Desc: Comuputes the product of homogeneous matrices a and b and stores it in c
// Args:
//      float a[4][4]: 2-D matrix A
//      float b[4][4]: 2-D matrix B
//      float c[4][4]: 2-D matrix c
// Returns:
//      Product of A & B, stored in c
void h_matrix_mult(float a[4][4], float b[4][4], float c[4][4]);

// void h_matrix_inv(float a[4][4], float b[4][4])
// Desc: Computes the inverse of homogeneous matriux a and stores in in b
// Args:
//      float a[4][4]: 2-D matrix A
//      float b[4][4]: 2-D matrix B
// Returns:
//      Inverse of A, stored in B
void h_matrix_inv(float a[4][4], float b[4][4]);

// void h_matrix_tform(float a[4][4], float p_i[3], float p_f[3]);
// Desc: Transforms the point p_i with T-matrix a and stores it in p_f
// Args:
//      float a[4][4]: 2-D matrix A
//      float p_i[3]: Initial point
//      float p_f[3]: Final point
// Returns:
//      p_f = A * p_i
void h_matrix_tform(float a[4][4], float p_i[3], float p_f[3]);

// void print_matrix(float m[4][4]);
// Desc: Prints a 4x4 matrix to STDOUT
// Args:
//      float m[4][4]: Printed matrix
// Returns:
//      Nothing
void print_matrix(float m[4][4]);

// void read_matrix(float m[4][4]);
// Desc: Reads a 4x4 matrix from STDIN
// Args:
//      float m[4][4]: Storage for read matrix
// Returns:
//      Nothing
void read_matrix(float m[4][4]);

// void print_point(float p[3]);
// Desc: Prints a 3-D point to STDOUT
// Args:
//      float p[3]: Printed point
// Returns:
//      Nothing
void print_point(float p[3]);

// void read_point(float p[3]);
// Desc: Reads a 3-D point from STDIN
// Args:
//      float p[3]: Storage for read point
void read_point(float p[3]);

// void param_matrix_fwd(float m[4][4] float d, float a, float alpha, float theta)
// Desc: Computes the forward kinematics matrix for joint i with respect to
//      joint i-1, given joint i's parameters. The resultant matrix is stored
//      in m
// Args:
//      float d:          Joint parameter d_i
//      float a:          Joint parameter a_i
//      float alpha:      Joint parameter alpha_i
//      float theta:      Joint parameter theta_i
void param_matrix_fwd(float m[4][4], float d, float a, float alpha, float theta);

// void matrix_5_3_inv(float m[4][4], float theta_4, float theta_5)
// Desc: Computes the inverse kinematics matrix from frames 5 to 3
// Args:
//      float theta_4   Joint angle 4
//      float theta_5   Joint angle 5
// void matrix_5_3_inv(float m[4][4], float theta_4, float theta_5);

// void joint_angle_inv(float m[4][4], float theta[5])
// Desc: Computes the joint angles given a transformation
//      matrix from 0 to 5
// Args:
//      float m[4][4]   Transformation matrix from 0 to 5
//      float theta[5]  Joint angles
void joint_angle_inv(float m[4][4], float theta[5]);

#endif
