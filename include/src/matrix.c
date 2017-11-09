// matrix.c
// Various matrix manipulation functions for use with the Lab-Volt robot
// Author: Matthew Blanchard

#include "../matrix.h"

float param_d[5]        = {27.2, 0, 0, 0, 10.5};
float param_theta[5]    = {0, 0, 0, 90, 0};
float param_a[5]        = {0, 19.2, 19.2, 0, 0};
float param_alpha[5]    = {90, 180, 0, 90, 0};

void h_matrix_mult(float a[4][4], float b[4][4], float c[4][4])
{
        int row = 0;    // Row loop index
        int col = 0;    // Column loop index
        int i = 0;      // Low index
        float sum = 0;    // Sum

        // Interate through each row an column of the product
        for (row = 0; row < 4; row++) {
                for (col = 0; col < 4; col++) {
                        for (i = 0; i < 4; i++) {
                                sum += a[row][i] * b[i][col];   // Iterate through the row for A and the column for B and sum the products
                        }
                        c[row][col] = sum;                      // Save the sum in each element of C
                        sum = 0;
                }        
        }
        return;
};

void h_matrix_inv(float a[4][4], float b[4][4])
{
        int row = 0;    // Row loop index
        int col = 0;    // Column loop index
        int i = 0;      // Loop index
        float sum = 0;    // Running sum of element products (for multiplication)

        // The upper left 3x3 partition is simply R^T
        for (row = 0; row < 3; row++) {
                for (col = 0; col < 3; col++) {
                        b[row][col] = a[col][row];
                }
        }

        // The upper right 1x3 partition is -R^T * t
        for (row = 0; row < 3; row ++) {
                for (i = 0; i < 3; i++) {
                        sum += -(b[row][i] * a[i][3]);
                }
                b[row][3] = sum;
                sum = 0;
        } 

        // Lower left 3x1 partition is 000
        for (i = 0; i < 3; i++) {
                b[3][i] = 0;
        }

        // Lower right 1x1 partition is 1
        b[3][3] = 1;

        return;
};

void h_matrix_tform(float a[4][4], float p_i[3], float p_f[3])
{
        int row = 0;    // Row index
        int i = 0;      // Loop index
        float sum = 0;    // Running sum for element product calculations

        // Interate through each row
        for (row = 0; row < 3; row ++) {

                // Iterate through the columns of A and the rows of p_i. For row 4, use a 1 for p_i 's augmentation
                for (i = 0; i < 3; i++) {
                        sum += a[row][i] * p_i[i];
                }
                sum += a[row][3] * 1;
                p_f[row] = sum;
                sum = 0;
        }

        return;
};

void print_matrix(float m[4][4])
{
        int i = 0;      // Loop index
        int j = 0;      // Loop index

        // Cycle through and print each element
        for (i = 0; i < 4; i++) {
                printf("| ");
                for (j = 0; j < 4; j++) {
                        printf("%+-8f ", m[i][j]);
                }
                printf("|\n");
        }

        return;
};

void read_matrix(float m[4][4])
{

        char *buf;                      // Buffer for input
        char *tok;                      // String token
        int cnt = 0;                    // Count of elements
        int i = 0;                      // Loop index
        const char delim[2] = " ";      // Delimiting character for columns in stdin

        buf = malloc(256);

        // Read 4 rows in
        for (i = 0; i < 4; i++) {
                fgets(buf, 256, stdin);         // Get the line
                tok = strtok(buf, delim);       // Pull out tokens delimited by spaces
                while (tok != NULL) {
                        cnt++;
                        m[i][cnt - 1] = atof(tok);
                        tok = strtok(NULL, delim);
                }
                if (cnt != 4) {
                        printf("Incorrect number of elements, please re-enter the matrix\n");
                        memset(m, 0, 16 * sizeof(float));
                        i = 0;
                }
                cnt = 0;
        }

        free(buf);

        return;
};

void print_point(float p[3])
{ 
        int i = 0;      // Loop index

        // Cycle through and print each element
        for (i = 0; i < 3; i++) {
                printf("| ");
                printf("%+-8f ", p[i]);
                printf("|\n");
        }

        return;
};

void read_point(float p[3])
{
        char *buf;                      // Buffer for input
        char *tok;                      // String token
        int cnt = 0;                    // Count of elements
        int i = 0;                      // Loop index
        const char delim[2] = " ";      // Delimiting character for columns in stdin

        buf = malloc(256);

        // Read 3 rows in
        for (i = 0; i < 3; i++) {
                fgets(buf, 256, stdin);         // Get the line
                tok = strtok(buf, delim);       // Pull out tokens delimited by spaces
                while (tok != NULL) {
                        cnt++;
                        p[i]= atof(tok);
                        tok = strtok(NULL, delim);
                }
                if (cnt != 1) {
                        printf("Incorrect number of elements, please re-enter the matrix\n");
                        memset(p, 0, 3 * sizeof(float));
                        i = 0;
                }
                cnt = 0;
        }

        free(buf);

        return;
};

void param_matrix_fwd(float m[4][4], float d, float a, float alpha, float theta)
{
        // Solve each element
        m[0][0] = cos(RAD(theta));
        m[0][1] = -sin(RAD(theta)) * cos(RAD(alpha));
        m[0][2] = sin(RAD(theta)) * sin(RAD(alpha));
        m[0][3] = a * cos(RAD(theta));

        m[1][0] = sin(RAD(theta));
        m[1][1] = cos(RAD(theta)) * cos(RAD(alpha));
        m[1][2] = -cos(RAD(theta)) * sin(RAD(alpha));
        m[1][3] = a * sin(RAD(theta));

        m[2][0] = 0;
        m[2][1] = sin(RAD(alpha));
        m[2][2] = cos(RAD(alpha));
        m[2][3] = d;

        m[3][0] = 0;
        m[3][1] = 0;
        m[3][2] = 0;
        m[3][3] = 1;

        return;
}

/*
void matrix_5_3_inv(float m[4][4], float theta_4, float theta_5)
{
        m[0][0] = -sin(RAD(theta_4)) * cos(RAD(theta_5));
        m[0][1] = cos(RAD(theta_4)) * cos(RAD(theta_5));
        m[0][2] = sin(RAD(theta_5));
        m[0][3] = 0;

        m[1][0] = sin(RAD(theta_4)) * sin(RAD(theta_5));
        m[1][1] = -cos(RAD(theta_4)) * sin(RAD(theta_5));
        m[1][2] = cos(RAD(theta_5));
        m[1][3] = 0;
        
        m[2][0] = cos(RAD(theta_4));
        m[2][1] = sin(RAD(theta_4));
        m[2][2] = 0;
        m[2][3] = -param_d[4];

        m[3][0] = 0;
        m[3][1] = 0;
        m[3][2] = 0;
        m[3][3] = 1;

        return;
}
*/

void joint_angle_inv(float m[4][4], float theta[5])
{
        // Compute the 4th column of 0_T_3
        float x = m[0][3] - (param_d[4] * m[0][2]);
        float y = m[1][3] - (param_d[4] * m[1][2]);
        float z = m[2][3] - (param_d[4] * m[2][2]);
       
        printf("x=%f, y=%f, z=%f\r\n", x, y, z);

        // Compute a and z_f
        float a = sqrt(pow(x,2) + pow(y,2));
        float z_f = z - param_d[0];

        printf("a=%f, z_f=%f\r\n", a, z_f);

        // Compute angle 1
        theta[0] = DEG(atan2(y, x));

        // Intermediate var to guard against acos(>1) due to floating point percision issues
        float intr = (pow(a,2) + pow(z_f,2) - pow(param_a[1],2) - pow(param_a[2],2)) / (2 * param_a[1] * param_a[2]);
        (intr > 1.0) ? (intr = 0.99999) : (intr = intr);

        // Compute angle 3
        theta[2] = DEG(acos((pow(a,2) + pow(z_f,2) - pow(param_a[1],2) - pow(param_a[2],2)) / (2 * param_a[1] * param_a[2])));

        // Compute angle 2
        theta[1] = DEG(atan2(z_f, a)) + DEG(atan2(param_a[2] * sin(RAD(theta[2])), param_a[1] + (param_a[2] * cos(RAD(theta[2])))));

        // Compute 3_T_0
        float n[4][4];
        n[0][0] = cos(RAD(theta[0])) * cos(RAD(theta[2] - theta[1]));
        n[0][1] = sin(RAD(theta[0])) * cos(RAD(theta[2] - theta[1]));
        n[0][2] = -sin(RAD(theta[2] - theta[1]));
        n[0][3] = -param_a[2] - (param_a[1] * cos(RAD(theta[2]))) + (param_d[0] * sin(RAD(theta[2] - theta[1])));

        n[1][0] = -cos(RAD(theta[0])) * sin(RAD(theta[2] - theta[1]));
        n[1][1] = -sin(RAD(theta[0])) * sin(RAD(theta[2] - theta[1]));
        n[1][2] = -cos(RAD(theta[2] - theta[1]));
        n[1][3] = (param_a[1] * sin(RAD(theta[2]))) + (param_d[0] * cos(RAD(theta[2] - theta[1])));

        n[2][0] = -sin(RAD(theta[0]));
        n[2][1] = cos(RAD(theta[0]));
        n[2][2] = 0;
        n[2][3] = 0;

        n[3][0] = 0;
        n[3][1] = 0;
        n[3][2] = 0;
        n[3][3] = 1;

        // compute 3_T_5
        float o[4][4];
        h_matrix_mult(n, m, o);

        theta[3] = DEG(atan2(o[1][2], o[0][2]));
        theta[4] = DEG(atan2(o[2][0], o[2][1]));

        return;
}
