#include <PIDController.h>

struct point{
    float x,y;
}point;

//coordinates
point path[5]{{10,10},{20,50},{70,80},{100,20},{-10,20}}; //the path of the robot
point p_target;
volatile float x_current=0,y_current=0; //the current location of the robot

//variables declarations
PIDController pos_pid1;
PIDController pos_pid2;
PIDController error;
volatile long int encoder_pos1 = 0; //the desired distance of the first motor
volatile long int encoder_pos2 = 0; //the desired distance of the second motor
volatile int counter = 0;
int motor_value1;
int motor_value2;
int motor_mode = 1; 
int v_limit1 = 250;
int v_limit2 = 250;
int counter = 0;
int Orientation_permission = 1;
float d_right,d_left;//the distance made by the right and left wheels
float translation_w_1,translation_w_2; //the translation of the robot from the current point to the target point after it rotates
float rotation_w_1,rotation_w_2;
float orientation;
float theta=0;
float ninety_deg_rotation_1 = M_PI_2*d_baseline/2, ninety_deg_rotation_2 = -M_PI_2*d_baseline/2;
