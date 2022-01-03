#include <PIDController.h>
#include <math.h>
#include "../headers/Functions.h"
#include "../headers/Pins.h"
#include "../headers/Variables.h"
 
void setup()
{
  //defining pin types
  pinMode(DMOTOR1,INPUT);
  pinMode(DMOTOR1D,INPUT);
  pinMode(DIRECTION1M1, OUTPUT);
  pinMode(DIRECTION2M1, OUTPUT);
  pinMode(PWM1,OUTPUT);
  pinMode(DMOTOR2,INPUT);
  pinMode(DMOTOR2D,INPUT);
  pinMode(DIRECTION1M2, OUTPUT);
  pinMode(DIRECTION2M2, OUTPUT);
  pinMode(PWM2,OUTPUT);
  pinMode(TRIGPIN1, OUTPUT);
  pinMode(ECHOPIN1, INPUT);
  Serial.begin(115200);
  
  // setting the PID controller
  attachInterrupt(digitalPinToInterrupt(DMOTOR1D),encoder1,RISING);
  attachInterrupt(digitalPinToInterrupt(DMOTOR2D),encoder2,RISING);
  pos_pid1.begin();    
  pos_pid1.tune(50, 0 ,5000);    
  pos_pid1.limit(-120, 120);
  pos_pid1.setpoint(0);
  pos_pid2.begin();    
  pos_pid2.tune(50, 0,5200);    
  pos_pid2.limit(-120, 120);
  pos_pid2.setpoint(0);
  delay(100);
}

void loop()
{
    Update_location(x_current,y_current,theta);
    if(obstacle()||obstacle_on_the_right()||((x_current == 0 && y_current == 0)||(x_current == path[counter].x && y_current == path[counter].y)))
    {
        p_target.x=path[counter].x;
        p_target.y=path[counter].y;
        translation_w_1 = sqrt((p_target.x-x_current)*(p_target.x-x_current)+(p_target.y-y_current)*(p_target.y-y_current));
        translation_w_2 = translation_w_1;
        orientation = atan((p_target.y-y_current)/(p_target.x-x_current))-theta;
        rotation_w_1 = (d_baseline/2)*orientation;
        rotation_w_2 = -rotation_w_1;
        Orientation_permission = 1;
        if ((x_current == path[counter].x && y_current == path[counter].y)) counter++;
        
    }
    if(obstacle()) Move(ninety_deg_rotation_1,ninety_deg_rotation_2);
    else if(obstacle_on_the_right()) {
        Avoid_obstacle();
        ninety_deg_rotation_1 = M_PI_2*d_baseline/2;
        ninety_deg_rotation_1 = -M_PI_2*d_baseline/2;
    }
    else if(Orientation_permission) Move(rotation_w_1,rotation_w_2);
    else Move(translation_w_1,translation_w_2);
    
    if(rotation_w_1 == 0 && rotation_w_2 == 0) Orientation_permission = 0;
  delay(20);
}
