#include <PIDController.h>
#include <math.h>
#include "../headers/Functions.h"
#include "../headers/Pins.h"
#include "../headers/Variables.h"

void encoder1()
  {
    
    if(digitalRead(DMOTOR1) == HIGH)
    {
      encoder_pos1++;
    }
    else
    {
      encoder_pos1--;
    }
  }
void encoder2()
  {
    
    if(digitalRead(DMOTOR2) == HIGH)
    {
      encoder_pos2++;
    }
    else
    {
      encoder_pos2--;
    }
  }
  
  void Motor1Clockwise(int power){
  if(power > 100){
  digitalWrite(DIRECTION1M1, HIGH);
  digitalWrite(DIRECTION2M1, LOW);
  analogWrite(PWM1, power);
  }else{
    digitalWrite(DIRECTION1M1, LOW);
    digitalWrite(DIRECTION2M1, LOW);
  }
}

void Motor1CounterClockwise(int power){
  if(power > 100){
  digitalWrite(DIRECTION1M1, LOW);
  digitalWrite(DIRECTION2M1, HIGH);
  analogWrite(PWM1, power);
  
  }else{
    digitalWrite(DIRECTION1M1, LOW);
    digitalWrite(DIRECTION2M1, LOW);
  }
}
void Motor2Clockwise(int power){
  if(power > 100){
  digitalWrite(DIRECTION1M2, HIGH);
  digitalWrite(DIRECTION2M2, LOW);
  analogWrite(PWM2, power);
  }else{
    digitalWrite(DIRECTION1M2, LOW);
    digitalWrite(DIRECTION2M2, LOW);
  }
}

void Motor2CounterClockwise(int power){
  if(power > 100){
  digitalWrite(DIRECTION1M2, LOW);
  digitalWrite(DIRECTION2M2, HIGH);
  analogWrite(PWM2, power);
  
  }else{
    digitalWrite(DIRECTION1M2, LOW);
    digitalWrite(DIRECTION2M2, LOW);
  }
}

//function for stoping the robot
void STOP(void)
{
  int i=1;
  if(i)
  {
  analogWrite(PWM2, -motor_value2);
  analogWrite(PWM1, -motor_value1);
  delay(600);
  i=0;
  }
  else{
      analogWrite(PWM2, 50);
      analogWrite(PWM1, 50);
  }
}


void Move(float &d1, float &d2)
{
   motor_value1 =  pos_pid1.compute(d1);
   motor_value2 =  pos_pid2.compute(d2);
   if(motor_value2 > 0){
       if(motor_value2>v_limit2) motor_value2=v_limit2;
    Motor2CounterClockwise(motor_value2);
   }else{
       if(motor_value2<-v_limit2) motor_value2=-v_limit2;
    Motor2Clockwise(abs(motor_value2));
   }
   if(motor_value1 > 0){
       if(motor_value1>v_limit1) motor_value1=v_limit1;
    Motor1CounterClockwise(motor_value1);
   }else{
       if(motor_value1<-v_limit1) motor_value1=-v_limit1;
    Motor1Clockwise(abs(motor_value1));
   }
}


//functions for rising a flag in case an obstacle exists in front or on the right side of the robot

int obstacle(void)
{
  int flag = 0;
  float duration,distance;
  digitalWrite(TRIGPIN1, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGPIN1, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGPIN1, LOW);
  duration = pulseIn(ECHOPIN1, HIGH);
  distance = duration*0.034/2;
  if(distance < 5) flag = 1;
  else flag = 0;
  return flag;
}

int obstacle_on_the_right(void)
{
    float duration,distance;
    int flag = 0;
    digitalWrite(TRIGPIN2, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGPIN2, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGPIN2, LOW);
    duration = pulseIn(ECHOPIN2, HIGH);
    distance = duration*0.034/2;
    if(distance < 5) flag = 1;
    else flag = 0;
  return flag;
}

void Avoid_obstacle(void)
{
  analogWrite(PWM2, 230);
  analogWrite(PWM1, 230);
}
void Update_location(float &x_current, float &y_current, float &theta)
{
    float phi,d_center;
    d_right = encoder_pos2/(ticks_per_revolution/(3.14*wheel_diameter));
    d_right = encoder_pos1/(ticks_per_revolution/(3.14*wheel_diameter));
    phi = (d_right-d_left)/d_baseline;
    theta+= phi;
    d_center = (d_right+d_left)/2;
    x_current+=d_center*sin(theta);
    y_current+=d_center*cos(theta);
}