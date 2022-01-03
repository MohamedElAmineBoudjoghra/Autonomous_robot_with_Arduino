#include <PIDController.h>
//FUNCTIONS DECLARATIONS
 void encoder1(); //the function encoder is used to convert the binary sequence recieved from the encoder into a useful information expressed in integer number
// using a counter every time an event occures , this information is used after that to regulate either the position the the speed of the motor.
 void encoder2();
 void Motor1Clockwise(int); // rotation in the one direction for the first motor
 void Motor2Clockwise(int); // rotation in the one direction for the second motor
 void Motor1CounterClockwise(int); // rotation in the other direction for the first motor
 void Motor2CounterClockwise(int); // rotation in the other direction for the second motor

 void Move(void); // after executing this function the two motors will rotate in a specific direction based on the sign of encoder_pos1 and 2
 int  obstacle(void); // this function is used to stop the two motors in case of encountring an unexpected obstacle
 int  obstacle_on_the_right(void); 
 void Update_location(float &x_current, float &y_current, float &theta);
 void Avoid_obstacle(void);